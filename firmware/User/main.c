/**
  ******************************************************************************
  * @file    main.c
  * @author  embedfire
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "stm32f4xx.h"
#include "bsp_led.h"
#include "bsp_key.h"
#include "bsp_beep.h"  
#include "bsp_uart1.h"
#include "bsp_dwt.h"
#include "bsp_lcd_display.h"
#include "control_glyphs.h"
#include "lcd_draw.h"
#include "lcd_touch.h"

/* M3 moves 65 calibrated 45-degree increments; clamp and open alternate. */
#define LCD_DIAGNOSTIC_MODE         0U
#define UI_STAGED_DIAGNOSTIC        0U
#define MOTOR_OUTPUT_ENABLED       1U
#define ISOLATOR_STATIC_TEST        0U
#define ACTIVE_MOTOR_COUNT          3U
#define TOUCH_REPORT_TIMEOUT_MS    300U

/* Serial remote layer (see the project guide, section 7). */
#define REMOTE_TIMEOUT_MS          1500U
#define REMOTE_MAX_STEP_HZ         10000U

#define MOTOR_COUNT                 3U
#define MOTOR_TIMER_TICK_HZ         20000U
#define MOTOR1_CONTINUOUS_SPEED_HZ  1600U
#define MOTOR_CONTINUOUS_SPEED_HZ   400U

/*
 * M3 travel calibration: 65 x 45 deg = 2925 deg = 8.125 revolutions.
 * SRX02-E is set to 3200 pulse/rev, so 3200 * 2925 / 360 = 26000 pulses
 * (45 deg = 400 pulses, 65 * 400 = 26000). A move takes about 5.4 s at
 * MOTOR3_SPEED_HZ (4800 Hz = 1.5 rev/s = 90 rpm).
 *
 * STEP is generated inside the MOTOR_TIMER_TICK_HZ interrupt, so one step
 * pulse lasts exactly one tick (50 us at 20 kHz) and the highest usable
 * frequency stays well below the tick rate; do not exceed ~10 kHz here.
 */
#define MOTOR3_STEPS               65U
#define MOTOR3_DEG_PER_STEP        45U
#define MOTOR3_PULSES_PER_REV      3200U
#define MOTOR3_TRAVEL_DEGREES      (MOTOR3_STEPS * MOTOR3_DEG_PER_STEP)
#define MOTOR3_TRAVEL_PULSES       ((MOTOR3_PULSES_PER_REV * MOTOR3_TRAVEL_DEGREES) / 360U)
#define MOTOR3_SPEED_HZ             4800U

/*
 * Wiring: the drivers are connected common-anode (STEP+/DIR+ tied to the
 * isolator's +5V, Y outputs to STEP-/DIR-). The isolator output is active
 * high, so the drivers see the inverted level at their own terminals; each
 * STEP pulse still produces exactly one opto current edge, therefore the
 * pulse count and the travel distance are unchanged. Set the isolator output
 * jumper to 5V, otherwise the opto is not fully turned off between pulses.
 */
#define MOTOR1_DIR_INVERTED         1U
#define MOTOR2_DIR_INVERTED         1U
#define MOTOR3_DIR_INVERTED         0U

#define MOTOR1_STEP_PIN             GPIO_PIN_0
#define MOTOR1_DIR_PIN              GPIO_PIN_1
#define MOTOR2_STEP_PIN             GPIO_PIN_2
#define MOTOR2_DIR_PIN              GPIO_PIN_3
#define MOTOR3_STEP_PIN             GPIO_PIN_4
#define MOTOR3_DIR_PIN              GPIO_PIN_5
#define MOTOR_GPIO_PORT             GPIOC

typedef enum
{
    MOTOR_DIR_CCW = 0,
    MOTOR_DIR_CW = 1
} MotorDirection_t;

typedef enum
{
    M3_ALLOW_EITHER = 0,
    M3_ALLOW_CLAMP = 1,
    M3_ALLOW_OPEN = 2
} M3NextAction_t;

typedef struct
{
    uint16_t step_pin;
    uint16_t dir_pin;
    volatile uint32_t phase_accumulator;
    volatile uint32_t pulse_frequency_hz;
    volatile uint32_t pulses_remaining;
    volatile uint8_t running;
    volatile uint8_t step_high;
    volatile uint8_t direction;
    volatile int32_t position;      /* signed pulse count, + = MOTOR_DIR_CW command */
} Motor_t;

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;
    uint8_t label_glyphs[5];
    uint8_t label_length;
} Button_t;

static void SystemClock_Config(void);
void Delay( uint32_t nCount);
static void Motor_GPIO_Config(void);
static void Isolator_StaticTest_Config(void);
static void Isolator_StaticTest_Update(void);
static void Motor_Timer_Config(void);
static void Motor_StartContinuous(uint8_t motor_index, MotorDirection_t direction);
static uint8_t Motor_StartContinuousHz(uint8_t motor_index, MotorDirection_t direction, uint32_t hz);
static uint8_t Motor_StartMove(uint8_t motor_index, int32_t pulses, uint32_t hz);
static void Motor_Stop(uint8_t motor_index);
static uint8_t Motor_StartFixedMove(MotorDirection_t direction);
static void Motor_Service(void);
static void Remote_Service(void);
static void Remote_Execute(char *line);
static void UI_Draw(void);
static void UI_ProcessTouch(void);
static uint8_t UI_PointInButton(uint16_t x, uint16_t y, const Button_t *button);
static void UI_DrawGlyph(uint16_t x, uint16_t y, uint8_t glyph_index, uint16_t background);
static void UI_DrawButton(const Button_t *button, uint16_t color);

TIM_HandleTypeDef htim6;

static Motor_t motors[MOTOR_COUNT] =
{
    {MOTOR1_STEP_PIN, MOTOR1_DIR_PIN, 0U, 0U, 0U, 0U, 0U, MOTOR_DIR_CW, 0},
    {MOTOR2_STEP_PIN, MOTOR2_DIR_PIN, 0U, 0U, 0U, 0U, 0U, MOTOR_DIR_CW, 0},
    {MOTOR3_STEP_PIN, MOTOR3_DIR_PIN, 0U, 0U, 0U, 0U, 0U, MOTOR_DIR_CW, 0}
};

static Button_t buttons[6] =
{
    {20U, 100U, 210U, 145U, {0U, 1U, 2U, 3U}, 4U},        /* 直线推进: M1 CW, hold */
    {250U, 100U, 210U, 145U, {0U, 1U, 4U, 5U}, 4U},       /* 直线回退: M1 CCW, hold */
    {20U, 295U, 210U, 145U, {6U, 7U, 8U, 9U, 10U}, 5U},   /* 顺时针旋转: M2 CW, hold */
    {250U, 295U, 210U, 145U, {11U, 7U, 8U, 9U, 10U}, 5U}, /* 逆时针旋转: M2 CCW, hold */
    {20U, 490U, 210U, 145U, {12U, 13U, 12U, 14U}, 4U},    /* Clamp: M3 CW, 65 x 45 degrees */
    {250U, 490U, 210U, 145U, {12U, 13U, 15U, 16U}, 4U}   /* Open: M3 CCW, 65 x 45 degrees */
};

static uint8_t ui_m3_button_was_down;
static M3NextAction_t m3_next_action = M3_ALLOW_EITHER;
static uint8_t ui_last_state[3] = {0xFFU, 0xFFU, 0xFFU};
static uint8_t ui_last_direction[3] = {0xFFU, 0xFFU, 0xFFU};
static uint8_t ui_last_m3_next_action = 0xFFU;
static uint32_t m3_clamp_count;
static uint32_t m3_open_count;
static uint32_t ui_last_clamp_count;
static uint32_t ui_last_open_count;
static volatile uint32_t ui_last_touch_report_ms;
static uint8_t touch_ready;
static uint16_t ui_raw_x;
static uint16_t ui_raw_y;
static uint8_t ui_raw_valid;

/* Remote (PC / master arm) state. remote_active is set by motion commands. */
static volatile uint8_t remote_active;
static volatile uint32_t remote_last_cmd_ms;
static uint8_t ui_last_remote;
static int32_t axis_min[MOTOR_COUNT] = {INT32_MIN, INT32_MIN, INT32_MIN};
static int32_t axis_max[MOTOR_COUNT] = {INT32_MAX, INT32_MAX, INT32_MAX};

/**
  * @brief  主函数
  * @param  
  * @retval 
  */
int main(void)
{
    /* 初始化系统中断组、系统定时器等 */
    HAL_Init();
    
     /* 配置系统时钟频率 */
    SystemClock_Config();
         
    /* KEY 端口初始化 */
    Key_GPIO_Config();   
        
    /* LED 端口初始化 */
    LED_GPIO_Config();  
      
    /* 串口1初始化 */
    UART1_Config();
    
    /* 初始化DWT */
    DWT_Init();
    
    if (LCD_DIAGNOSTIC_MODE == 0U)
    {
        if (ISOLATOR_STATIC_TEST != 0U)
        {
            Isolator_StaticTest_Config();
        }
        else if (MOTOR_OUTPUT_ENABLED != 0U)
        {
            Motor_GPIO_Config();
        }
    }

    printf("欢迎使用野火开发板 \r\n");
    printf("三电机触摸控制程序启动 \r\n");
        
    /* Initialize the display before drawing. Do not use null LCD callbacks. */
    if (LCD_Draw_Config() != 0U)
    {
        LED_RGBOFF;
        LED_RED;
        printf("LCD initialization failed\r\n");
        while (1)
        {
        }
    }
    LED_RGBOFF;
    LED_GREEN;
    
    if (LCD_DIAGNOSTIC_MODE != 0U)
    {
        uint16_t band_height = LCD_Y_LENGTH / 3U;

        printf("LCD size: %u x %u\r\n", LCD_X_LENGTH, LCD_Y_LENGTH);
        lcd_fill_color(0U, 0U, LCD_X_LENGTH, band_height, RED);
        lcd_fill_color(0U, band_height, LCD_X_LENGTH, band_height, GREEN);
        lcd_fill_color(0U, (uint16_t)(2U * band_height),
                       LCD_X_LENGTH, (uint16_t)(LCD_Y_LENGTH - 2U * band_height), BLUE);
        while (1)
        {
            HAL_Delay(1000);
        }
    }

    if (UI_STAGED_DIAGNOSTIC != 0U)
    {
        uint8_t button_index;

        LED_RGBOFF;
        LED_GREEN;
        lcd_fill_color(0U, 0U, LCD_X_LENGTH, LCD_Y_LENGTH, BLACK);
        for (button_index = 0U; button_index < 6U; button_index++)
        {
            lcd_fill_color(buttons[button_index].x, buttons[button_index].y,
                           buttons[button_index].width, buttons[button_index].height, BLUE);
        }
        HAL_Delay(4000);

        LED_RGBOFF;
        LED_BLUE;
        LCD_DispString_EN(20U, 25U, "STEPPER MOTOR CONTROL", YELLOW, BLACK, &Font16x24);
        HAL_Delay(4000);

        LED_RGBOFF;
        LED_CYAN;
        UI_Draw();
        HAL_Delay(4000);

        LED_RGBOFF;
        LED_PURPLE;
        touch_ready = (LCD_Touch_Config() == 0U) ? 1U : 0U;
        touch_points_num = 0;
        ui_last_touch_report_ms = HAL_GetTick();
        HAL_Delay(4000);

        LED_RGBOFF;
        LED_WHITE;
        Motor_Timer_Config();
        HAL_Delay(4000);

        LED_RGBOFF;
        LED_YELLOW;
        while (1)
        {
            UI_ProcessTouch();
            if ((ui_last_state[0] != motors[0].running) ||
                (ui_last_state[1] != motors[1].running) ||
                (ui_last_state[2] != motors[2].running) ||
                (ui_last_direction[0] != motors[0].direction) ||
                (ui_last_direction[1] != motors[1].direction) ||
                (ui_last_direction[2] != motors[2].direction) ||
                (ui_last_m3_next_action != (uint8_t)m3_next_action) ||
                (ui_last_clamp_count != m3_clamp_count) ||
                (ui_last_open_count != m3_open_count))
            {
                UI_Draw();
            }
            HAL_Delay(10);
        }
    }

    if (ISOLATOR_STATIC_TEST == 0U)
    {
        Motor_Timer_Config();
    }

    /* 初始化屏幕触摸芯片应用 */
    touch_ready = (LCD_Touch_Config() == 0U) ? 1U : 0U;
    touch_points_num = 0;
    ui_last_touch_report_ms = HAL_GetTick();
    
    UI_Draw();
    
    while (1)
    { 
        Remote_Service();
        UI_ProcessTouch();
        if (ISOLATOR_STATIC_TEST != 0U)
        {
            Isolator_StaticTest_Update();
        }

        /* The timer creates pulses. The main loop only handles touch and UI. */
        if ((ui_last_state[0] != motors[0].running) ||
            (ui_last_state[1] != motors[1].running) ||
            (ui_last_state[2] != motors[2].running) ||
            (ui_last_direction[0] != motors[0].direction) ||
            (ui_last_direction[1] != motors[1].direction) ||
            (ui_last_direction[2] != motors[2].direction) ||
            (ui_last_m3_next_action != (uint8_t)m3_next_action) ||
            (ui_last_clamp_count != m3_clamp_count) ||
            (ui_last_open_count != m3_open_count) ||
            (ui_last_remote != remote_active))
        {
            UI_Draw();
        }

        HAL_Delay(10);
    }
}

static void Motor_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin = MOTOR1_STEP_PIN | MOTOR1_DIR_PIN |
                          MOTOR2_STEP_PIN | MOTOR2_DIR_PIN;
    if (ACTIVE_MOTOR_COUNT == MOTOR_COUNT)
    {
        GPIO_InitStruct.Pin |= MOTOR3_STEP_PIN | MOTOR3_DIR_PIN;
    }
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MOTOR_GPIO_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(MOTOR_GPIO_PORT, GPIO_InitStruct.Pin, GPIO_PIN_RESET);
}

static void Isolator_StaticTest_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOC, MOTOR1_STEP_PIN | MOTOR1_DIR_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = MOTOR1_STEP_PIN | MOTOR1_DIR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

static void Isolator_StaticTest_Update(void)
{
    uint8_t active = (motors[0].running != 0U) &&
                     ((uint32_t)(HAL_GetTick() - ui_last_touch_report_ms) <= TOUCH_REPORT_TIMEOUT_MS);

    HAL_GPIO_WritePin(GPIOC, MOTOR1_STEP_PIN,
                      (active && (motors[0].direction == MOTOR_DIR_CW)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, MOTOR1_DIR_PIN,
                      (active && (motors[0].direction == MOTOR_DIR_CCW)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Motor_Timer_Config(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();

    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 84U - 1U;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 50U - 1U;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
        while (1) {}
    }

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    {
        while (1) {}
    }
}

static void Motor_StartContinuous(uint8_t motor_index, MotorDirection_t direction)
{
    uint32_t hz = (motor_index == 0U) ? MOTOR1_CONTINUOUS_SPEED_HZ
                                      : MOTOR_CONTINUOUS_SPEED_HZ;
    (void)Motor_StartContinuousHz(motor_index, direction, hz);
}

/* Continuous run at an explicit step rate (remote FWD/REV). */
static uint8_t Motor_StartContinuousHz(uint8_t motor_index, MotorDirection_t direction,
                                       uint32_t hz)
{
    Motor_t *motor = &motors[motor_index];
    uint8_t dir_high = (direction == MOTOR_DIR_CW) ? 1U : 0U;

    if (motor_index >= ACTIVE_MOTOR_COUNT)
    {
        return 0U;
    }
    if (hz == 0U)
    {
        return 0U;
    }
    if (hz > REMOTE_MAX_STEP_HZ)
    {
        hz = REMOTE_MAX_STEP_HZ;
    }

    if ((motor->running != 0U) && (motor->direction == direction) &&
        (motor->pulses_remaining == 0U))
    {
        motor->pulse_frequency_hz = hz;      /* same direction: just change speed */
        return 1U;
    }

    if (MOTOR_OUTPUT_ENABLED != 0U)
    {
        if (((motor_index == 0U) && (MOTOR1_DIR_INVERTED != 0U)) ||
            ((motor_index == 1U) && (MOTOR2_DIR_INVERTED != 0U)) ||
            ((motor_index == 2U) && (MOTOR3_DIR_INVERTED != 0U)))
        {
            dir_high ^= 1U;
        }
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, motor->dir_pin,
                          (dir_high != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    motor->direction = direction;
    motor->phase_accumulator = 0U;
    motor->pulses_remaining = 0U;
    motor->pulse_frequency_hz = hz;
    motor->running = 1U;
    return 1U;
}

/* Relative move: positive pulses = MOTOR_DIR_CW command, negative = CCW. */
static uint8_t Motor_StartMove(uint8_t motor_index, int32_t pulses, uint32_t hz)
{
    Motor_t *motor = &motors[motor_index];
    MotorDirection_t direction;
    uint8_t dir_high;
    uint32_t magnitude;

    if (motor_index >= ACTIVE_MOTOR_COUNT)
    {
        return 0U;
    }
    if (pulses == 0)
    {
        return 1U;
    }
    if (hz == 0U)
    {
        hz = 1U;
    }
    if (hz > REMOTE_MAX_STEP_HZ)
    {
        hz = REMOTE_MAX_STEP_HZ;
    }

    direction = (pulses > 0) ? MOTOR_DIR_CW : MOTOR_DIR_CCW;
    magnitude = (pulses > 0) ? (uint32_t)pulses : (uint32_t)(-pulses);
    dir_high = (direction == MOTOR_DIR_CW) ? 1U : 0U;

    if (MOTOR_OUTPUT_ENABLED != 0U)
    {
        if (((motor_index == 0U) && (MOTOR1_DIR_INVERTED != 0U)) ||
            ((motor_index == 1U) && (MOTOR2_DIR_INVERTED != 0U)) ||
            ((motor_index == 2U) && (MOTOR3_DIR_INVERTED != 0U)))
        {
            dir_high ^= 1U;
        }
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, motor->dir_pin,
                          (dir_high != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    motor->direction = direction;
    motor->phase_accumulator = 0U;
    motor->pulses_remaining = magnitude;
    motor->pulse_frequency_hz = hz;
    motor->running = 1U;
    return 1U;
}

static void Motor_Stop(uint8_t motor_index)
{
    motors[motor_index].running = 0U;
}

static uint8_t Motor_StartFixedMove(MotorDirection_t direction)
{
    Motor_t *motor = &motors[2];

    if (ACTIVE_MOTOR_COUNT < MOTOR_COUNT)
    {
        return 0U;
    }

    if ((motor->running != 0U) ||
        ((direction == MOTOR_DIR_CW) && (m3_next_action == M3_ALLOW_OPEN)) ||
        ((direction == MOTOR_DIR_CCW) && (m3_next_action == M3_ALLOW_CLAMP)))
    {
        return 0U;
    }

    if (MOTOR_OUTPUT_ENABLED != 0U)
    {
        uint8_t dir_high = (direction == MOTOR_DIR_CW) ? 1U : 0U;

        if (MOTOR3_DIR_INVERTED != 0U)
        {
            dir_high ^= 1U;
        }
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, motor->dir_pin,
                          (dir_high != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    motor->direction = direction;
    motor->phase_accumulator = 0U;
    motor->pulses_remaining = MOTOR3_TRAVEL_PULSES;
    motor->pulse_frequency_hz = MOTOR3_SPEED_HZ;
    motor->running = 1U;
    m3_next_action = (direction == MOTOR_DIR_CW) ? M3_ALLOW_OPEN : M3_ALLOW_CLAMP;
    return 1U;
}

static void Motor_Service(void)
{
    uint8_t motor_index;

    if (remote_active != 0U)
    {
        /* Remote owns M1/M2. No command for 1.5 s -> stop and hand back to LCD. */
        if ((uint32_t)(HAL_GetTick() - remote_last_cmd_ms) > REMOTE_TIMEOUT_MS)
        {
            motors[0].running = 0U;
            motors[1].running = 0U;
            remote_active = 0U;
        }
    }
    else if ((uint32_t)(HAL_GetTick() - ui_last_touch_report_ms) > TOUCH_REPORT_TIMEOUT_MS)
    {
        motors[0].running = 0U;
        motors[1].running = 0U;
    }

    for (motor_index = 0U; motor_index < MOTOR_COUNT; motor_index++)
    {
        Motor_t *motor = &motors[motor_index];

        if (motor_index >= ACTIVE_MOTOR_COUNT)
        {
            continue;
        }

        if (motor->step_high != 0U)
        {
            if (MOTOR_OUTPUT_ENABLED != 0U)
            {
                HAL_GPIO_WritePin(MOTOR_GPIO_PORT, motor->step_pin, GPIO_PIN_RESET);
            }
            motor->step_high = 0U;
        }

        if (motor->running == 0U)
        {
            continue;
        }

        motor->phase_accumulator += motor->pulse_frequency_hz;
        if (motor->phase_accumulator >= MOTOR_TIMER_TICK_HZ)
        {
            motor->phase_accumulator -= MOTOR_TIMER_TICK_HZ;
            if (MOTOR_OUTPUT_ENABLED != 0U)
            {
                HAL_GPIO_WritePin(MOTOR_GPIO_PORT, motor->step_pin, GPIO_PIN_SET);
            }
            motor->step_high = 1U;

            /* Closed-loop position is not available; this is the command count. */
            motor->position += (motor->direction == MOTOR_DIR_CW) ? 1 : -1;

            if (motor->pulses_remaining != 0U)
            {
                motor->pulses_remaining--;
                if (motor->pulses_remaining == 0U)
                {
                    motor->running = 0U;
                }
            }
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        Motor_Service();
    }
}

static uint8_t UI_PointInButton(uint16_t x, uint16_t y, const Button_t *button)
{
    return ((x >= button->x) && (x < (button->x + button->width)) &&
            (y >= button->y) && (y < (button->y + button->height))) ? 1U : 0U;
}

static void UI_DrawGlyph(uint16_t x, uint16_t y, uint8_t glyph_index, uint16_t background)
{
    const uint8_t *glyph = &control_glyphs_32x32[(uint32_t)glyph_index * 128U];
    uint8_t row;
    uint8_t col;

    lcd_open_window(x, y, 32U, 32U);
    for (row = 0U; row < 32U; row++)
    {
        for (col = 0U; col < 32U; col++)
        {
            uint8_t pixel = glyph[(uint16_t)row * 4U + col / 8U] &
                            (uint8_t)(0x80U >> (col % 8U));
            LCD_WRITE_DATA((pixel != 0U) ? WHITE : background);
        }
    }
}

static void UI_DrawButton(const Button_t *button, uint16_t color)
{
    uint16_t text_x = button->x + (button->width - (uint16_t)button->label_length * 32U) / 2U;
    uint16_t text_y = button->y + (button->height - 32U) / 2U;
    uint8_t index;

    lcd_fill_color(button->x, button->y, button->width, button->height, color);
    /*
     * Avoid LCD_Draw_Line here: the example's implementation mishandles
     * decreasing coordinates. Four filled 1-pixel strips are safe borders.
     */
    lcd_fill_color(button->x, button->y, button->width, 1U, WHITE);
    lcd_fill_color(button->x, button->y + button->height - 1U, button->width, 1U, WHITE);
    lcd_fill_color(button->x, button->y, 1U, button->height, WHITE);
    lcd_fill_color(button->x + button->width - 1U, button->y, 1U, button->height, WHITE);
    for (index = 0U; index < button->label_length; index++)
    {
        UI_DrawGlyph((uint16_t)(text_x + (uint16_t)index * 32U), text_y,
                     button->label_glyphs[index], color);
    }
}

static void UI_DrawRawTouch(void)
{
    char position_text[32];

    if (ui_raw_valid == 0U)
    {
        return;
    }

    sprintf(position_text, "RAW X:%4u  Y:%4u", ui_raw_x, ui_raw_y);
    lcd_fill_color(0U, 756U, LCD_X_LENGTH, 32U, BLACK);
    LCD_DispString_EN(20U, 760U, position_text, CYAN, BLACK, &Font16x24);
}

static void UI_Draw(void)
{
    char status_text[48];
    char count_text[48];

    lcd_fill_color(0U, 0U, LCD_X_LENGTH, LCD_Y_LENGTH, BLACK);
    LCD_DispString_EN(20U, 25U, "STEPPER MOTOR CONTROL", YELLOW, BLACK, &Font16x24);

    UI_DrawButton(&buttons[0], (motors[0].running != 0U && motors[0].direction == MOTOR_DIR_CW) ? GREEN : BLUE);
    UI_DrawButton(&buttons[1], (motors[0].running != 0U && motors[0].direction == MOTOR_DIR_CCW) ? GREEN : BLUE);
    UI_DrawButton(&buttons[2], (motors[1].running != 0U && motors[1].direction == MOTOR_DIR_CW) ? GREEN : BLUE);
    UI_DrawButton(&buttons[3], (motors[1].running != 0U && motors[1].direction == MOTOR_DIR_CCW) ? GREEN : BLUE);
    UI_DrawButton(&buttons[4],
                  (motors[2].running != 0U && motors[2].direction == MOTOR_DIR_CW) ? GREEN :
                  (motors[2].running != 0U || m3_next_action == M3_ALLOW_OPEN) ? GRAY : BLUE);
    UI_DrawButton(&buttons[5],
                  (motors[2].running != 0U && motors[2].direction == MOTOR_DIR_CCW) ? GREEN :
                  (motors[2].running != 0U || m3_next_action == M3_ALLOW_CLAMP) ? GRAY : BLUE);

    /* Counts are accepted commands, not measured position or clamp force. */
    sprintf(count_text, "CLAMP:%lu  OPEN:%lu",
            (unsigned long)m3_clamp_count, (unsigned long)m3_open_count);
    LCD_DispString_EN(20U, 646U, count_text, CYAN, BLACK, &Font16x24);

    sprintf(status_text, "M1:%s  M2:%s  M3:%s",
            (motors[0].running != 0U) ? "RUN" : "STOP",
            (motors[1].running != 0U) ? "RUN" : "STOP",
            (motors[2].running != 0U) ? "MOVE" : "STOP");
    if (remote_active != 0U)
    {
        strcat(status_text, " REMOTE");
    }
    LCD_DispString_EN(20U, 680U, status_text, WHITE, BLACK, &Font16x24);
    if (ACTIVE_MOTOR_COUNT < MOTOR_COUNT)
    {
        LCD_DispString_EN(20U, 720U, "M1/M2 TEST; M3 DISABLED", YELLOW, BLACK, &Font16x24);
    }
    else
    {
        if (motors[2].running != 0U)
            sprintf(count_text, "M3 MOVING 65 x 45 DEG");
        else if (m3_next_action == M3_ALLOW_OPEN)
            sprintf(count_text, "M3 NEXT:OPEN 65 x 45 DEG");
        else if (m3_next_action == M3_ALLOW_CLAMP)
            sprintf(count_text, "M3 NEXT:CLAMP 65 x 45 DEG");
        else
            sprintf(count_text, "M3 65 x 45 DEG");
        LCD_DispString_EN(20U, 720U, count_text, YELLOW, BLACK, &Font16x24);
    }
    UI_DrawRawTouch();

    ui_last_state[0] = motors[0].running;
    ui_last_state[1] = motors[1].running;
    ui_last_state[2] = motors[2].running;
    ui_last_direction[0] = motors[0].direction;
    ui_last_direction[1] = motors[1].direction;
    ui_last_direction[2] = motors[2].direction;
    ui_last_m3_next_action = (uint8_t)m3_next_action;
    ui_last_clamp_count = m3_clamp_count;
    ui_last_open_count = m3_open_count;
    ui_last_remote = remote_active;
}

static void UI_ProcessTouch(void)
{
    uint8_t m1_command = 0U;
    uint8_t m2_command = 0U;
    uint8_t m3_command = 0U;
    uint8_t touch_result = 3U;
    uint8_t point_index;
    uint16_t screen_x;
    uint16_t screen_y;

    if (touch_ready != 0U)
    {
        touch_result = LCD_Touch_Get_Points_Data(touch_points, &touch_points_num);

        if (touch_result == 0U)
        {
            ui_last_touch_report_ms = HAL_GetTick();
            if (touch_points_num > 0)
            {
                ui_raw_x = touch_points[0].x;
                ui_raw_y = touch_points[0].y;
                ui_raw_valid = 1U;
                UI_DrawRawTouch();
            }
        }
        else if ((touch_result != 2U) ||
                 ((uint32_t)(HAL_GetTick() - ui_last_touch_report_ms) > TOUCH_REPORT_TIMEOUT_MS))
        {
            touch_points_num = 0;
        }
    }
    else
    {
        touch_points_num = 0;
    }

    for (point_index = 0U; point_index < (uint8_t)touch_points_num; point_index++)
    {
        /* The touch sensor is rotated 90 degrees relative to LCD scan mode 6. */
        if ((touch_points[point_index].x >= LCD_Y_LENGTH) ||
            (touch_points[point_index].y >= LCD_X_LENGTH))
        {
            continue;
        }
        screen_x = touch_points[point_index].y;
        screen_y = (uint16_t)(LCD_Y_LENGTH - 1U - touch_points[point_index].x);

        if (UI_PointInButton(screen_x, screen_y, &buttons[0]) != 0U)
            m1_command = 1U;
        else if (UI_PointInButton(screen_x, screen_y, &buttons[1]) != 0U)
            m1_command = 2U;
        else if (UI_PointInButton(screen_x, screen_y, &buttons[2]) != 0U)
            m2_command = 1U;
        else if (UI_PointInButton(screen_x, screen_y, &buttons[3]) != 0U)
            m2_command = 2U;
        else if (UI_PointInButton(screen_x, screen_y, &buttons[4]) != 0U)
            m3_command = 1U;
        else if (UI_PointInButton(screen_x, screen_y, &buttons[5]) != 0U)
            m3_command = 2U;
    }

    /*
     * In remote mode the PC owns M1/M2. A touch on any button hands control
     * back to the LCD panel; without a touch, leave the motors alone (the old
     * "no touch -> stop" logic must not fight the bridge).
     */
    if (remote_active != 0U)
    {
        if ((m1_command | m2_command | m3_command) != 0U)
        {
            remote_active = 0U;
        }
        else
        {
            return;
        }
    }

    if (m1_command == 1U)
        Motor_StartContinuous(0U, MOTOR_DIR_CW);
    else if (m1_command == 2U)
        Motor_StartContinuous(0U, MOTOR_DIR_CCW);
    else
        Motor_Stop(0U);

    if (ACTIVE_MOTOR_COUNT >= 2U)
    {
        if (m2_command == 1U)
            Motor_StartContinuous(1U, MOTOR_DIR_CW);
        else if (m2_command == 2U)
            Motor_StartContinuous(1U, MOTOR_DIR_CCW);
        else
            Motor_Stop(1U);
    }

    if (ACTIVE_MOTOR_COUNT == MOTOR_COUNT)
    {
        if ((m3_command != 0U) && (ui_m3_button_was_down == 0U))
        {
            if (Motor_StartFixedMove((m3_command == 1U) ? MOTOR_DIR_CW : MOTOR_DIR_CCW) != 0U)
            {
                if (m3_command == 1U)
                    m3_clamp_count++;
                else
                    m3_open_count++;
            }
        }
    }

    if (m3_command != 0U)
    {
        ui_m3_button_was_down = 1U;
    }
    else if ((touch_result == 0U) && (touch_points_num == 0))
    {
        /* Only a fresh, explicit release re-arms the M3 button edge. */
        ui_m3_button_was_down = 0U;
    }
}


static void Remote_Upper(char *text)
{
    while (*text != '\0')
    {
        if ((*text >= 'a') && (*text <= 'z'))
        {
            *text = (char)(*text - 'a' + 'A');
        }
        text++;
    }
}

static int32_t Remote_ToLong(const char *text, int32_t fallback)
{
    char *end = NULL;
    long value;

    if (text == NULL)
    {
        return fallback;
    }
    value = strtol(text, &end, 10);
    if (end == text)
    {
        return fallback;
    }
    return (int32_t)value;
}

static uint32_t Remote_ToHz(const char *text, uint32_t fallback)
{
    int32_t value = Remote_ToLong(text, (int32_t)fallback);

    if (value <= 0)
    {
        return fallback;
    }
    return (uint32_t)value;
}

/*
 * ASCII line protocol, one command per line, "\r\n" or "\n":
 *   PING
 *   STATUS
 *   ZERO
 *   STOP
 *   LOCAL
 *   M1|M2 FWD [Hz] | REV [Hz] | MOVE <pulses> [Hz] | GOTO <pos> [Hz] | STOP
 *   M1|M2 LIMIT <min> <max>
 *   M3 CLAMP | OPEN | STOP | MOVE <pulses> [Hz]
 * Motion commands claim remote control (the LCD panel is ignored until the
 * screen is touched, LOCAL is sent, or 1.5 s pass without any command).
 */
static void Remote_Execute(char *line)
{
    char *argv[6];
    uint8_t argc = 0U;
    char *token;
    uint8_t index;
    int32_t arg;
    uint32_t hz;

    token = strtok(line, " \t\r\n");
    while ((token != NULL) && (argc < 6U))
    {
        Remote_Upper(token);
        argv[argc++] = token;
        token = strtok(NULL, " \t\r\n");
    }
    if (argc == 0U)
    {
        return;
    }

    remote_last_cmd_ms = HAL_GetTick();

    if (strcmp(argv[0], "PING") == 0)
    {
        printf("OK PONG\r\n");
        return;
    }

    if (strcmp(argv[0], "STATUS") == 0)
    {
        printf("OK POS=%ld,%ld,%ld RUN=%u%u%u REMOTE=%u\r\n",
               (long)motors[0].position, (long)motors[1].position, (long)motors[2].position,
               (unsigned)motors[0].running, (unsigned)motors[1].running,
               (unsigned)motors[2].running, (unsigned)remote_active);
        return;
    }

    if (strcmp(argv[0], "ZERO") == 0)
    {
        motors[0].position = 0;
        motors[1].position = 0;
        motors[2].position = 0;
        printf("OK ZERO\r\n");
        return;
    }

    if (strcmp(argv[0], "LOCAL") == 0)
    {
        remote_active = 0U;
        Motor_Stop(0U);
        Motor_Stop(1U);
        printf("OK LOCAL\r\n");
        return;
    }

    if (strcmp(argv[0], "STOP") == 0)
    {
        Motor_Stop(0U);
        Motor_Stop(1U);
        Motor_Stop(2U);
        remote_active = 0U;
        printf("OK STOP\r\n");
        return;
    }

    if ((argv[0][0] != 'M') || (argv[0][1] < '1') || (argv[0][1] > '3') ||
        (argv[0][2] != '\0'))
    {
        printf("ERR UNKNOWN\r\n");
        return;
    }
    index = (uint8_t)(argv[0][1] - '1');

    if (argc < 2U)
    {
        printf("ERR ARG\r\n");
        return;
    }

    if (strcmp(argv[1], "STOP") == 0)
    {
        Motor_Stop(index);
        /*
         * Do not clear remote_active here: the bridge stops one axis while
         * the other may still be running, and dropping remote mode would let
         * the 300 ms touch timeout stop the other axis too. Use "STOP" or
         * "LOCAL" to hand control back.
         */
        printf("OK M%u STOP\r\n", (unsigned)(index + 1U));
        return;
    }

    if (index == 2U)
    {
        if ((strcmp(argv[1], "CLAMP") == 0) || (strcmp(argv[1], "OPEN") == 0))
        {
            MotorDirection_t dir = (strcmp(argv[1], "CLAMP") == 0) ? MOTOR_DIR_CW : MOTOR_DIR_CCW;

            if (Motor_StartFixedMove(dir) == 0U)
            {
                printf("ERR M3 BUSY\r\n");
                return;
            }
            if (dir == MOTOR_DIR_CW)
            {
                m3_clamp_count++;
            }
            else
            {
                m3_open_count++;
            }
            remote_active = 1U;
            printf("OK M3 %s %lu\r\n", (dir == MOTOR_DIR_CW) ? "CLAMP" : "OPEN",
                   (unsigned long)MOTOR3_TRAVEL_PULSES);
            return;
        }
        if (strcmp(argv[1], "MOVE") == 0)
        {
            if (argc < 3U)
            {
                printf("ERR ARG\r\n");
                return;
            }
            arg = Remote_ToLong(argv[2], 0);
            hz = Remote_ToHz((argc > 3U) ? argv[3] : NULL, MOTOR3_SPEED_HZ);
            (void)Motor_StartMove(2U, arg, hz);
            remote_active = 1U;
            printf("OK M3 MOVE %ld %lu\r\n", (long)arg, (unsigned long)hz);
            return;
        }
        printf("ERR UNKNOWN\r\n");
        return;
    }

    if ((strcmp(argv[1], "FWD") == 0) || (strcmp(argv[1], "REV") == 0))
    {
        MotorDirection_t dir = (strcmp(argv[1], "FWD") == 0) ? MOTOR_DIR_CW : MOTOR_DIR_CCW;
        uint32_t default_hz = (index == 0U) ? MOTOR1_CONTINUOUS_SPEED_HZ
                                            : MOTOR_CONTINUOUS_SPEED_HZ;

        hz = Remote_ToHz((argc > 2U) ? argv[2] : NULL, default_hz);
        if (Motor_StartContinuousHz(index, dir, hz) == 0U)
        {
            printf("ERR M%u\r\n", (unsigned)(index + 1U));
            return;
        }
        remote_active = 1U;
        printf("OK M%u %s %lu\r\n", (unsigned)(index + 1U),
               (dir == MOTOR_DIR_CW) ? "FWD" : "REV", (unsigned long)hz);
        return;
    }

    if (strcmp(argv[1], "MOVE") == 0)
    {
        if (argc < 3U)
        {
            printf("ERR ARG\r\n");
            return;
        }
        arg = Remote_ToLong(argv[2], 0);
        hz = Remote_ToHz((argc > 3U) ? argv[3] : NULL, 800U);
        (void)Motor_StartMove(index, arg, hz);
        remote_active = 1U;
        printf("OK M%u MOVE %ld %lu\r\n", (unsigned)(index + 1U),
               (long)arg, (unsigned long)hz);
        return;
    }

    if (strcmp(argv[1], "GOTO") == 0)
    {
        int32_t target;

        if (argc < 3U)
        {
            printf("ERR ARG\r\n");
            return;
        }
        target = Remote_ToLong(argv[2], 0);
        if ((target < axis_min[index]) || (target > axis_max[index]))
        {
            printf("ERR LIMIT\r\n");
            return;
        }
        hz = Remote_ToHz((argc > 3U) ? argv[3] : NULL, 800U);
        arg = target - motors[index].position;
        (void)Motor_StartMove(index, arg, hz);
        remote_active = 1U;
        printf("OK M%u GOTO %ld\r\n", (unsigned)(index + 1U), (long)target);
        return;
    }

    if (strcmp(argv[1], "LIMIT") == 0)
    {
        if (argc < 4U)
        {
            printf("ERR ARG\r\n");
            return;
        }
        axis_min[index] = Remote_ToLong(argv[2], INT32_MIN);
        axis_max[index] = Remote_ToLong(argv[3], INT32_MAX);
        printf("OK M%u LIMIT %ld %ld\r\n", (unsigned)(index + 1U),
               (long)axis_min[index], (long)axis_max[index]);
        return;
    }

    printf("ERR UNKNOWN\r\n");
}

static void Remote_Service(void)
{
    uint8_t *data;
    uint16_t size;
    char *line;
    char *newline;

    if (u1_get_rx_flag() == 0U)
    {
        return;
    }

    data = u1_get_rx_data();
    size = u1_get_rx_size();
    if (size > (U1_BUFFER_MAX_SIZE - 1U))
    {
        size = U1_BUFFER_MAX_SIZE - 1U;
    }
    data[size] = 0;

    /* One frame may carry more than one line; run each of them. */
    line = (char *)data;
    while (*line != '\0')
    {
        newline = strchr(line, '\n');
        if (newline != NULL)
        {
            *newline = '\0';
        }
        Remote_Execute(line);
        if (newline == NULL)
        {
            break;
        }
        line = newline + 1;
    }
    u1_clean_rx_data();
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4   (42 MHz, TIM6 clock = 84 MHz)
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  *
  *         HSE is a 25 MHz crystal on the Fire F407ZG board, so PLL_M = 25 gives
  *         a 1 MHz VCO input: 25 MHz / 25 * 336 / 2 = 168 MHz.
  *         Keep HSE_VALUE in stm32f4xx_hal_conf.h at 25000000 as well.
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    while(1) {};
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    while(1) {};
  }

  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
  if (HAL_GetREVID() == 0x1001)
  {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }
}


/**
  * @brief  模拟不精确延时函数
  * @param  空指令数
  */
void Delay(uint32_t nCount)	
{
	for(uint32_t i = nCount; i != 0; i--)
    {
        __ASM("NOP"); //需要空指令占位，否则在高优化等级失效
    }
}


/* 勾选Use MicroLIB情况下，重定向c库函数printf到串口DEBUG_USART，重定向后可使用printf函数*/
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&Uart1Handle, (uint8_t *)&ch, 1, 1000);		
	return (ch);
}
