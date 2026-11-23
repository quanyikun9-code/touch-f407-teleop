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
#include "second_instrument.h"

/* M3 moves 65 calibrated 45-degree increments; clamp and open alternate. */
#define LCD_DIAGNOSTIC_MODE         0U
#define UI_STAGED_DIAGNOSTIC        0U
#define MOTOR_OUTPUT_ENABLED       1U
#define ISOLATOR_STATIC_TEST        0U
#define ACTIVE_MOTOR_COUNT          (SECOND_INSTRUMENT_ENABLED ? 6U : 3U)
#define TOUCH_REPORT_TIMEOUT_MS    300U

/* Serial remote layer (see the project guide, section 7). */
#define REMOTE_TIMEOUT_MS          1500U
#define REMOTE_MAX_STEP_HZ         10000U

#define MOTOR_COUNT                 6U
#define INSTRUMENT_COUNT            2U
#define BUTTON_COUNT                7U
#define UI_GLYPH_SIZE               24U
#define MOTOR_TIMER_TICK_HZ         20000U
/*
 * Screen buttons for M1 run at 4800 Hz (3x the original 1600 Hz): 4800/3200 =
 * 1.5 rev/s = 90 rpm. There is no acceleration ramp, so a loaded mechanism may
 * stall or lose steps at this speed - lower it here if it does. Note that this
 * value is also the default rate for a bare "M1 FWD" over serial; the bridge
 * always sends an explicit Hz, so its --max-hz1 is unaffected.
 */
#define MOTOR1_CONTINUOUS_SPEED_HZ  4800U

/*
 * Fixed return travel, in pulses. This is the distance measured on the machine
 * (limit_test: POS = 129776 when the photoelectric sensor triggered), i.e. the
 * gap between the initial position and the limit. 回位 always moves exactly
 * this many pulses backwards, no matter what the counter currently reads.
 */
#define MOTOR1_HOME_PULSES          129776L

/*
 * Speed of the on-screen "return to zero" move: 4800 Hz, about 27 s for the
 * 129776-pulse travel.
 */
#define MOTOR1_RETURN_SPEED_HZ      4800U

/*
 * Acceleration ramp, in Hz added per 20 kHz tick. 1 Hz/tick = 20000 Hz/s, so
 * 1600 Hz is reached in 80 ms and 4800 Hz in 240 ms. Without this the driver
 * jumps straight to the target rate and a loaded stepper stalls.
 */
#define MOTOR_RAMP_HZ_PER_TICK      1U
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

/*
 * Photoelectric sensor (beam / limit switch) on PC7.
 *
 * NPN open-collector output: the transistor pulls the line low while the beam
 * is blocked, so the input gets a pull-up and PHOTO_ACTIVE_LOW stays 1.
 * Sampling runs inside the 20 kHz TIM6 tick, decimated by PHOTO_SAMPLE_DIVIDER
 * to PHOTO_SAMPLE_HZ, so edges are still caught while the main loop spends
 * tens of milliseconds repainting the LCD.
 *
 * Electrical check before trusting the readings: with the sensor powered,
 * measure PC7 to GND in both beam states. It must never exceed 3.3 V (PC7 is
 * 5 V tolerant, not 24 V tolerant), and the sensor supply ground must be
 * common with the board GND. If the internal ~40k pull-up is too weak for the
 * cable, add an external 4.7k-10k pull-up from PC7 to 3.3 V.
 *
 * Note: on the F407 PC7 is also SDIO_D7. This project never enables SDIO, so
 * using it as a plain GPIO input is safe.
 */
#define PHOTO_GPIO_PORT             GPIOC
#define PHOTO_GPIO_PIN              GPIO_PIN_7
#define PHOTO_ACTIVE_LOW            1U
#define PHOTO_SAMPLE_DIVIDER        20U     /* 20 kHz tick / 20 = 1 kHz sampling */
#define PHOTO_SAMPLE_HZ             (MOTOR_TIMER_TICK_HZ / PHOTO_SAMPLE_DIVIDER)
#define PHOTO_SAMPLE_US             (1000000U / PHOTO_SAMPLE_HZ)
#define PHOTO_DEBOUNCE_SAMPLES      3U      /* accept a level after 3 stable samples */

/*
 * Alert while the beam is blocked: blink the red LED and/or sound the buzzer.
 * PHOTO_ALERT_MODE is a bit mask - 1 = red LED, 2 = buzzer, 3 = both, 0 = off.
 * The blink is driven from the same 1 kHz sample tick, so a slow LCD repaint
 * cannot make it stutter, and it costs a couple of GPIO writes per half period
 * (nothing at all while the beam is free).
 *
 * While alerting, the normal green LED is switched off so the blink reads as
 * red/off instead of red/green. A diagnostic build (LCD_DIAGNOSTIC_MODE or
 * UI_STAGED_DIAGNOSTIC) sets its own LED colours and will be overridden here.
 */
#define PHOTO_ALERT_MODE            3U
#define PHOTO_ALERT_HZ              4U      /* blink / beep cycles per second */

#if (PHOTO_ALERT_MODE != 0U) && (PHOTO_ALERT_HZ == 0U)
#error "PHOTO_ALERT_HZ must be greater than 0"
#endif

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
    GPIO_TypeDef *step_port;
    GPIO_TypeDef *dir_port;
    uint16_t step_pin;
    uint16_t dir_pin;
    uint8_t dir_inverted;
    volatile uint32_t phase_accumulator;
    volatile uint32_t pulse_frequency_hz;
    volatile uint32_t pulses_remaining;
    volatile uint8_t running;
    volatile uint8_t step_high;
    volatile uint8_t direction;
    volatile int32_t position;      /* signed pulse count, + = MOTOR_DIR_CW command */
    volatile uint32_t speed_now_hz;  /* ramped rate; pulse_frequency_hz is the target */
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

typedef struct
{
    M3NextAction_t next_action;
    uint16_t held_clamp_ids;
    uint32_t clamp_count;
    uint32_t open_count;
    volatile uint8_t return_pending;
    uint8_t return_was_down;
    uint32_t return_edges;
    uint16_t last_colors[BUTTON_COUNT];
    uint8_t last_remote;
    uint32_t last_clamp_count;
    uint32_t last_open_count;
} Instrument_t;

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
static uint8_t Motor_StartFixedMove(uint8_t group, MotorDirection_t direction);
static void Motor_Service(void);
static void Remote_Service(void);
static void Remote_Execute(char *line);
static void UI_Draw(void);
static void UI_Refresh(void);
static void UI_ProcessTouch(void);
static uint8_t UI_PointInButton(uint16_t x, uint16_t y, const Button_t *button);
static void UI_DrawGlyph(uint16_t x, uint16_t y, uint8_t glyph_index, uint16_t background);
static void UI_DrawButton(const Button_t *button, uint16_t top, uint16_t color);
static uint8_t UI_ReturnEnabled(uint8_t group);
static uint32_t UI_ReturnEdges(uint8_t group);
static void Photo_GPIO_Config(void);
static uint8_t Photo_IsActive(uint8_t raw);
static void Photo_SampleTick(void);
static void Photo_Snapshot(uint8_t *level, uint8_t *raw, uint32_t *edges,
                           uint32_t *edge_ms, uint32_t *period_us);
static void Photo_AlertTick(void);
static void Photo_AlertSet(uint8_t on);
static void Photo_AlertRestore(void);

TIM_HandleTypeDef htim6;

static Motor_t motors[MOTOR_COUNT] =
{
    {MOTOR_GPIO_PORT, MOTOR_GPIO_PORT, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN, MOTOR1_DIR_INVERTED},
    {MOTOR_GPIO_PORT, MOTOR_GPIO_PORT, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN, MOTOR2_DIR_INVERTED},
    {MOTOR_GPIO_PORT, MOTOR_GPIO_PORT, MOTOR3_STEP_PIN, MOTOR3_DIR_PIN, MOTOR3_DIR_INVERTED},
    {MOTOR4_STEP_PORT, MOTOR4_DIR_PORT, MOTOR4_STEP_PIN, MOTOR4_DIR_PIN, MOTOR4_DIR_INVERTED},
    {MOTOR5_STEP_PORT, MOTOR5_DIR_PORT, MOTOR5_STEP_PIN, MOTOR5_DIR_PIN, MOTOR5_DIR_INVERTED},
    {MOTOR6_STEP_PORT, MOTOR6_DIR_PORT, MOTOR6_STEP_PIN, MOTOR6_DIR_PIN, MOTOR6_DIR_INVERTED}
};

/* Coordinates are relative to the top of each half of the LCD. */
static const Button_t buttons[BUTTON_COUNT] =
{
    {16U, 64U, 218U, 76U, {0U, 1U, 2U, 3U}, 4U},
    {246U, 64U, 218U, 76U, {0U, 1U, 4U, 5U}, 4U},
    {16U, 152U, 218U, 76U, {6U, 7U, 8U, 9U, 10U}, 5U},
    {246U, 152U, 218U, 76U, {11U, 7U, 8U, 9U, 10U}, 5U},
    {16U, 240U, 218U, 76U, {12U, 13U, 12U, 14U}, 4U},
    {246U, 240U, 218U, 76U, {12U, 13U, 15U, 16U}, 4U},
    {344U, 14U, 120U, 40U, {4U, 17U}, 2U}
};

static Instrument_t instruments[INSTRUMENT_COUNT];
static uint8_t motor_config_ready;
static volatile uint32_t ui_last_touch_report_ms;
static uint8_t touch_ready;

/* Remote (PC / master arm) state. remote_active is set by motion commands. */
static volatile uint8_t remote_active;
static volatile uint32_t remote_last_cmd_ms;
static volatile uint8_t dual_active[INSTRUMENT_COUNT];
static uint8_t dual_touch[INSTRUMENT_COUNT];
static volatile uint8_t dual_rearm[INSTRUMENT_COUNT];
static volatile uint32_t dual_last_cmd_ms;
#define DUAL_TIMEOUT_MS 250U
#define DUAL_LINEAR_HZ 1600
#define DUAL_ROTARY_HZ 400
static uint32_t ui_last_pos_ms;
/* Legacy individual-axis commands still address only M1-M3. */
static int32_t axis_min[3] = {INT32_MIN, INT32_MIN, INT32_MIN};
static int32_t axis_max[3] = {INT32_MAX, INT32_MAX, INT32_MAX};

/* Photoelectric sensor state; written from the TIM6 tick, read by SENSOR. */
static volatile uint8_t  photo_level;        /* debounced: 1 = beam blocked */
static volatile uint8_t  photo_raw;          /* last raw pin level, for diagnostics */
static volatile uint32_t photo_edges;        /* blocked edges since boot/reset */
static volatile uint32_t photo_edge_ms;      /* HAL_GetTick() at the last edge */
static volatile uint32_t photo_period_us;    /* interval from the previous edge */
static volatile uint32_t photo_sample_index; /* 1 kHz sample counter */
static volatile uint32_t photo_edge_sample;  /* sample index at the last edge */
static volatile uint8_t  photo_has_edge;
static uint16_t photo_divider;
static uint8_t  photo_candidate;
static uint8_t  photo_same;
static volatile uint8_t photo_alert_on;
static uint8_t  photo_alert_started;
static uint16_t photo_alert_div;
static uint8_t photo4_enabled;
static uint8_t photo4_level;
static uint8_t photo4_candidate;
static uint8_t photo4_same;
static volatile uint32_t photo4_edges;

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

    /* 光电传感器（PC7）。采样在 TIM6 中断里做，这里只配引脚。 */
    BEEP_GPIO_Config();     /* PG7，遮挡提示用 */
    Photo_GPIO_Config();

    printf("欢迎使用野火开发板 \r\n");
    printf("Dual instrument touch control\r\n");
        
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
        LED_RGBOFF;
        LED_GREEN;
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
            UI_Refresh();
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

        UI_Refresh();

        HAL_Delay(10);
    }
}

static uint8_t GPIO_EnableClock(GPIO_TypeDef *port)
{
    if      (port == GPIOA) { __HAL_RCC_GPIOA_CLK_ENABLE(); }
    else if (port == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
    else if (port == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); }
    else if (port == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); }
    else if (port == GPIOE) { __HAL_RCC_GPIOE_CLK_ENABLE(); }
    else if (port == GPIOF) { __HAL_RCC_GPIOF_CLK_ENABLE(); }
    else if (port == GPIOG) { __HAL_RCC_GPIOG_CLK_ENABLE(); }
    else if (port == GPIOH) { __HAL_RCC_GPIOH_CLK_ENABLE(); }
    else if (port == GPIOI) { __HAL_RCC_GPIOI_CLK_ENABLE(); }
    else { return 0U; }
    return 1U;
}

static uint8_t GPIO_IsSinglePin(uint16_t pin)
{
    return (pin != 0U) && ((pin & (pin - 1U)) == 0U);
}

static uint8_t GPIO_SamePin(GPIO_TypeDef *a_port, uint16_t a_pin,
                            GPIO_TypeDef *b_port, uint16_t b_pin)
{
    return a_port == b_port && a_pin == b_pin;
}

static uint8_t Motor_PinValid(GPIO_TypeDef *port, uint16_t pin)
{
    return GPIO_IsSinglePin(pin) &&
           !GPIO_SamePin(port, pin, PHOTO_GPIO_PORT, PHOTO_GPIO_PIN) &&
           !(SECOND_INSTRUMENT_ENABLED && GPIO_SamePin(port, pin, MOTOR4_PHOTO_PORT, MOTOR4_PHOTO_PIN)) &&
           GPIO_EnableClock(port);
}

static void Motor_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < ACTIVE_MOTOR_COUNT; ++i)
    {
        const Motor_t *motor = &motors[i];
        if (!Motor_PinValid(motor->step_port, motor->step_pin) ||
            !Motor_PinValid(motor->dir_port, motor->dir_pin) ||
            GPIO_SamePin(motor->step_port, motor->step_pin, motor->dir_port, motor->dir_pin))
        {
            printf("ERR M%u PIN CONFIG\r\n", (unsigned)(i + 1U));
            return;
        }
        for (j = 0U; j < i; ++j)
        {
            const Motor_t *other = &motors[j];
            if (GPIO_SamePin(motor->step_port, motor->step_pin, other->step_port, other->step_pin) ||
                GPIO_SamePin(motor->step_port, motor->step_pin, other->dir_port, other->dir_pin) ||
                GPIO_SamePin(motor->dir_port, motor->dir_pin, other->step_port, other->step_pin) ||
                GPIO_SamePin(motor->dir_port, motor->dir_pin, other->dir_port, other->dir_pin))
            {
                printf("ERR M%u PIN CONFLICT\r\n", (unsigned)(i + 1U));
                return;
            }
        }
    }
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    for (i = 0U; i < ACTIVE_MOTOR_COUNT; ++i)
    {
        GPIO_InitStruct.Pin = motors[i].step_pin;
        HAL_GPIO_WritePin(motors[i].step_port, GPIO_InitStruct.Pin, GPIO_PIN_RESET);
        HAL_GPIO_Init(motors[i].step_port, &GPIO_InitStruct);
        GPIO_InitStruct.Pin = motors[i].dir_pin;
        HAL_GPIO_WritePin(motors[i].dir_port, GPIO_InitStruct.Pin, GPIO_PIN_RESET);
        HAL_GPIO_Init(motors[i].dir_port, &GPIO_InitStruct);
    }
    motor_config_ready = 1U;
}

static uint8_t Photo_IsActive(uint8_t raw)
{
#if (PHOTO_ACTIVE_LOW != 0U)
    return (raw == 0U) ? 1U : 0U;
#else
    return (raw != 0U) ? 1U : 0U;
#endif
}

static void Photo_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin = PHOTO_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
#if (PHOTO_ACTIVE_LOW != 0U)
    GPIO_InitStruct.Pull = GPIO_PULLUP;     /* NPN open collector needs a pull-up */
#else
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
#endif
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PHOTO_GPIO_PORT, &GPIO_InitStruct);

    photo_raw = (HAL_GPIO_ReadPin(PHOTO_GPIO_PORT, PHOTO_GPIO_PIN) == GPIO_PIN_SET) ? 1U : 0U;
    photo_level = Photo_IsActive(photo_raw);
    photo_candidate = photo_level;
    photo_same = 0U;
    photo_divider = 0U;
    photo_edges = 0U;
    photo_edge_ms = 0U;
    photo_period_us = 0U;
    photo_sample_index = 0U;
    photo_edge_sample = 0U;
    photo_has_edge = 0U;

    if (SECOND_INSTRUMENT_ENABLED && motor_config_ready && MOTOR4_RETURN_PULSES > 0L &&
        GPIO_IsSinglePin(MOTOR4_PHOTO_PIN) &&
        !(MOTOR4_PHOTO_PORT == PHOTO_GPIO_PORT && MOTOR4_PHOTO_PIN == PHOTO_GPIO_PIN) &&
        GPIO_EnableClock(MOTOR4_PHOTO_PORT))
    {
        GPIO_InitStruct.Pin = MOTOR4_PHOTO_PIN;
        GPIO_InitStruct.Pull = MOTOR4_PHOTO_ACTIVE_LOW ? GPIO_PULLUP : GPIO_PULLDOWN;
        HAL_GPIO_Init(MOTOR4_PHOTO_PORT, &GPIO_InitStruct);
        photo4_level = ((HAL_GPIO_ReadPin(MOTOR4_PHOTO_PORT, MOTOR4_PHOTO_PIN) == GPIO_PIN_RESET)
                        == (MOTOR4_PHOTO_ACTIVE_LOW != 0U));
        photo4_candidate = photo4_level;
        photo4_enabled = 1U;
    }
}

static void Photo4_Sample(void)
{
    uint8_t active;
    if (photo4_enabled == 0U)
        return;
    active = ((HAL_GPIO_ReadPin(MOTOR4_PHOTO_PORT, MOTOR4_PHOTO_PIN) == GPIO_PIN_RESET)
              == (MOTOR4_PHOTO_ACTIVE_LOW != 0U));
    if (active != photo4_candidate)
    {
        photo4_candidate = active;
        photo4_same = 1U;
    }
    else if (photo4_same < PHOTO_DEBOUNCE_SAMPLES)
    {
        ++photo4_same;
    }
    if (photo4_same >= PHOTO_DEBOUNCE_SAMPLES && active != photo4_level)
    {
        photo4_level = active;
        if (active != 0U)
            ++photo4_edges;
    }
}

/*
 * Runs from the 20 kHz TIM6 tick, so it does not care how long UI_Draw takes.
 * A level is committed only after PHOTO_DEBOUNCE_SAMPLES identical samples
 * (3 ms by default); EDGES counts the blocked transitions only.
 */
static void Photo_SampleTick(void)
{
    uint8_t raw;
    uint8_t active;

    if ((uint32_t)(photo_divider + 1U) < (uint32_t)PHOTO_SAMPLE_DIVIDER)
    {
        photo_divider++;
        return;
    }
    photo_divider = 0U;
    photo_sample_index++;
    Photo4_Sample();

    /* Uses the level committed on the previous sample: one 1 ms tick of lag. */
    Photo_AlertTick();

    raw = (HAL_GPIO_ReadPin(PHOTO_GPIO_PORT, PHOTO_GPIO_PIN) == GPIO_PIN_SET) ? 1U : 0U;
    photo_raw = raw;
    active = Photo_IsActive(raw);

    if (active != photo_candidate)
    {
        photo_candidate = active;
        photo_same = 1U;
        return;
    }
    if ((uint32_t)photo_same < (uint32_t)PHOTO_DEBOUNCE_SAMPLES)
    {
        photo_same++;
        return;
    }
    if (active == photo_level)
    {
        return;
    }

    photo_level = active;
    if (active != 0U)
    {
        photo_edges++;
        if (photo_has_edge != 0U)
        {
            photo_period_us = (uint32_t)(photo_sample_index - photo_edge_sample) * PHOTO_SAMPLE_US;
        }
        photo_edge_sample = photo_sample_index;
        photo_edge_ms = HAL_GetTick();
        photo_has_edge = 1U;
    }
}

/* Copy the live values with the tick masked, so no uint32 read can tear. */
static void Photo_Snapshot(uint8_t *level, uint8_t *raw, uint32_t *edges,
                           uint32_t *edge_ms, uint32_t *period_us)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    *level = photo_level;
    *raw = photo_raw;
    *edges = photo_edges;
    *edge_ms = photo_edge_ms;
    *period_us = photo_period_us;
    __set_PRIMASK(primask);
}

#if (PHOTO_ALERT_MODE != 0U)
static void Photo_AlertSet(uint8_t on)
{
#if (PHOTO_ALERT_MODE & 2U)
    HAL_GPIO_WritePin(BEEP_GPIO_PORT, BEEP_PIN,
                      (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
#if (PHOTO_ALERT_MODE & 1U)
    /* Red LED is active low; hide the normal green so the blink reads red/off. */
    HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN,
                      (on != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_SET);
#endif
}

static void Photo_AlertRestore(void)
{
#if (PHOTO_ALERT_MODE & 2U)
    HAL_GPIO_WritePin(BEEP_GPIO_PORT, BEEP_PIN, GPIO_PIN_RESET);
#endif
#if (PHOTO_ALERT_MODE & 1U)
    HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_RESET);   /* back to LED_GREEN */
#endif
}

/* Runs once per 1 ms sample: blocked -> toggle at PHOTO_ALERT_HZ. */
static void Photo_AlertTick(void)
{
    uint16_t half = (uint16_t)(PHOTO_SAMPLE_HZ / (2U * PHOTO_ALERT_HZ));

    if (half == 0U)
    {
        half = 1U;
    }

    if (photo_level == 0U)
    {
        if (photo_alert_started != 0U)
        {
            photo_alert_started = 0U;
            photo_alert_on = 0U;
            photo_alert_div = 0U;
            Photo_AlertRestore();
        }
        return;
    }

    if (photo_alert_started == 0U)
    {
        /* Start with the LED/buzzer on so the first blocked millisecond is visible. */
        photo_alert_started = 1U;
        photo_alert_on = 1U;
        photo_alert_div = 0U;
        Photo_AlertSet(1U);
        return;
    }

    if (++photo_alert_div < half)
    {
        return;
    }
    photo_alert_div = 0U;
    photo_alert_on ^= 1U;
    Photo_AlertSet(photo_alert_on);
}
#else
static void Photo_AlertTick(void)
{
}
#endif

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
    uint32_t hz = (motor_index % 3U == 0U) ? MOTOR1_CONTINUOUS_SPEED_HZ
                                      : MOTOR_CONTINUOUS_SPEED_HZ;
    (void)Motor_StartContinuousHz(motor_index, direction, hz);
}

/* Continuous run at an explicit step rate (remote FWD/REV). */
static uint8_t Motor_StartContinuousHz(uint8_t motor_index, MotorDirection_t direction,
                                       uint32_t hz)
{
    Motor_t *motor = &motors[motor_index];
    uint8_t dir_high = (direction == MOTOR_DIR_CW) ? 1U : 0U;

    if (motor_index >= ACTIVE_MOTOR_COUNT ||
        (MOTOR_OUTPUT_ENABLED && !ISOLATOR_STATIC_TEST && !motor_config_ready))
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

    if (MOTOR_OUTPUT_ENABLED && !ISOLATOR_STATIC_TEST)
    {
        dir_high ^= (motor->dir_inverted != 0U);
        HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin,
                          (dir_high != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    motor->direction = direction;
    motor->phase_accumulator = 0U;
    motor->speed_now_hz = 0U;
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

    if (motor_index >= ACTIVE_MOTOR_COUNT ||
        (MOTOR_OUTPUT_ENABLED && !ISOLATOR_STATIC_TEST && !motor_config_ready))
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
    magnitude = (pulses > 0) ? (uint32_t)pulses : (0U - (uint32_t)pulses);
    dir_high = (direction == MOTOR_DIR_CW) ? 1U : 0U;

    if (MOTOR_OUTPUT_ENABLED && !ISOLATOR_STATIC_TEST)
    {
        dir_high ^= (motor->dir_inverted != 0U);
        HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin,
                          (dir_high != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    motor->direction = direction;
    motor->phase_accumulator = 0U;
    motor->speed_now_hz = 0U;
    motor->pulses_remaining = magnitude;
    motor->pulse_frequency_hz = hz;
    motor->running = 1U;
    return 1U;
}

static void Motor_Stop(uint8_t motor_index)
{
    motors[motor_index].running = 0U;
}

static uint8_t Motor_StartFixedMove(uint8_t group, MotorDirection_t direction)
{
    uint8_t index = group * 3U + 2U;
    Motor_t *motor = &motors[index];
    Instrument_t *instrument = &instruments[group];
    int32_t pulses = (group == 0U) ? MOTOR3_TRAVEL_PULSES : MOTOR6_TRAVEL_PULSES;

    if ((motor->running != 0U) ||
        ((direction == MOTOR_DIR_CW) && (instrument->next_action == M3_ALLOW_OPEN)) ||
        ((direction == MOTOR_DIR_CCW) && (instrument->next_action == M3_ALLOW_CLAMP)) ||
        (pulses <= 0) ||
        !Motor_StartMove(index, (direction == MOTOR_DIR_CW) ? pulses : -pulses, MOTOR3_SPEED_HZ))
    {
        return 0U;
    }

    instrument->next_action = (direction == MOTOR_DIR_CW) ? M3_ALLOW_OPEN : M3_ALLOW_CLAMP;
    if (direction == MOTOR_DIR_CW)
        ++instrument->clamp_count;
    else
        ++instrument->open_count;
    return 1U;
}

static void Motor_Service(void)
{
    uint8_t motor_index;
    uint8_t group;
    uint32_t now = HAL_GetTick();

    for (group = 0U; group < INSTRUMENT_COUNT; ++group)
    {
        if (dual_active[group] && (uint32_t)(now - dual_last_cmd_ms) >= DUAL_TIMEOUT_MS)
        {
            Motor_Stop(group * 3U);
            Motor_Stop(group * 3U + 1U);
            dual_rearm[group] = 1U;
            dual_active[group] = 0U;
        }
    }

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
    if ((uint32_t)(HAL_GetTick() - ui_last_touch_report_ms) > TOUCH_REPORT_TIMEOUT_MS)
    {
        for (group = 0U; group < INSTRUMENT_COUNT; ++group)
        {
            if (dual_active[group] || (group == 0U && remote_active))
                continue;
            if (instruments[group].return_pending == 0U)
                Motor_Stop(group * 3U);
            Motor_Stop(group * 3U + 1U);
        }
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
                HAL_GPIO_WritePin(motor->step_port, motor->step_pin, GPIO_PIN_RESET);
            }
            motor->step_high = 0U;
        }

        if (motor->running == 0U)
        {
            continue;
        }

        /* Ramp towards the commanded rate instead of jumping straight to it. */
        if (motor->speed_now_hz < motor->pulse_frequency_hz)
        {
            motor->speed_now_hz += MOTOR_RAMP_HZ_PER_TICK;
            if (motor->speed_now_hz > motor->pulse_frequency_hz)
            {
                motor->speed_now_hz = motor->pulse_frequency_hz;
            }
        }
        else if (motor->speed_now_hz > motor->pulse_frequency_hz)
        {
            motor->speed_now_hz -= MOTOR_RAMP_HZ_PER_TICK;
            if (motor->speed_now_hz < motor->pulse_frequency_hz)
            {
                motor->speed_now_hz = motor->pulse_frequency_hz;
            }
        }

        motor->phase_accumulator += motor->speed_now_hz;
        if (motor->phase_accumulator >= MOTOR_TIMER_TICK_HZ)
        {
            motor->phase_accumulator -= MOTOR_TIMER_TICK_HZ;
            if (MOTOR_OUTPUT_ENABLED != 0U)
            {
                HAL_GPIO_WritePin(motor->step_port, motor->step_pin, GPIO_PIN_SET);
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
        Photo_SampleTick();
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

    lcd_open_window(x, y, UI_GLYPH_SIZE, UI_GLYPH_SIZE);
    for (row = 0U; row < UI_GLYPH_SIZE; row++)
    {
        for (col = 0U; col < UI_GLYPH_SIZE; col++)
        {
            uint8_t src_row = row * 32U / UI_GLYPH_SIZE;
            uint8_t src_col = col * 32U / UI_GLYPH_SIZE;
            uint8_t pixel = glyph[(uint16_t)src_row * 4U + src_col / 8U] &
                            (uint8_t)(0x80U >> (src_col % 8U));
            LCD_WRITE_DATA(pixel ? (background == YELLOW || background == GREEN ? BLACK : WHITE) : background);
        }
    }
}

static void UI_DrawButton(const Button_t *button, uint16_t top, uint16_t color)
{
    uint16_t y = top + button->y;
    uint16_t text_x = button->x + (button->width - (uint16_t)button->label_length * UI_GLYPH_SIZE) / 2U;
    uint16_t text_y = y + (button->height - UI_GLYPH_SIZE) / 2U;
    uint8_t index;

    lcd_fill_color(button->x, y, button->width, button->height, color);
    /*
     * Avoid LCD_Draw_Line here: the example's implementation mishandles
     * decreasing coordinates. Four filled 1-pixel strips are safe borders.
     */
    lcd_fill_color(button->x, y, button->width, 1U, WHITE);
    lcd_fill_color(button->x, y + button->height - 1U, button->width, 1U, WHITE);
    lcd_fill_color(button->x, y, 1U, button->height, WHITE);
    lcd_fill_color(button->x + button->width - 1U, y, 1U, button->height, WHITE);
    for (index = 0U; index < button->label_length; index++)
    {
        UI_DrawGlyph((uint16_t)(text_x + (uint16_t)index * UI_GLYPH_SIZE), text_y,
                     button->label_glyphs[index], color);
    }
}

static uint8_t UI_GroupEnabled(uint8_t group)
{
    return (group * 3U < ACTIVE_MOTOR_COUNT) &&
           (!MOTOR_OUTPUT_ENABLED || ISOLATOR_STATIC_TEST || motor_config_ready);
}

static uint32_t UI_ReturnEdges(uint8_t group)
{
    return (group == 0U) ? photo_edges : photo4_edges;
}

static uint8_t UI_ReturnEnabled(uint8_t group)
{
    Instrument_t *instrument = &instruments[group];
    uint32_t edges = UI_ReturnEdges(group);
    /* The legacy SENSOR RESET command can lower the M1 counter. */
    if (edges < instrument->return_edges)
        instrument->return_edges = edges;
    return (group == 0U || photo4_enabled) && edges > instrument->return_edges;
}

static uint16_t UI_ButtonColor(uint8_t group, uint8_t index)
{
    const Instrument_t *instrument = &instruments[group];
    uint8_t forward = (index & 1U) == 0U;
    const Motor_t *motor;

    if (!UI_GroupEnabled(group))
        return GRAY;
    if (index == 6U)
    {
        if (instrument->return_pending)
            return GREEN;
        return (motors[group * 3U].running == 0U && UI_ReturnEnabled(group)) ? YELLOW : GRAY;
    }
    motor = &motors[group * 3U + index / 2U];
    if (motor->running && motor->direction == (forward ? MOTOR_DIR_CW : MOTOR_DIR_CCW))
        return GREEN;
    if (index >= 4U && (motor->running ||
        instrument->next_action == (forward ? M3_ALLOW_OPEN : M3_ALLOW_CLAMP)))
        return GRAY;
    return BLUE;
}

static void UI_Refresh(void)
{
    uint8_t group;
    uint32_t now = HAL_GetTick();
    uint8_t refresh_position = (uint32_t)(now - ui_last_pos_ms) >= 500U;

    for (group = 0U; group < INSTRUMENT_COUNT; ++group)
    {
        uint16_t top = group * (LCD_Y_LENGTH / 2U);
        Instrument_t *instrument = &instruments[group];
        uint8_t base = group * 3U;
        uint8_t remote = dual_active[group] || (group == 0U && remote_active);
        uint8_t index;
        char text[48];

        if (instrument->return_pending && !motors[base].running)
            instrument->return_pending = 0U;
        for (index = 0U; index < BUTTON_COUNT; ++index)
        {
            uint16_t color = UI_ButtonColor(group, index);
            if (color != instrument->last_colors[index])
            {
                UI_DrawButton(&buttons[index], top, color);
                instrument->last_colors[index] = color;
            }
        }
        if (refresh_position)
        {
            lcd_fill_color(16U, top + 332U, 432U, 16U, BLACK);
            sprintf(text, "M%u POS:%ld pulses", (unsigned)(base + 1U), (long)motors[base].position);
            LCD_DispString_EN(16U, top + 332U, text, WHITE, BLACK, &Font8x16);
        }
        if (instrument->last_clamp_count != instrument->clamp_count ||
            instrument->last_open_count != instrument->open_count || instrument->last_remote != remote)
        {
            lcd_fill_color(16U, top + 356U, 432U, 16U, BLACK);
            if (UI_GroupEnabled(group))
                sprintf(text, "CLAMP:%lu  OPEN:%lu  %s", (unsigned long)instrument->clamp_count,
                        (unsigned long)instrument->open_count, remote ? "REMOTE" : "LOCAL");
            else
                strcpy(text, "DISABLED: CHECK PIN CONFIG");
            LCD_DispString_EN(16U, top + 356U, text, CYAN, BLACK, &Font8x16);
            instrument->last_clamp_count = instrument->clamp_count;
            instrument->last_open_count = instrument->open_count;
            instrument->last_remote = remote;
        }
    }
    if (refresh_position)
        ui_last_pos_ms = now;
}

static void UI_Draw(void)
{
    uint8_t group;
    lcd_fill_color(0U, 0U, LCD_X_LENGTH, LCD_Y_LENGTH, BLACK);
    for (group = 0U; group < INSTRUMENT_COUNT; ++group)
    {
        uint16_t top = group * (LCD_Y_LENGTH / 2U);
        char title[24];
        sprintf(title, "SET %u / M%u-M%u", (unsigned)(group + 1U),
                (unsigned)(group * 3U + 1U), (unsigned)(group * 3U + 3U));
        lcd_fill_color(0U, top, LCD_X_LENGTH, 2U, group == 0U ? CYAN : YELLOW);
        LCD_DispString_EN(16U, top + 20U, title, group == 0U ? CYAN : YELLOW, BLACK, &Font16x24);
        instruments[group].last_remote = 0xFFU;
        memset(instruments[group].last_colors, 0xFF, sizeof(instruments[group].last_colors));
    }
    ui_last_pos_ms = HAL_GetTick() - 500U;
    UI_Refresh();
}

static void UI_ProcessTouch(void)
{
    uint8_t commands[INSTRUMENT_COUNT][4] = {{0U}};
    uint16_t clamp_ids[INSTRUMENT_COUNT] = {0U};
    uint16_t live_ids = 0U;
    uint16_t half = LCD_Y_LENGTH / 2U;
    uint8_t result = 3U;
    int8_t point;
    uint8_t group;

    if (touch_ready != 0U)
    {
        result = LCD_Touch_Get_Points_Data(touch_points, &touch_points_num);
        if (result == 0U)
            ui_last_touch_report_ms = HAL_GetTick();
    }
    if (result != 0U && (result != 2U ||
        (uint32_t)(HAL_GetTick() - ui_last_touch_report_ms) > TOUCH_REPORT_TIMEOUT_MS))
        touch_points_num = 0;

    for (point = 0; point < touch_points_num; ++point)
    {
        const Touch_Point_t *touch = &touch_points[point];
        uint16_t id = (uint16_t)(1U << touch->track_id);
        uint16_t y;
        uint8_t button;
        live_ids |= id;
        /* Keep the existing GT911 to LCD scan-mode-6 transform. */
        if (touch->x >= LCD_Y_LENGTH || touch->y >= LCD_X_LENGTH)
            continue;
        y = LCD_Y_LENGTH - 1U - touch->x;
        group = (y >= half) ? 1U : 0U;
        y -= group * half;
        for (button = 0U; button < BUTTON_COUNT; ++button)
        {
            if (UI_PointInButton(touch->y, y, &buttons[button]))
            {
                commands[group][button / 2U] |= (uint8_t)(1U << (button & 1U));
                if (button == 4U || button == 5U)
                    clamp_ids[group] |= id;
                break;
            }
        }
    }

    for (group = 0U; group < INSTRUMENT_COUNT; ++group)
    {
        Instrument_t *instrument = &instruments[group];
        const uint8_t *command = commands[group];
        uint8_t base = group * 3U;
        uint8_t axis;

        if (!UI_GroupEnabled(group))
            continue;
        dual_touch[group] = (command[0] | command[1] | command[3]) != 0U;
        if (dual_touch[group] && dual_active[group])
        {
            Motor_Stop(base);
            Motor_Stop(base + 1U);
            dual_active[group] = 0U;
            dual_rearm[group] = 1U;
        }
        /* Keep the legacy single-hand arbitration unchanged. */
        if (group == 0U && remote_active)
        {
            if (!(command[0] | command[1] | command[2] | command[3]))
            {
                if (result == 0U)
                {
                    instrument->held_clamp_ids &= live_ids;
                    instrument->return_was_down = 0U;
                }
                continue;
            }
            remote_active = 0U;
        }

        if (result == 0U)
        {
            if (command[3] && !instrument->return_was_down && !command[0] &&
                !motors[base].running && UI_ReturnEnabled(group))
            {
                int32_t pulses = group == 0U ? MOTOR1_HOME_PULSES : MOTOR4_RETURN_PULSES;
                uint32_t edges = UI_ReturnEdges(group);
                if (Motor_StartMove(base, -pulses, MOTOR1_RETURN_SPEED_HZ))
                {
                    instrument->return_pending = 1U;
                    instrument->return_edges = edges;
                }
            }
            instrument->return_was_down = command[3] != 0U;

            /* Rearm when this finger lifts, even if the other panel is held. */
            instrument->held_clamp_ids &= live_ids;
            if ((clamp_ids[group] & ~instrument->held_clamp_ids) &&
                (command[2] == 1U || command[2] == 2U))
                (void)Motor_StartFixedMove(group, command[2] == 1U ? MOTOR_DIR_CW : MOTOR_DIR_CCW);
            instrument->held_clamp_ids |= clamp_ids[group];
        }

        if (dual_active[group])
            continue; /* Clamp events above remain local even during remote jog. */

        if (command[0] != 0U && instrument->return_pending)
        {
            Motor_Stop(base);
            instrument->return_pending = 0U;
        }
        for (axis = 0U; axis < 2U; ++axis)
        {
            if (command[axis] == 1U || command[axis] == 2U)
                Motor_StartContinuous(base + axis, command[axis] == 1U ? MOTOR_DIR_CW : MOTOR_DIR_CCW);
            else if (axis != 0U || !instrument->return_pending)
                Motor_Stop(base + axis); /* Includes conflicting direction touches. */
        }
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

static void Remote_DualRelease(uint8_t group)
{
    if (dual_active[group])
    {
        Motor_Stop(group * 3U);
        Motor_Stop(group * 3U + 1U);
        dual_active[group] = 0U;
    }
}

/* A = armed with zero rate, so returning to the deadband is not a release. */
static void Remote_DualVelocity(char **argv, uint8_t argc)
{
    long rate[4];
    uint8_t armed[2];
    uint8_t i;
    uint8_t group;
    uint32_t primask;

    if (argc != 5U || !UI_GroupEnabled(1U) || remote_active ||
        ISOLATOR_STATIC_TEST || !MOTOR_OUTPUT_ENABLED || !motor_config_ready)
    {
        printf("ERR DUAL UNAVAILABLE\r\n");
        return;
    }
    for (i = 0U; i < 4U; ++i)
    {
        char *end;
        long cap = (i % 2U == 0U) ? DUAL_LINEAR_HZ : DUAL_ROTARY_HZ;
        const char *text = argv[i + 1U];
        rate[i] = strtol(text, &end, 10);
        if (end == text || *end != '\0' || rate[i] < -cap || rate[i] > cap)
        {
            if (strcmp(text, "A") != 0 || i % 2U != 0U)
            {
                printf("ERR DUAL RATE\r\n");
                return;
            }
            rate[i] = 0;
        }
        if (i % 2U != 0U)
        {
            armed[i / 2U] = rate[i] != 0 || rate[i - 1U] != 0 ||
                strcmp(argv[i], "A") == 0;
            if (rate[i] != 0 && rate[i - 1U] != 0)
            {
                printf("ERR DUAL MODE\r\n");
                return;
            }
        }
    }
    primask = __get_PRIMASK();
    __disable_irq();
    dual_last_cmd_ms = HAL_GetTick();
    for (group = 0U; group < INSTRUMENT_COUNT; ++group)
    {
        uint8_t axis;
        if (!armed[group])
        {
            Remote_DualRelease(group);
            dual_rearm[group] = 0U;
            continue;
        }
        if (dual_touch[group] || instruments[group].return_pending || dual_rearm[group])
        {
            Remote_DualRelease(group);
            dual_rearm[group] = 1U;
            continue;
        }
        dual_active[group] = 1U;
        for (axis = 0U; axis < 2U; ++axis)
        {
            uint8_t index = group * 3U + axis;
            long hz = rate[group * 2U + axis];
            if (hz == 0)
                Motor_Stop(index);
            else
                (void)Motor_StartContinuousHz(index, hz > 0 ? MOTOR_DIR_CW : MOTOR_DIR_CCW,
                                              (uint32_t)(hz > 0 ? hz : -hz));
        }
    }
    __set_PRIMASK(primask);
    printf("OK DUAL %u%u\r\n", (unsigned)dual_active[0], (unsigned)dual_active[1]);
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

    /*
     * SENSOR is a pure read. Handle it before the watchdog refresh: a sensor
     * logger left polling after the bridge dies must not keep M1/M2 running.
     */
    if (strcmp(argv[0], "SENSOR") == 0)
    {
        uint8_t level;
        uint8_t raw;
        uint32_t edges;
        uint32_t edge_ms;
        uint32_t period_us;

        if ((argc > 1U) && (strcmp(argv[1], "RESET") == 0))
        {
            uint32_t primask = __get_PRIMASK();

            __disable_irq();
            photo_edges = 0U;
            photo_has_edge = 0U;
            photo_period_us = 0U;
            __set_PRIMASK(primask);
            printf("OK SENSOR RESET\r\n");
            return;
        }

        Photo_Snapshot(&level, &raw, &edges, &edge_ms, &period_us);
        printf("OK SENSOR=%u RAW=%u EDGES=%lu MS=%lu PERIOD=%lu\r\n",
               (unsigned)level, (unsigned)raw, (unsigned long)edges,
               (unsigned long)edge_ms, (unsigned long)period_us);
        return;
    }

    if (strcmp(argv[0], "DUAL?") == 0)
    {
        if (UI_GroupEnabled(1U) && !ISOLATOR_STATIC_TEST &&
            MOTOR_OUTPUT_ENABLED && motor_config_ready)
            printf("OK DUAL V1\r\n");
        else
            printf("ERR DUAL UNAVAILABLE\r\n");
        return;
    }

    if (strcmp(argv[0], "DUAL") == 0)
    {
        Remote_DualVelocity(argv, argc);
        return;
    }

    remote_last_cmd_ms = HAL_GetTick();

    if (strcmp(argv[0], "PING") == 0)
    {
        printf("OK PONG\r\n");
        return;
    }

    if (strcmp(argv[0], "BEEP") == 0)
    {
        int32_t ms = Remote_ToLong((argc > 1U) ? argv[1] : NULL, 200);

        if (ms < 1)
        {
            ms = 1;
        }
        if (ms > 5000)
        {
            ms = 5000;
        }
        HAL_GPIO_WritePin(BEEP_GPIO_PORT, BEEP_PIN, GPIO_PIN_SET);
        HAL_Delay((uint32_t)ms);
        HAL_GPIO_WritePin(BEEP_GPIO_PORT, BEEP_PIN, GPIO_PIN_RESET);
        printf("OK BEEP %ld\r\n", (long)ms);
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
        Remote_DualRelease(0U);
        Remote_DualRelease(1U);
        dual_rearm[0] = dual_rearm[1] = 1U;
        remote_active = 0U;
        Motor_Stop(0U);
        Motor_Stop(1U);
        printf("OK LOCAL\r\n");
        return;
    }

    if (strcmp(argv[0], "STOP") == 0)
    {
        uint8_t motor_index;
        for (motor_index = 0U; motor_index < MOTOR_COUNT; ++motor_index)
            Motor_Stop(motor_index);
        remote_active = 0U;
        dual_active[0] = dual_active[1] = 0U;
        dual_rearm[0] = dual_rearm[1] = 1U;
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

    if (dual_active[0] || dual_active[1])
    {
        printf("ERR DUAL ACTIVE\r\n");
        return;
    }

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

            if (Motor_StartFixedMove(0U, dir) == 0U)
            {
                printf("ERR M3 BUSY\r\n");
                return;
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
    char frame[U1_BUFFER_MAX_SIZE];
    uint16_t size;
    char *line;
    char *newline;
    uint32_t primask;

    if (u1_get_rx_flag() == 0U)
    {
        return;
    }

    /*
     * Copy the frame out and release the RX buffer *before* printing the reply.
     * The USART ISR drops bytes while a frame is still pending, so holding the
     * buffer through a blocking printf() lost every command that arrived during
     * the ~4 ms reply. With a 10 ms main loop that hit roughly every other poll,
     * and since a lost command does not refresh the watchdog, a moving axis could
     * be stopped by the 1.5 s remote timeout in the middle of a move.
     */
    primask = __get_PRIMASK();
    __disable_irq();
    size = u1_get_rx_size();
    if (size > (U1_BUFFER_MAX_SIZE - 1U))
    {
        size = U1_BUFFER_MAX_SIZE - 1U;
    }
    memcpy(frame, u1_get_rx_data(), size);
    frame[size] = '\0';
    u1_clean_rx_data();
    __set_PRIMASK(primask);

    /* One frame may carry more than one line; run each of them. */
    line = frame;
    while (*line != '\0')
    {
        newline = strchr(line, '\n');
        if (newline != NULL)
        {
            *newline = '\0';
        }
        /* Never apply a partial DUAL rate frame (e.g. after cable loss). */
        if (newline != NULL || (line[0] != 'D' && line[0] != 'd'))
            Remote_Execute(line);
        if (newline == NULL)
        {
            break;
        }
        line = newline + 1;
    }
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
