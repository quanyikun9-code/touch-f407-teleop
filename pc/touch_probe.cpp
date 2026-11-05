// touch_probe.cpp — Geomagic Touch 只读采集：位姿/速度/万向角/按键/墨水井 → CSV
//
// 特点：
//   * 不调用 hdEnable(HD_FORCE_OUTPUT)，电机不出力，手臂完全自由拖动
//   * 固定节拍采样（默认 200 Hz），每行一个样本，可直接 pandas / Excel 打开
//   * 启动时检查标定状态；标定不是 OK 就停下并告诉你怎么办（--update-calib 可自动更新）
//
// 编译（VS2022，x64 Native Tools Command Prompt）：
//   set OH_SDK_BASE=C:\OpenHaptics\Developer\3.4.0
//   cl /nologo /EHsc /MT /O2 /std:c++17 touch_probe.cpp ^
//      /I"%OH_SDK_BASE%\include" /I"%OH_SDK_BASE%\utilities\include" ^
//      /link /LIBPATH:"%OH_SDK_BASE%\lib" hd.lib hdu.lib
//   （若 SDK 把库放在 lib\x64，就把 /LIBPATH 换成 lib\x64）
//
// 运行（先把 hd.dll 所在目录加进 PATH）：
//   set PATH=%OH_SDK_BASE%\bin;%PATH%
//   touch_probe.exe --out touch_30s.csv --seconds 30 --hz 200
//
// 采集完用 analyze_master.py 看噪声/量程/跳变：
//   python analyze_master.py touch_30s.csv --plot touch_30s.png
//
// 注意：本文件没有在本机编译过（这台机器没有 MSVC / OpenHaptics SDK）。
//       第一次编译若报符号不存在，先核对 hd.h 里对应常量名，别改采样逻辑。

#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>

#include <windows.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>

namespace {

HHD g_hd = 0;
using Clock = std::chrono::steady_clock;

struct Sample {
    hduVector3Dd pos;
    hduVector3Dd vel;
    hduVector3Dd gimbal;
    HDint        buttons = 0;
    HDboolean    inkwell = 0;
};

// 在伺服回调里取一帧一致快照。只读，不设力。
HDCallbackCode HDCALLBACK cbSample(void* userData) {
    Sample* s = static_cast<Sample*>(userData);
    hdBeginFrame(g_hd);
    hdGetDoublev(HD_CURRENT_POSITION, s->pos);
    hdGetDoublev(HD_CURRENT_VELOCITY, s->vel);
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, s->gimbal);
    hdGetIntegerv(HD_CURRENT_BUTTONS, &s->buttons);
    hdGetBooleanv(HD_CURRENT_INKWELL_SWITCH, &s->inkwell);
    hdEndFrame(g_hd);
    return HD_CALLBACK_DONE;
}

HDCallbackCode HDCALLBACK cbCheckCalib(void* userData) {
    HDenum* state = static_cast<HDenum*>(userData);
    hdBeginFrame(g_hd);
    *state = hdCheckCalibration();
    hdEndFrame(g_hd);
    return HD_CALLBACK_DONE;
}

// 官方要求：hdUpdateCalibration 只能在回调里调，且不要再套 begin/endFrame。
HDCallbackCode HDCALLBACK cbUpdateCalib(void* userData) {
    HDenum style = *static_cast<HDenum*>(userData);
    hdUpdateCalibration(style);
    return HD_CALLBACK_DONE;
}

void printHdError(const char* where) {
    HDErrorInfo err = hdGetError();
    std::fprintf(stderr, "[HD ERROR] %s: code=0x%x internal=0x%x\n", where,
                 static_cast<unsigned>(err.errorCode),
                 static_cast<unsigned>(err.internalErrorCode));
    if (err.description) std::fprintf(stderr, "           %s\n", err.description);
}

void usage() {
    std::fprintf(stderr,
        "用法: touch_probe.exe [--out FILE] [--seconds N] [--hz HZ] [--update-calib]\n"
        "  --out FILE       输出 CSV，默认 touch_capture.csv\n"
        "  --seconds N      采集时长秒数，默认 30\n"
        "  --hz HZ          采样率（1~1000），默认 200\n"
        "  --update-calib   标定状态不是 OK 时，自动执行一次 hdUpdateCalibration\n");
}

}  // namespace

int main(int argc, char** argv) {
    std::string outPath = "touch_capture.csv";
    double seconds = 30.0;
    double hz = 200.0;
    bool updateCalib = false;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--out") == 0 && i + 1 < argc) {
            outPath = argv[++i];
        } else if (std::strcmp(argv[i], "--seconds") == 0 && i + 1 < argc) {
            seconds = std::atof(argv[++i]);
        } else if (std::strcmp(argv[i], "--hz") == 0 && i + 1 < argc) {
            hz = std::atof(argv[++i]);
        } else if (std::strcmp(argv[i], "--update-calib") == 0) {
            updateCalib = true;
        } else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            usage();
            return 0;
        } else {
            std::fprintf(stderr, "未知参数: %s\n", argv[i]);
            usage();
            return 1;
        }
    }
    if (seconds <= 0 || seconds > 3600) seconds = 30.0;
    if (hz < 1 || hz > 1000) hz = 200.0;

    g_hd = hdInitDevice(HD_DEFAULT_DEVICE);
    HDErrorInfo err = hdGetError();
    if (err.errorCode != HD_SUCCESS) {
        std::fprintf(stderr,
            "[HD ERROR] hdInitDevice 失败 (0x%x)\n"
            "  逐项查：\n"
            "   1) 设备管理器 -> 端口 里有没有 '3D Systems Touch Haptic Device (COMx)'\n"
            "   2) 那个 COM 口是不是被串口助手/Python 占着（同一时刻只能一个程序用）\n"
            "   3) 设备电源灯亮不亮（墨水井蓝灯上电快闪两下）\n"
            "   4) 驱动是否装错型号（Touch 用 PID_0302 / gtouchusb.inf）\n",
            static_cast<unsigned>(err.errorCode));
        return 2;
    }

    // 身份信息必须在调度器启动前查（线程安全要求）
    HDint inDof = 0, outDof = 0, calibStyle = 0;
    hdGetIntegerv(HD_INPUT_DOF, &inDof);
    hdGetIntegerv(HD_OUTPUT_DOF, &outDof);
    hdGetIntegerv(HD_CALIBRATION_STYLE, &calibStyle);

    HDdouble ws[6] = {0};
    hdGetDoublev(HD_USABLE_WORKSPACE_DIMENSIONS, ws);

    std::fprintf(stderr, "model      : %s\n", hdGetString(HD_DEVICE_MODEL_TYPE));
    std::fprintf(stderr, "serial     : %s\n", hdGetString(HD_DEVICE_SERIAL_NUMBER));
    std::fprintf(stderr, "driver     : %s\n", hdGetString(HD_DEVICE_DRIVER_VERSION));
    std::fprintf(stderr, "DOF in/out : %d / %d\n", static_cast<int>(inDof), static_cast<int>(outDof));
    std::fprintf(stderr, "workspace  : X[%.1f, %.1f] Y[%.1f, %.1f] Z[%.1f, %.1f] mm\n",
                 ws[0], ws[3], ws[1], ws[4], ws[2], ws[5]);
    std::fprintf(stderr, "calibStyle : 0x%x\n", static_cast<unsigned>(calibStyle));

    hdStartScheduler();
    if ((err = hdGetError()).errorCode != HD_SUCCESS) {
        printHdError("hdStartScheduler");
        hdDisableDevice(hdGetCurrentDevice());
        return 2;
    }

    HDenum calib = HD_CALIBRATION_NEEDS_UPDATE;
    hdScheduleSynchronous(cbCheckCalib, &calib, HD_DEFAULT_SCHEDULER_PRIORITY);

    if (calib != HD_CALIBRATION_OK) {
        if (updateCalib) {
            std::fprintf(stderr, "标定状态 %d，尝试更新（笔要插在墨水井里）...\n",
                         static_cast<int>(calib));
            hdScheduleSynchronous(cbUpdateCalib, &calibStyle, HD_DEFAULT_SCHEDULER_PRIORITY);
            hdScheduleSynchronous(cbCheckCalib, &calib, HD_DEFAULT_SCHEDULER_PRIORITY);
        }
        if (calib != HD_CALIBRATION_OK) {
            std::fprintf(stderr,
                "[标定未通过] 状态 = %s\n"
                "  先把笔插进墨水井跑一次 Touch Smart Setup（或加 --update-calib），\n"
                "  蓝灯慢闪就是缺标定。标定没好之前采到的数据不能用。\n",
                calib == HD_CALIBRATION_NEEDS_MANUAL_INPUT
                    ? "NEEDS_MANUAL_INPUT（等人工操作）"
                    : "NEEDS_UPDATE（需要更新）");
            hdStopScheduler();
            hdDisableDevice(hdGetCurrentDevice());
            return 3;
        }
    }
    std::fprintf(stderr, "calib      : OK\n");

    std::ofstream f(outPath.c_str(), std::ios::binary);
    if (!f) {
        std::fprintf(stderr, "打不开输出文件: %s\n", outPath.c_str());
        hdStopScheduler();
        hdDisableDevice(hdGetCurrentDevice());
        return 4;
    }
    f << "t_ms,x,y,z,vx,vy,vz,gx,gy,gz,btn1,btn2,inkwell\n";

    const auto t0 = Clock::now();
    const std::chrono::duration<double> period(1.0 / hz);
    const long long total = static_cast<long long>(std::llround(seconds * hz));
    const long long progressEvery = static_cast<long long>(hz);  // 每秒一行进度

    Sample s;
    char buf[256];
    long long written = 0;

    for (long long i = 0; i < total; ++i) {
        hdScheduleSynchronous(cbSample, &s, HD_DEFAULT_SCHEDULER_PRIORITY);

        const double tMs =
            std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        const int len = std::snprintf(
            buf, sizeof(buf),
            "%.3f,%.4f,%.4f,%.4f,%.1f,%.1f,%.1f,%.4f,%.4f,%.4f,%d,%d,%d\n",
            tMs,
            s.pos[0], s.pos[1], s.pos[2],
            s.vel[0], s.vel[1], s.vel[2],
            s.gimbal[0], s.gimbal[1], s.gimbal[2],
            (s.buttons & HD_DEVICE_BUTTON_1) ? 1 : 0,
            (s.buttons & HD_DEVICE_BUTTON_2) ? 1 : 0,
            s.inkwell ? 1 : 0);
        if (len > 0) {
            f.write(buf, len);
            ++written;
        }

        if (i % progressEvery == 0) {
            std::fprintf(stderr, "\r采集中 %5.1f / %.1f s   pos=(%7.2f,%7.2f,%7.2f)",
                         tMs / 1000.0, seconds, s.pos[0], s.pos[1], s.pos[2]);
            f.flush();
        }

        // 固定节拍；如果某次调度超时，next 已经过去，就自然进入追赶状态
        const auto next = t0 + std::chrono::duration_cast<Clock::duration>(period * (i + 1));
        std::this_thread::sleep_until(next);
    }

    f.flush();
    f.close();

    const double elapsed =
        std::chrono::duration<double>(Clock::now() - t0).count();
    std::fprintf(stderr, "\n完成: %lld 行, %.2f s, 实际 %.1f Hz -> %s\n",
                 written, elapsed, written / elapsed, outPath.c_str());

    hdStopScheduler();
    hdDisableDevice(hdGetCurrentDevice());
    while (true) {
        const HDErrorInfo e = hdGetError();
        if (e.errorCode == HD_SUCCESS) break;
        std::fprintf(stderr, "[HD ERROR] shutdown: code=0x%x\n",
                     static_cast<unsigned>(e.errorCode));
    }
    return 0;
}
