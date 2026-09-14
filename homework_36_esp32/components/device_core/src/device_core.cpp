#include "device_core.hpp"
#include <cstdio>

namespace core {

AccelSample accel_from_raw(int16_t rx, int16_t ry, int16_t rz)
{
    return { rx / 16384.0f, ry / 16384.0f, rz / 16384.0f };
}

const char* mode_from_period(uint32_t period_ms)
{
    return (period_ms <= 250) ? "fast" : "slow";
}

int format_status(char* out, size_t out_size,
                  int64_t t_ms, const AccelSample& a, uint32_t period_ms)
{
    return snprintf(out, out_size,
        "t=%lld ms  ax=%.2f ay=%.2f az=%.2f  mode=%s",
        (long long)t_ms, a.ax, a.ay, a.az, mode_from_period(period_ms));
}

Command parse_command(const char* line)
{
    Command cmd;
    unsigned int val = 0;
    if (sscanf(line, "p %u", &val) == 1) {
        cmd.type = CmdType::SetPeriod;
        cmd.period_ms = val;
        
        return cmd;
    }
    cmd.type = CmdType::Unknown;

    return cmd;
}

}  // namespace core