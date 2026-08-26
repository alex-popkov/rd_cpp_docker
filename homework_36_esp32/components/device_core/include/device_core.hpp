#pragma once
#include <cstdint>
#include <cstddef>

namespace core {

enum class CmdType { 
    None, 
    SetPeriod,
    Unknown
};

struct AccelSample { float ax, ay, az; };

struct Command {
    CmdType type= CmdType::None;
    uint32_t period_ms = 0;
};

AccelSample accel_from_raw(int16_t rx, int16_t ry, int16_t rz);

const char* mode_from_period(uint32_t period_ms);

int format_status(char* out, size_t out_size,
                  int64_t t_ms, const AccelSample& a, uint32_t period_ms);

Command parse_command(const char* line);

}  // namespace core