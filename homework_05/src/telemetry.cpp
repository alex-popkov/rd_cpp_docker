#include "telemetry.hpp"

#include <cstdlib>
#include <fstream>
#include <iostream>

#define ENABLE_DEBUG 0

#if ENABLE_DEBUG
#define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
#define DEBUG(msg)
#endif

// Debugging exercise notes:
// this file intentionally contains four runtime defects.
// The defects are related to malformed input shape, invalid numeric values,
// unsafe time deltas, and empty logs. Exact locations are not marked on purpose.

const int EXPECTED_FIELD_COUNT = 7;
const int MAX_LINE_LENGTH = 256;
const int SEQ_STEP = 1;
const int ALLOWED_TEMPERATURE_RANGE[2] = {-40, 120};
const int ALLOWED_GPS_FIX_VALUES[2] = {0, 1};

int split_line(char line[], char* fields[], int max_fields)
{
  int count = 0;
  char* cursor = line;

  while (*cursor != '\0' && count < max_fields) {
    while (*cursor == ' ' || *cursor == '\t' || *cursor == '\n' || *cursor == '\r') {
      *cursor = '\0';
      ++cursor;
    }

    if (*cursor == '\0') {
      break;
    }

    fields[count] = cursor;
    ++count;

    while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t' && *cursor != '\n' && *cursor != '\r') {
      ++cursor;
    }
  }

  return count;
}

long parse_long(const char* text)
{
  char* end = nullptr;
  const long value = std::strtol(text, &end, 10);

  if (end == text) {
    throw std::runtime_error("telemetry.cpp: parse_long: Invalid number.");
  }

  return value;
}

int parse_int(const char* text)
{
  return static_cast<int>(parse_long(text));
}

double parse_double(const char* text)
{
  char* end = nullptr;
  const double value = std::strtod(text, &end);

  if (end == text) {
    throw std::runtime_error("telemetry.cpp: parse_double: Invalid number.");
  }

  return value;
}

Frame parse_frame(char line[])
{
  char* fields[EXPECTED_FIELD_COUNT] = {};
  const int field_count = split_line(line, fields, EXPECTED_FIELD_COUNT);

  if (field_count < EXPECTED_FIELD_COUNT) {
    throw std::runtime_error("telemetry.cpp: parse_frame: expected " + std::to_string(EXPECTED_FIELD_COUNT) +
                             " fields. Got: " + std::to_string(field_count));
  }

  Frame frame{};
  frame.timestamp_ms = parse_long(fields[0]);
  frame.seq = parse_int(fields[1]);
  frame.voltage_v = parse_double(fields[2]);

  if (frame.voltage_v <= 0) {
    throw std::runtime_error("telemetry.cpp: parse_frame: voltage should be more then 0. Got: " + std::to_string(frame.voltage_v) +
                             " for frame seq " + std::to_string(frame.seq));
  }

  frame.current_a = parse_double(fields[3]);
  frame.temperature_c = parse_double(fields[4]);

  if (frame.temperature_c < ALLOWED_TEMPERATURE_RANGE[0] || frame.temperature_c > ALLOWED_TEMPERATURE_RANGE[1]) {
    throw std::runtime_error("telemetry.cpp: parse_frame: temperature should be inside the range [" +
                             std::to_string(ALLOWED_TEMPERATURE_RANGE[0]) + ", " + std::to_string(ALLOWED_TEMPERATURE_RANGE[1]) +
                             "] but it's " + std::to_string(frame.temperature_c) + " for frame seq " + std::to_string(frame.seq));
  }

  frame.gps_fix = parse_int(fields[5]);

  if (frame.gps_fix != ALLOWED_GPS_FIX_VALUES[0] && frame.gps_fix != ALLOWED_GPS_FIX_VALUES[1]) {
    throw std::runtime_error("telemetry.cpp: parse_frame: gps_fix should be " + std::to_string(ALLOWED_GPS_FIX_VALUES[0]) + " or " +
                             std::to_string(ALLOWED_GPS_FIX_VALUES[1]) + " but it's " + std::to_string(frame.gps_fix) + " for frame seq " +
                             std::to_string(frame.seq));
  }

  frame.satellites = parse_int(fields[6]);

  if (frame.satellites < 0) {
    throw std::runtime_error("telemetry.cpp: parse_frame: satellites number should be more then 0. But it's " +
                             std::to_string(frame.satellites) + " for frame seq " + std::to_string(frame.seq));
  }

  return frame;
}

double compute_frame_rate_hz(const Frame frames[], int frame_count)
{
  const long elapsed_ms = frames[frame_count - 1].timestamp_ms - frames[0].timestamp_ms;

  return static_cast<double>((frame_count - 1) * 1000 / elapsed_ms);
}

int read_frames(const char* path, Frame frames[], int max_frames)
{
  std::ifstream input{path};

  if (!input) {
    throw std::runtime_error(std::string("telemetry.cpp: read_frames: failed to open input file: ") + path);
  }

  int frame_count = 0;
  char line[MAX_LINE_LENGTH];

  while (input.getline(line, MAX_LINE_LENGTH)) {
    if (line[0] == '\0') {
      continue;
    }

    if (frame_count < max_frames) {
      frames[frame_count] = parse_frame(line);

      if (frame_count > 0) {
        const Frame& prevFrame = frames[frame_count - 1];
        const Frame& currFrame = frames[frame_count];

        if (prevFrame.timestamp_ms >= currFrame.timestamp_ms) {
          throw std::runtime_error("telemetry.cpp: read_frames: non-monotonic timestamp_ms at frame " + std::to_string(frame_count) +
                                   ": previous=" + std::to_string(prevFrame.timestamp_ms) +
                                   ", current=" + std::to_string(currFrame.timestamp_ms));
        }

        if (currFrame.seq - prevFrame.seq != SEQ_STEP) {
          throw std::runtime_error("telemetry.cpp: read_frames: unexpected seq gap at frame " + std::to_string(frame_count) +
                                   ": previous=" + std::to_string(prevFrame.seq) + ", current=" + std::to_string(currFrame.seq) +
                                   ", expected step=" + std::to_string(SEQ_STEP));
        }
      }

      ++frame_count;
    }
  }

  if (frame_count == 0) {
    throw std::runtime_error("telemetry.cpp: read_frames: no telemetry frames in input file");
  }

  return frame_count;
}

Summary summarize(const Frame frames[], int frame_count)
{
  Summary summary{};
  summary.frames_total = frame_count;
  summary.frames_valid = frame_count;
  summary.voltage_min = frames[0].voltage_v;
  summary.voltage_max = frames[0].voltage_v;
  summary.low_voltage_frames = 0;

  double temperature_sum = 0.0;

  for (int i = 0; i < frame_count; ++i) {
    if (frames[i].voltage_v < summary.voltage_min) {
      summary.voltage_min = frames[i].voltage_v;
    }

    if (frames[i].voltage_v > summary.voltage_max) {
      summary.voltage_max = frames[i].voltage_v;
    }

    temperature_sum += frames[i].temperature_c;

    if (frames[i].voltage_v < 22.0) {
      ++summary.low_voltage_frames;
    }
  }

  const int temperature_tenths = static_cast<int>(temperature_sum * 10.0) / frame_count;
  summary.temperature_avg = static_cast<double>(temperature_tenths) / 10.0;
  summary.frame_rate_hz = compute_frame_rate_hz(frames, frame_count);
  return summary;
}

void print_summary(const Summary& summary)
{
  std::cout << "frames_total " << summary.frames_total << '\n';
  std::cout << "frames_valid " << summary.frames_valid << '\n';
  std::cout << "voltage_min " << summary.voltage_min << '\n';
  std::cout << "voltage_max " << summary.voltage_max << '\n';
  std::cout << "temperature_avg " << summary.temperature_avg << '\n';
  std::cout << "low_voltage_frames " << summary.low_voltage_frames << '\n';
  std::cout << "frame_rate_hz " << summary.frame_rate_hz << '\n';
}
