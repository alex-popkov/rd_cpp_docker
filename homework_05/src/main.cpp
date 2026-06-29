#include "telemetry.hpp"

#include <iostream>

#define ERROR(msg) std::cerr << "[ERROR] " << msg << std::endl

int main(int argc, char** argv)
{
  // The executable expects exactly one telemetry log path.
  if (argc != 2) {
    std::cerr << "usage: balistics_check <input_path>\n";
    return 1;
  }

  try {
    Frame frames[MAX_TELEMETRY_FRAMES];
    const int frame_count = read_frames(argv[1], frames, MAX_TELEMETRY_FRAMES);

    const Summary summary = summarize(frames, frame_count);
    print_summary(summary);

    return 0;
  }
  catch (const std::exception& error) {
    ERROR("Error: " << error.what() << "\n");

    return 1;
  }
}
