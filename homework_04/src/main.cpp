#include <iostream>
#include <fstream>
#include <cmath> 

#define ENABLE_LOG	1
#define ENABLE_DEBUG  1
 
#if ENABLE_LOG
  #define LOG(msg) std::cout << "[LOG] " << msg << std::endl
#else
  #define LOG(msg)
#endif
 
#if ENABLE_DEBUG
  #define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
  #define DEBUG(msg)
#endif

struct RobotState {
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;
    long timestamp_ms = 0l;
};

struct Tick {
    long timestamp_ms;
    long fl_ticks;
    long fr_ticks;
    long bl_ticks;
    long br_ticks;

    Tick operator-(const Tick& prev) const {
    	Tick result;
        result.fl_ticks = fl_ticks - prev.fl_ticks;
        result.fr_ticks = fr_ticks - prev.fr_ticks;
        result.bl_ticks = bl_ticks - prev.bl_ticks;
        result.br_ticks = br_ticks - prev.br_ticks;
        result.timestamp_ms = timestamp_ms;

        return result;
	}
};

struct TickArray {
    Tick* data;
    std::size_t count;
};

TickArray readTicksFile(char* path) {
    std::ifstream input(path);

    if (!input.is_open()) {
        DEBUG("Could not open input file\n");

        throw std::runtime_error("Could not open input file\n ");
    }

    std::size_t count = 0;
    std::string line;

    while (std::getline(input, line)) {
        if (!line.empty()) {
            ++count;
        }
    }

    input.clear();
    input.seekg(0);

    Tick* ticks = new Tick[count];
    for (std::size_t i = 0; i < count; ++i) {
        input >> ticks[i].timestamp_ms
              >> ticks[i].fl_ticks
              >> ticks[i].fr_ticks
              >> ticks[i].bl_ticks
              >> ticks[i].br_ticks;
    }

    input.close();

    return { ticks, count };
}

float getDistanceFromTick(float deltaTick, float distance_per_tick) {
    return deltaTick * distance_per_tick;
}

float getArithmeticMean(float one, float two) {
    return (one + two) / 2.0f;
}

RobotState updateRobotState(const RobotState& robot_state, const Tick& delta, float wheelbase_m, float distance_per_tick) {
    float delta_left  = getArithmeticMean(delta.fl_ticks, delta.bl_ticks);
    float delta_right = getArithmeticMean(delta.fr_ticks, delta.br_ticks);
    float distance_left = getDistanceFromTick(delta_left, distance_per_tick);
    float distance_right = getDistanceFromTick(delta_right, distance_per_tick);
    float distance_center = getArithmeticMean(distance_left, distance_right);
    float delta_theta = (distance_right - distance_left) / wheelbase_m;

    RobotState updated_state = robot_state;

    updated_state.x += distance_center * std::cos(robot_state.theta +  delta_theta / 2.0f);
    updated_state.y += distance_center * std::sin(robot_state.theta +  delta_theta / 2.0f);
    updated_state.theta += delta_theta;
    updated_state.timestamp_ms = delta.timestamp_ms;

    return updated_state;
}

void writeOutput(RobotState* robot_states, std::size_t count){
    std::ofstream output("homework_04/src/output.txt");
    
    if (!output.is_open()) { 
        DEBUG("Could not open output file\n");

        throw std::runtime_error("Could not open output file\n ");
    }

    for (std::size_t i = 0; i < count; ++i) {

        output << robot_states[i].timestamp_ms << " " 
            << robot_states[i].x << " "
            << robot_states[i].y << " "
            << robot_states[i].theta
            << "\n";
    }

    output.close();
}

void freeTickArray(TickArray tick_array) {
    delete[] tick_array.data;
    tick_array.data = nullptr;
}

void freeRobotStates(RobotState* robot_states) {
    delete[] robot_states;
    robot_states = nullptr;
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: ugv_odometry <input_path>\n";
        return 1;
    }


    TickArray tick_array{ nullptr, 0 };
    RobotState* robot_states = nullptr; 

    try {
        const int ticks_per_revolution = 1024;
        const float wheel_radius_m = 0.3f;
        const float wheelbase_m = 1.0f;
        const float distance_per_tick = 2 * M_PI * wheel_radius_m / ticks_per_revolution;

        tick_array = readTicksFile(argv[1]);

        RobotState robot_state;
        robot_states = new RobotState[tick_array.count]{{0 , 0, 0, 0}};

        for (std::size_t i = 1; i < tick_array.count; ++i) {
            Tick delta_tick = tick_array.data[i] - tick_array.data[i - 1];
            robot_state = updateRobotState(robot_state, delta_tick, wheelbase_m, distance_per_tick);
            robot_states[i] = robot_state;
        }

        writeOutput(robot_states, tick_array.count);

        freeTickArray(tick_array);
        freeRobotStates(robot_states);
    } catch (const std::exception& error) {
        LOG("Error: " << error.what());
        freeTickArray(tick_array);
        freeRobotStates(robot_states);

        return 1;
    }

    return 0;
}
