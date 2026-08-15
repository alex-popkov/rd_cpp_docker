#include "c2_controller.hpp"
#include "fc_link.hpp"     // MAVSDK обгортка, API описано у fc_link.hpp
#include "udp_socket.hpp"  // UDP прийом, API описано у udp_socket.hpp
#include "log.hpp"
#include <nlohmann/json.hpp>  // Розбiр JSON з точками маршруту вiд auto_stub

#include <fstream>
#include <iostream>
#include <string>

using json = nlohmann::json;

static constexpr uint16_t STUB_PORT = 14560;
static const std::string LOG_FILE_PATH = "/var/log/c2/c2.log";

static const std::string state_to_string(C2State state)
{
  switch (state) {
    case C2State::DISARMED: {
      return "DISARMED";
    }
    case C2State::ARMED_HOLD: {
      return "ARMED_HOLD";
    }
    case C2State::ARMED_GUIDED: {
      return "ARMED_GUIDED";
    }
    case C2State::ARMED_MANUAL: {
      return "ARMED_MANUAL";
    }
  }
  return "UNKNOWN";
}

struct C2Controller::Impl {
  C2State state = C2State::DISARMED;

  // TODO: додати FcLink, UdpSocket, лог-файл та прапорцi стану.
  // FcLink потребує fc_port у конструкторi Impl.
  // UdpSocket має слухати STUB_PORT.

  FcLink fc;
  UdpSocket updSocket;
  std::ofstream log_file;
  bool healthy_written = false;

  Impl(uint16_t fc_port)
    : fc(fc_port)
    , updSocket(STUB_PORT)
  {
    log_file.open(LOG_FILE_PATH, std::ios::app);
  };

  void transition(C2State next)
  {
    if (next == state) {
      return;
    }

    log(std::string("[C2] state: ") + state_to_string(state) + " -> " + state_to_string(next));
    state = next;
  }

  void log(const std::string& message)
  {
    LOG(message);

    if (!log_file.is_open()) {
      return;
    }
    log_file << message << std::endl;
  }

  void close_log_file()
  {
    if (log_file.is_open()) {
      log_file.close();
    }
  }
};

C2Controller::C2Controller(uint16_t fc_port)
  : impl_(std::make_unique<Impl>(fc_port))
{
}

C2Controller::~C2Controller()
{
  this->impl_->close_log_file();
};

void C2Controller::tick()
{
  if (!this->impl_->healthy_written && this->impl_->fc.is_connected()) {
    std::ofstream("/tmp/c2_healthy").close();
    this->impl_->healthy_written = true;
  }

  C2State state = C2State::DISARMED;
  if (this->impl_->fc.is_armed() == true) {
    switch (this->impl_->fc.flight_mode()) {
      case FcLink::FlightMode::Unknown:
        return;

      case FcLink::FlightMode::Manual:
        // не заважати ручному керуванню
        state = C2State::ARMED_MANUAL;
        break;

      case FcLink::FlightMode::Hold:
        // один раз на вхiд у стан надiслати fc.hold()
        state = C2State::ARMED_HOLD;
        break;

      case FcLink::FlightMode::Guided:
        // передавати
        // розбирати JSON i викликати fc.go_to_ned(north, east)
        state = C2State::ARMED_GUIDED;
        break;
    }
  }

  const bool entering_hold = (state == C2State::ARMED_HOLD && this->impl_->state != C2State::ARMED_HOLD);
  this->impl_->transition(state);
  if (entering_hold) {
    this->impl_->fc.hold();
  }

  char buf[1024];
  ssize_t n = this->impl_->updSocket.recv(buf, sizeof(buf) - 1);
  if (n <= 0) {
    return;
  }
  buf[n] = '\0';

  if (this->impl_->state == C2State::ARMED_GUIDED) {
    try {
      auto waypoint = json::parse(buf);
      float north = waypoint.at("north_m").get<float>();
      float east = waypoint.at("east_m").get<float>();
      this->impl_->fc.go_to_ned(north, east);
      this->impl_->log("[C2] fwd: north=" + std::to_string(north) + " east=" + std::to_string(east));
    }
    catch (const std::exception& e) {
      DEBUG("Error: waypoint failed " << e.what());
    }
  }
  else {
    this->impl_->log(std::string("[C2] blocked: waypoint in ") + state_to_string(this->impl_->state));
  }
}

C2State C2Controller::current_state() const
{
  return impl_->state;
}
