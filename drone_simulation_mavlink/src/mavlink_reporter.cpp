#include <cmath>
#include <common/mavlink.h>  // єдине місце, де підключаємо MAVLink
#include "mavlink_reporter.hpp"
#include "log.hpp"

namespace {
constexpr double LAT0 = 50.4501;
constexpr double LON0 = 30.5234;
constexpr double LEN_DEGREE = 111320.0;
constexpr uint8_t SYSID = 1;
constexpr uint8_t COMPID = MAV_COMP_ID_AUTOPILOT1;

void sendMsg(UdpPort& udp, const mavlink_message_t& msg)
{
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  udp.send(buf, len);
}
}  // namespace

MavlinkReporter::MavlinkReporter(const std::string& host, uint16_t port)
  : udp(host, port)
{
  // за замовчуванням pack може слати MAVLink 1. Ставимо v2 на каналі 0.
  mavlink_set_proto_version(MAVLINK_COMM_0, 2);
}

auto MavlinkReporter::localToGps(float x, float y, double& lat, double& lon) -> void
{
  lat = LAT0 + static_cast<double>(y) / LEN_DEGREE;
  lon = LON0 + static_cast<double>(x) / (LEN_DEGREE * std::cos(LAT0 * M_PI / 180.0));
}

auto MavlinkReporter::sendHeartbeat() -> void
{
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(SYSID,
                             COMPID,
                             &msg,
                             MAV_TYPE_QUADROTOR,     // type
                             MAV_AUTOPILOT_GENERIC,  // autopilot
                             0,                      // base_mode
                             0,                      // custom_mode
                             MAV_STATE_ACTIVE);      // system_status
  sendMsg(this->udp, msg);
}

auto MavlinkReporter::sendTelemetry(const dlink::Telemetry& t) -> void
{
  double lat, lon;
  localToGps(t.x, t.y, lat, lon);

  // --- GLOBAL_POSITION_INT ---
  int32_t lat_e7 = static_cast<int32_t>(std::lround(lat * 1e7));  // градуси ×1e7
  int32_t lon_e7 = static_cast<int32_t>(std::lround(lon * 1e7));
  int32_t alt_mm = static_cast<int32_t>(std::lround(t.z * 1000.0f));  // висота, мм
  int32_t rel_mm = alt_mm;                                            // relative_alt = та сама висота
  // Швидкість рахуємо зі speed+dir
  // Симулятор: x=схід=speed*cos(dir), y=північ=speed*sin(dir).
  float east = t.speed * std::cos(t.dir);
  float north = t.speed * std::sin(t.dir);

  // MAVLink: vx = НА ПІВНІЧ, vy = НА СХІД (не вздовж x/y!)
  int16_t vx_cms = static_cast<int16_t>(std::lround(north * 100.0f));  // см/с
  int16_t vy_cms = static_cast<int16_t>(std::lround(east * 100.0f));
  int16_t vz_cms = 0;  // вертикальної швидкості не рахуємо

  // hdg — КОМПАСНИЙ азимут: 0=північ, за годинниковою.
  // Переклад з матем. кута (0=схід, проти годинникової): bearing = 90 - dir.
  float deg = 90.0f - t.dir * 180.0f / static_cast<float>(M_PI);
  while (deg < 0.0f) {
    deg += 360.0f;
  }
  while (deg >= 360.0f) {
    deg -= 360.0f;
  }

  uint16_t hdg_cdeg = static_cast<uint16_t>(std::lround(deg * 100.0f)) % 36000;

  mavlink_message_t pos;
  mavlink_msg_global_position_int_pack(SYSID, COMPID, &pos, t.t_ms, lat_e7, lon_e7, alt_mm, rel_mm, vx_cms, vy_cms, vz_cms, hdg_cdeg);
  sendMsg(this->udp, pos);

  // --- ATTITUDE ---
  // yaw — той самий компасний курс, у радіанах, нормалізований у [-pi, pi].
  float yaw = static_cast<float>(M_PI) / 2.0f - t.dir;  // 90 - dir
  while (yaw > static_cast<float>(M_PI)) {
    yaw -= 2.0f * static_cast<float>(M_PI);
  }
  while (yaw < -static_cast<float>(M_PI)) {
    yaw += 2.0f * static_cast<float>(M_PI);
  }

  mavlink_message_t att;
  mavlink_msg_attitude_pack(SYSID,
                            COMPID,
                            &att,
                            t.t_ms,  // time_boot_ms
                            0.0f,
                            0.0f,  // roll, pitch — нулі (модель їх не рахує)
                            yaw,   // yaw — компасний курс у радіанах
                            0.0f,
                            0.0f,
                            0.0f);  // *speed — нулі
  sendMsg(this->udp, att);
}

auto MavlinkReporter::sendDropCommand(double lat, double lon, float altM, uint8_t confirmation) -> void
{
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(SYSID,
                                COMPID,
                                &msg,
                                SYSID,
                                COMPID,          // target_system, target_component
                                MAV_CMD_USER_1,  // command
                                confirmation,    // 0 на першій спробі, далі 1,2,3,4
                                0.0f,
                                0.0f,
                                0.0f,
                                0.0f,                     // param1..4 — не використовуємо
                                static_cast<float>(lat),  // param5 = lat (градуси)
                                static_cast<float>(lon),  // param6 = lon (градуси)
                                altM);                    // param7 = висота, метри
  sendMsg(this->udp, msg);
}

auto MavlinkReporter::pollDropAck() -> bool
{
  uint8_t buf[512];
  mavlink_message_t msg;
  mavlink_status_t status;

  // в циклі витягуємо все, що вже могло накопичитись
  int n;
  while ((n = this->udp.recv(buf, sizeof(buf))) > 0) {
    for (int i = 0; i < n; ++i) {
      if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
        if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
          mavlink_command_ack_t ack;
          mavlink_msg_command_ack_decode(&msg, &ack);
          if (ack.command == MAV_CMD_USER_1 && ack.result == MAV_RESULT_ACCEPTED) {
            return true;  // саме на нашу команду і саме ACCEPTED
          }
        }
      }
    }
  }

  return false;
}