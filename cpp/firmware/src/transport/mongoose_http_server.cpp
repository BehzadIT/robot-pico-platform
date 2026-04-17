#include "robot/transport/mongoose_http_server.hpp"

#include <cstdio>
#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>

#include "FreeRTOS.h"
#include "task.h"
#include "robot/protocol/api_drive_request.hpp"
#include "robot/support/logger.hpp"

namespace robot::transport {

namespace {

constexpr const char* kWsTag = "ws";

uint32_t NowMs() {
  return static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

void LogRateLimitedWarning(robot::support::RateLimitFilter& filter, std::string_view message) {
  if (filter.ShouldEmit(NowMs())) {
    robot::support::LogW(kWsTag, message);
  }
}

std::optional<int> FindIntAfter(std::string_view json, std::string_view needle) {
  const std::size_t key_pos = json.find(needle);
  if (key_pos == std::string_view::npos) {
    return std::nullopt;
  }
  std::size_t pos = key_pos + needle.size();
  while (pos < json.size() && (json[pos] == ' ' || json[pos] == ':')) {
    ++pos;
  }
  bool negative = false;
  if (pos < json.size() && json[pos] == '-') {
    negative = true;
    ++pos;
  }
  if (pos >= json.size() || json[pos] < '0' || json[pos] > '9') {
    return std::nullopt;
  }
  int value = 0;
  while (pos < json.size() && json[pos] >= '0' && json[pos] <= '9') {
    value = (value * 10) + (json[pos] - '0');
    ++pos;
  }
  return negative ? -value : value;
}

bool UriMatches(const mg_str& uri, const char* pattern) {
  return mg_match(uri, mg_str(pattern), nullptr);
}

}  // namespace

MongooseHttpServer::~MongooseHttpServer() {
  if (initialized_) {
    mg_mgr_free(&manager_);
  }
}

bool MongooseHttpServer::Start(robot::runtime::ServiceCoreRuntime& runtime,
                               const char* listen_url) {
  runtime_ = &runtime;
  mg_mgr_init(&manager_);
  initialized_ = mg_http_listen(&manager_, listen_url, &StaticEventHandler, this) != nullptr;
  if (!initialized_) {
    robot::support::LogE(kWsTag, std::string("listen failed on ") + listen_url);
  }
  return initialized_;
}

void MongooseHttpServer::Poll(int timeout_ms) {
  if (initialized_) {
    mg_mgr_poll(&manager_, timeout_ms);
  }
}

void MongooseHttpServer::StaticEventHandler(mg_connection* connection,
                                            int ev,
                                            void* ev_data) {
  static_cast<MongooseHttpServer*>(connection->fn_data)->EventHandler(connection, ev, ev_data);
}

void MongooseHttpServer::EventHandler(mg_connection* connection, int ev, void* ev_data) {
  if (ev == MG_EV_HTTP_MSG) {
    HandleHttp(connection, static_cast<mg_http_message*>(ev_data));
  } else if (ev == MG_EV_WS_OPEN) {
    HandleWebSocketOpen(connection);
  } else if (ev == MG_EV_WS_MSG) {
    HandleWebSocket(connection, static_cast<mg_ws_message*>(ev_data));
  } else if (ev == MG_EV_CLOSE) {
    HandleWebSocketClose(connection);
  }
}

std::string MongooseHttpServer::ConnectionSourceId(const mg_connection* connection) const {
  return std::string("ws-") + std::to_string(connection == nullptr ? 0UL : connection->id);
}

void MongooseHttpServer::HandleWebSocketOpen(mg_connection* connection) {
  robot::support::LogI(kWsTag, std::string("websocket open source_id=") + ConnectionSourceId(connection));
}

void MongooseHttpServer::HandleWebSocketClose(mg_connection* connection) {
  if (connection == nullptr || !connection->is_websocket) {
    return;
  }

  const std::string source_id = ConnectionSourceId(connection);
  robot::support::LogI(kWsTag, std::string("websocket close source_id=") + source_id);
  if (!runtime_->QueueDisconnect(source_id)) {
    robot::support::LogW(kWsTag,
                         std::string("websocket close cleanup rejected because control queue is "
                                     "full source_id=") +
                             source_id);
  }
}

void MongooseHttpServer::HandleHttp(mg_connection* connection, mg_http_message* message) {
  static robot::support::RateLimitFilter queue_full_filter(1000);
  static robot::support::RateLimitFilter bad_drive_filter(1000);
  static robot::support::RateLimitFilter route_not_found_filter(1000);

  if (UriMatches(message->uri, "/ws")) {
    mg_ws_upgrade(connection, message, nullptr);
    return;
  }

  const std::string_view body(message->body.buf, message->body.len);
  if (UriMatches(message->uri, "/stop")) {
    if (!runtime_->QueueStop(std::nullopt, "ws-http")) {
      LogRateLimitedWarning(queue_full_filter, "HTTP /stop rejected because control queue is full");
      mg_http_reply(connection, 503, "", "{\"type\":\"error\",\"code\":\"queue_full\"}");
      return;
    }
    WaitAndReply(connection, std::nullopt);
    return;
  }

  if (UriMatches(message->uri, "/recover")) {
    if (!runtime_->QueueRecover(std::nullopt, "ws-http")) {
      LogRateLimitedWarning(queue_full_filter, "HTTP /recover rejected because control queue is full");
      mg_http_reply(connection, 503, "", "{\"type\":\"error\",\"code\":\"queue_full\"}");
      return;
    }
    WaitAndReply(connection, std::nullopt);
    return;
  }

  if (UriMatches(message->uri, "/drive")) {
    const auto rpm = ExtractJsonInt(body, "target_rpm");
    const auto direction = ExtractJsonInt(body, "target_direction");
    const auto angle = ExtractJsonInt(body, "target_angle");
    if (!rpm.has_value() || !direction.has_value() || !angle.has_value()) {
      LogRateLimitedWarning(bad_drive_filter, "HTTP /drive rejected due to bad_drive_command");
      mg_http_reply(connection, 400, "", "{\"type\":\"error\",\"code\":\"bad_drive_command\"}");
      return;
    }
    robot::protocol::ApiDriveRequest request;
    request.target_rpm = *rpm;
    request.target_direction = *direction;
    request.target_angle = *angle;
    if (!runtime_->QueueDrive(request, "ws-http")) {
      LogRateLimitedWarning(queue_full_filter, "HTTP /drive rejected because control queue is full");
      mg_http_reply(connection, 503, "", "{\"type\":\"error\",\"code\":\"queue_full\"}");
      return;
    }
    WaitAndReply(connection, request.seq);
    return;
  }

  LogRateLimitedWarning(route_not_found_filter, "HTTP route not found");
  mg_http_reply(connection, 404, "", "{\"type\":\"error\",\"code\":\"not_found\"}");
}

void MongooseHttpServer::HandleWebSocket(mg_connection* connection, mg_ws_message* message) {
  static robot::support::RateLimitFilter missing_kind_filter(1000);
  static robot::support::RateLimitFilter queue_full_filter(1000);
  static robot::support::RateLimitFilter bad_drive_filter(1000);
  static robot::support::RateLimitFilter unknown_command_filter(1000);
  const std::string source_id = ConnectionSourceId(connection);

  const std::string_view json(message->data.buf, message->data.len);
  const auto kind = ExtractJsonChar(json, "k");
  const auto seq = ExtractJsonInt(json, "q");
  if (!kind.has_value()) {
    LogRateLimitedWarning(missing_kind_filter, "recoverable protocol error: missing command kind");
    ReplyError(connection, "invalid_json", seq, "missing command kind");
    return;
  }

  if (*kind == 's') {
    if (!runtime_->QueueStop(seq, source_id)) {
      LogRateLimitedWarning(queue_full_filter, "stop rejected because control queue is full");
      ReplyError(connection, "queue_full", seq, "control queue full");
      return;
    }
    WaitAndReply(connection, seq);
    return;
  }

  if (*kind == 'r') {
    if (!runtime_->QueueRecover(seq, source_id)) {
      LogRateLimitedWarning(queue_full_filter, "recover rejected because control queue is full");
      ReplyError(connection, "queue_full", seq, "control queue full");
      return;
    }
    WaitAndReply(connection, seq);
    return;
  }

  if (*kind == 'd') {
    const auto throttle = ExtractJsonInt(json, "t");
    const auto turn = ExtractJsonInt(json, "r");
    const auto sent_at = ExtractJsonInt(json, "w");
    if (!throttle.has_value() || !turn.has_value()) {
      LogRateLimitedWarning(bad_drive_filter, "recoverable protocol error: drive command missing t or r");
      ReplyError(connection, "bad_drive_command", seq, "drive command missing t or r");
      return;
    }
    const auto request = robot::protocol::ApiDriveRequest::FromIntent(*throttle, *turn, seq, sent_at);
    if (!runtime_->QueueDrive(request, source_id)) {
      LogRateLimitedWarning(queue_full_filter, "drive rejected because control queue is full");
      ReplyError(connection, "queue_full", seq, "control queue full");
      return;
    }
    WaitAndReply(connection, seq);
    return;
  }

  LogRateLimitedWarning(unknown_command_filter,
                        std::string("recoverable protocol error: unknown command type=") +
                            std::string(1, *kind));
  ReplyError(connection, "unknown_command", seq, "unknown command");
}

void MongooseHttpServer::ReplyAck(mg_connection* connection,
                                  const char* code,
                                  std::optional<int> seq) {
  char buffer[128];
  if (seq.has_value()) {
    std::snprintf(buffer, sizeof(buffer), "{\"k\":\"a\",\"c\":\"%s\",\"q\":%d}", code,
                  *seq);
    mg_ws_send(connection, buffer, std::strlen(buffer), WEBSOCKET_OP_TEXT);
    return;
  }
  std::snprintf(buffer, sizeof(buffer), "{\"k\":\"a\",\"c\":\"%s\"}", code);
  mg_ws_send(connection, buffer, std::strlen(buffer), WEBSOCKET_OP_TEXT);
}

void MongooseHttpServer::ReplyError(mg_connection* connection,
                                    const char* code,
                                    std::optional<int> seq,
                                    const char* message) {
  char buffer[192];
  if (seq.has_value()) {
    std::snprintf(buffer, sizeof(buffer),
                  "{\"k\":\"e\",\"c\":\"%s\",\"q\":%d,\"m\":\"%s\"}", code, *seq,
                  message);
    mg_ws_send(connection, buffer, std::strlen(buffer), WEBSOCKET_OP_TEXT);
    return;
  }
  std::snprintf(buffer, sizeof(buffer), "{\"k\":\"e\",\"c\":\"%s\",\"m\":\"%s\"}",
                code, message);
  mg_ws_send(connection, buffer, std::strlen(buffer), WEBSOCKET_OP_TEXT);
}

void MongooseHttpServer::WaitAndReply(mg_connection* connection, std::optional<int> seq) {
  static robot::support::RateLimitFilter completion_timeout_filter(1000);
  const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(250);
  while (xTaskGetTickCount() <= deadline) {
    if (const auto completion = runtime_->PollCompletion(); completion.has_value()) {
      if (completion->ok) {
        robot::support::LogI(
            kWsTag, std::string("ack ") + completion->code +
                         (completion->seq.has_value()
                              ? std::string(" seq=") + std::to_string(*completion->seq)
                              : std::string("")));
        if (connection->is_websocket) {
          ReplyAck(connection, completion->code.c_str(), completion->seq);
        } else {
          mg_http_reply(connection, 200, "", "{\"ok\":true}");
        }
      } else if (connection->is_websocket) {
        robot::support::LogW(
            kWsTag, std::string("error ") + completion->code +
                         (completion->seq.has_value()
                              ? std::string(" seq=") + std::to_string(*completion->seq)
                              : std::string("")) +
                         " detail=" + completion->detail);
        ReplyError(connection, completion->code.c_str(), completion->seq,
                   completion->detail.c_str());
      } else {
        robot::support::LogW(kWsTag,
                             std::string("HTTP command failed code=") + completion->code +
                                 " detail=" + completion->detail);
        mg_http_reply(connection, 409, "", "{\"ok\":false}");
      }
      return;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  if (connection->is_websocket) {
    LogRateLimitedWarning(completion_timeout_filter, "control completion timeout");
    ReplyError(connection, "timeout", seq, "control completion timeout");
  } else {
    LogRateLimitedWarning(completion_timeout_filter, "HTTP control completion timeout");
    mg_http_reply(connection, 504, "", "{\"ok\":false,\"code\":\"timeout\"}");
  }
}

std::optional<int> MongooseHttpServer::ExtractJsonInt(std::string_view json,
                                                      std::string_view key) {
  char needle[32];
  std::snprintf(needle, sizeof(needle), "\"%.*s\"", static_cast<int>(key.size()), key.data());
  return FindIntAfter(json, needle);
}

std::optional<char> MongooseHttpServer::ExtractJsonChar(std::string_view json,
                                                        std::string_view key) {
  char needle[32];
  std::snprintf(needle, sizeof(needle), "\"%.*s\"", static_cast<int>(key.size()), key.data());
  const std::size_t key_pos = json.find(needle);
  if (key_pos == std::string_view::npos) {
    return std::nullopt;
  }
  const std::size_t quote = json.find('"', key_pos + std::strlen(needle) + 1);
  if (quote == std::string_view::npos || quote + 1 >= json.size()) {
    return std::nullopt;
  }
  return json[quote + 1];
}

}  // namespace robot::transport
