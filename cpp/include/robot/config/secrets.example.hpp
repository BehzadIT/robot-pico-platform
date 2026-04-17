#pragma once

namespace robot::config {

struct WifiCredentials {
  const char* ssid;
  const char* password;
};

inline constexpr WifiCredentials kWifiCredentials{
    "replace-me",
    "replace-me",
};

}  // namespace robot::config
