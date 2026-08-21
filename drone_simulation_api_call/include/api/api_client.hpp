#pragma once
#include <string>
#include "httplib.h"

class ApiClient {
public:
  ApiClient(const std::string url, int port, int connectionTimeout, int readTimeout);

  auto get(const std::string& path, const httplib::Headers& headers = {}) -> httplib::Result;

  auto post(const std::string& path,
            const std::string& body,
            const std::string& contentType = "application/json",
            const httplib::Headers& headers = {}) -> httplib::Result;

private:
  httplib::Client client;
};
