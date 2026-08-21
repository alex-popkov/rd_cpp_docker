#include "api/api_client.hpp"

ApiClient::ApiClient(const std::string url, int port, int connectionTimeout, int readTimeout)
  : client(url, port)
{
  client.set_connection_timeout(connectionTimeout);
  client.set_read_timeout(readTimeout);
}

auto ApiClient::get(const std::string& path, const httplib::Headers& headers) -> httplib::Result
{
  return client.Get(path, headers);
}

auto ApiClient::post(const std::string& path,
                     const std::string& body,
                     const std::string& contentType,
                     const httplib::Headers& headers) -> httplib::Result
{
  return client.Post(path, headers, body, contentType);
}
