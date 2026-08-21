#include "api/network.hpp"

ApiCallOutcome classifyApiCallResult(const httplib::Result& result)
{
  if (!result) {
    return ApiCallOutcome::Retry;
  }
  switch (result->status) {
    case 200:
    case 201:
      return ApiCallOutcome::Success;
    case 400:
    case 401:
      return ApiCallOutcome::DoNotRetry;
    case 503:
      return ApiCallOutcome::Retry;
    default:
      return ApiCallOutcome::DoNotRetry;
  }
}
