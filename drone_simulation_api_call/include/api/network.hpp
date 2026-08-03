#pragma once
#include "httplib.h"

enum class ApiCallOutcome { Success, DoNotRetry, Retry };

ApiCallOutcome classifyApiCallResult(const httplib::Result& result);
