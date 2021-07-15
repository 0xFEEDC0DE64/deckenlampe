#pragma once

// esp-idf includes
#include <esp_http_server.h>

namespace deckenlampe {
extern httpd_handle_t httpdHandle;

void init_webserver();
void update_webserver();
} // namespace deckenlampe
