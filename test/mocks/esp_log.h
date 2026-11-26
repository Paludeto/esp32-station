#ifndef MOCK_ESP_LOG_H
#define MOCK_ESP_LOG_H

#include <stdio.h>

#define MOCK_ESP_LOG_NOP(tag, ...) do { \
    if (0) {                            \
        (void)(tag);                    \
        printf(__VA_ARGS__);            \
    }                                   \
} while (0)

#define ESP_LOGE(tag, ...) MOCK_ESP_LOG_NOP(tag, __VA_ARGS__)
#define ESP_LOGW(tag, ...) MOCK_ESP_LOG_NOP(tag, __VA_ARGS__)
#define ESP_LOGI(tag, ...) MOCK_ESP_LOG_NOP(tag, __VA_ARGS__)
#define ESP_LOGD(tag, ...) MOCK_ESP_LOG_NOP(tag, __VA_ARGS__)

#endif /* MOCK_ESP_LOG_H */
