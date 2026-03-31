#ifndef SRC_RADIO_RM126X_H_
#define SRC_RADIO_RM126X_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "main.h"

#define RM126X_DEFAULT_BAUD_RATE            9600UL
#define RM126X_DEFAULT_WAKE_TIMEOUT_MS      250UL
#define RM126X_DEFAULT_COMMAND_TIMEOUT_MS   1000UL
#define RM126X_DEFAULT_RX_POLL_TIMEOUT_MS   5UL
#define RM126X_DEFAULT_FPORT                1U
#define RM126X_DIAGNOSTIC_TX_INTERVAL       50UL
#define RM126X_MAX_RESPONSE_LINE_LENGTH     128U
#define RM126X_MAX_COMMAND_LENGTH           192U
#define RM126X_MAX_SEND_PAYLOAD_BYTES       96U

#define RM126X_FPORT_NORMAL                 1U
#define RM126X_FPORT_SOIL                   2U
#define RM126X_FPORT_PULSE_COUNT            3U
#define RM126X_FPORT_DIAGNOSTIC             9U
#define RM126X_FPORT_ERROR                  10U
#define RM126X_FPORT_SENSOR_MISMATCH        11U

typedef enum {
    RM126X_RESULT_OK = 0,
    RM126X_RESULT_INVALID_ARG,
    RM126X_RESULT_TIMEOUT,
    RM126X_RESULT_HAL_ERROR,
    RM126X_RESULT_UART_UNSUPPORTED,
    RM126X_RESULT_BUFFER_TOO_SMALL,
    RM126X_RESULT_PARSE_ERROR,
    RM126X_RESULT_MODULE_ERROR
} RM126xResult;

typedef enum {
    RM126X_AT_ERROR_NONE = 0,
    RM126X_AT_ERROR_COMMAND_CANNOT_BE_PROCESSED = 14,
    RM126X_AT_ERROR_VALUE_NOT_VALID = 33,
    RM126X_AT_ERROR_HEX_STRING_INVALID = 49,
    RM126X_AT_ERROR_LORA_ALREADY_JOINED = 80,
    RM126X_AT_ERROR_LORA_NOT_JOINED = 81
} RM126xAtErrorCode;

typedef enum {
    RM126X_APP_STATE_UNKNOWN = -1,
    RM126X_APP_STATE_IDLE = 0,
    RM126X_APP_STATE_JOINING = 1,
    RM126X_APP_STATE_JOINED = 2,
    RM126X_APP_STATE_TX = 3,
    RM126X_APP_STATE_RX = 4,
    RM126X_APP_STATE_DISCONNECTED = 5
} RM126xApplicationState;

typedef enum {
    RM126X_CONNECTION_STATUS_UNKNOWN = -1,
    RM126X_CONNECTION_STATUS_NOT_CONNECTED = 0,
    RM126X_CONNECTION_STATUS_CONNECTED = 1
} RM126xConnectionStatus;

typedef enum {
    RM126X_ASYNC_NONE = 0,
    RM126X_ASYNC_WAKE,
    RM126X_ASYNC_CLASS,
    RM126X_ASYNC_JOIN,
    RM126X_ASYNC_TX,
    RM126X_ASYNC_RX,
    RM126X_ASYNC_LINK_CHECK,
    RM126X_ASYNC_TIME,
    RM126X_ASYNC_SUPPLY
} RM126xAsyncEventType;

typedef struct {
    USART_TypeDef* instance;
    GPIO_TypeDef* tx_port;
    uint16_t tx_pin;
    uint8_t tx_af;
    GPIO_TypeDef* rx_port;
    uint16_t rx_pin;
    uint8_t rx_af;
    uint32_t baud_rate;
    uint32_t kernel_clock_hz;
} RM126xUartConfig;

typedef struct {
    RM126xUartConfig uart;
    uint32_t wake_timeout_ms;
    uint32_t command_timeout_ms;
    uint32_t rx_poll_timeout_ms;
} RM126xConfig;

typedef struct {
    RM126xAsyncEventType type;
    char line[RM126X_MAX_RESPONSE_LINE_LENGTH];
} RM126xAsyncEvent;

typedef struct {
    RM126xConfig config;
    bool uart_ready;
    bool is_connected;
    bool pending_fport9;
    uint8_t current_fport;
    uint32_t accepted_uplink_count;
    RM126xResult last_result;
    RM126xAtErrorCode last_error;
    uint32_t last_error_code;
    RM126xApplicationState last_app_state;
    RM126xConnectionStatus last_connection_status;
    RM126xAsyncEvent last_async_event;
} RM126xHandle;

void RM126x_Init(RM126xHandle* handle, const RM126xConfig* config);
RM126xResult RM126x_HostUartResume(RM126xHandle* handle);
RM126xResult RM126x_HostUartSuspend(RM126xHandle* handle);

RM126xResult RM126x_WakeAndPing(RM126xHandle* handle);
RM126xResult RM126x_At(RM126xHandle* handle);
RM126xResult RM126x_Reset(RM126xHandle* handle);

RM126xResult RM126x_Join(RM126xHandle* handle);
RM126xResult RM126x_SendBytes(RM126xHandle* handle, const uint8_t* data,
                              size_t length);
RM126xResult RM126x_ReadTemperature(RM126xHandle* handle, char* response,
                                    size_t response_size);
RM126xResult RM126x_Sleep(RM126xHandle* handle);
RM126xResult RM126x_SetFPort(RM126xHandle* handle, uint8_t fport);

bool RM126x_IsConnected(RM126xHandle* handle);
RM126xAtErrorCode RM126x_GetLastAtError(const RM126xHandle* handle);
const RM126xAsyncEvent* RM126x_GetLastAsyncEvent(const RM126xHandle* handle);
bool RM126x_ShouldSendDiagnosticPort9(const RM126xHandle* handle);

#ifdef __cplusplus
}
#endif

#endif /* SRC_RADIO_RM126X_H_ */
