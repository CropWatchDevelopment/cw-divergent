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

typedef enum {
    RM126X_INFO_DEVICE_TYPE = 0,
    RM126X_INFO_APP_FIRMWARE_VERSION = 3,
    RM126X_INFO_APPLICATION_STATE = 42,
    RM126X_INFO_MINIMUM_BAUD_RATE = 1002,
    RM126X_INFO_MAXIMUM_BAUD_RATE = 1003,
    RM126X_INFO_RESET_REASON = 2000,
    RM126X_INFO_LORA_RADIO_ACTIVITY = 2016,
    RM126X_INFO_CONNECTION_STATUS = 3001,
    RM126X_INFO_MODULE_TYPE = 3005
} RM126xInfoId;

typedef enum {
    RM126X_SREG_STARTUP_FLAGS = 100,
    RM126X_SREG_JOIN_STARTUP_FLAG = 117,
    RM126X_SREG_CONNECTION_STATUS_SIO = 205,
    RM126X_SREG_UART_IDLE_TIME_MS = 213,
    RM126X_SREG_UART_BAUD_RATE = 302,
    RM126X_SREG_ADR_ENABLED = 600,
    RM126X_SREG_ACTIVATION_MODE = 602,
    RM126X_SREG_DEVICE_CLASS = 603,
    RM126X_SREG_CONFIRMED_PACKETS = 604,
    RM126X_SREG_CONFIRMED_RETRY_COUNT = 605,
    RM126X_SREG_JOIN_DELAY_MS = 606,
    RM126X_SREG_JOIN_JITTER_MS = 607,
    RM126X_SREG_PING_SLOT_PERIODICITY = 609,
    RM126X_SREG_REGION = 611,
    RM126X_SREG_SUB_BAND = 617,
    RM126X_SREG_APPLICATION_PORT = 629,
    RM126X_SREG_PREAMBLE_COUNT = 711,
    RM126X_SREG_STATIC_ADR_DATARATE = 713,
    RM126X_SREG_STATIC_ADR_TX_POWER = 714,
    RM126X_SREG_TX_POWER_LIMIT = 715
} RM126xSRegisterId;

typedef enum {
    RM126X_SREG_STRING_APP_KEY = 500,
    RM126X_SREG_STRING_DEV_EUI = 501,
    RM126X_SREG_STRING_JOIN_EUI = 502,
    RM126X_SREG_STRING_DEVICE_ADDRESS = 635,
    RM126X_SREG_STRING_NETWORK_SESSION_KEY = 636,
    RM126X_SREG_STRING_APP_SESSION_KEY = 637
} RM126xStringRegisterId;

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
    int32_t min_value;
    int32_t max_value;
} RM126xNumericRange;

typedef struct {
    uint16_t min_length;
    uint16_t max_length;
} RM126xStringRange;

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

RM126xResult RM126x_Command(RM126xHandle* handle, const char* command,
                            char* response, size_t response_size);
RM126xResult RM126x_WakeAndPing(RM126xHandle* handle);
RM126xResult RM126x_At(RM126xHandle* handle);
RM126xResult RM126x_Save(RM126xHandle* handle);
RM126xResult RM126x_Reset(RM126xHandle* handle);

RM126xResult RM126x_GetInfoInt(RM126xHandle* handle, RM126xInfoId info_id,
                               int32_t* value);
RM126xResult RM126x_GetInfoString(RM126xHandle* handle, RM126xInfoId info_id,
                                  char* value, size_t value_size);
RM126xResult RM126x_SetNumericRegister(RM126xHandle* handle,
                                       RM126xSRegisterId register_id,
                                       int32_t value);
RM126xResult RM126x_GetNumericRegister(RM126xHandle* handle,
                                       RM126xSRegisterId register_id,
                                       int32_t* value);
RM126xResult RM126x_GetNumericRegisterRange(RM126xHandle* handle,
                                            RM126xSRegisterId register_id,
                                            RM126xNumericRange* range);
RM126xResult RM126x_SetStringRegister(RM126xHandle* handle,
                                      RM126xStringRegisterId register_id,
                                      const char* value);
RM126xResult RM126x_GetStringRegister(RM126xHandle* handle,
                                      RM126xStringRegisterId register_id,
                                      char* value, size_t value_size);
RM126xResult RM126x_GetStringRegisterRange(RM126xHandle* handle,
                                           RM126xStringRegisterId register_id,
                                           RM126xStringRange* range);

RM126xResult RM126x_I2CRead(RM126xHandle* handle, const char* hex_data,
                            uint32_t out_length, char* response,
                            size_t response_size);
RM126xResult RM126x_I2CWrite(RM126xHandle* handle, const char* hex_data);
RM126xResult RM126x_Drop(RM126xHandle* handle);
RM126xResult RM126x_Join(RM126xHandle* handle);
RM126xResult RM126x_MulticastGroupAdd(RM126xHandle* handle, uint8_t group_id,
                                      uint32_t group_address,
                                      const char* ns_key_hex);
RM126xResult RM126x_MulticastGroupEnd(RM126xHandle* handle, uint8_t group_id);
RM126xResult RM126x_MulticastGroupList(RM126xHandle* handle, uint8_t group_id,
                                       char* response, size_t response_size);
RM126xResult RM126x_MulticastGroupRemove(RM126xHandle* handle, uint8_t group_id);
RM126xResult RM126x_MulticastGroupStart(RM126xHandle* handle, uint8_t group_id);
RM126xResult RM126x_DutyCheck(RM126xHandle* handle, uint8_t payload_length);
RM126xResult RM126x_LinkCheck(RM126xHandle* handle);
RM126xResult RM126x_SendHex(RM126xHandle* handle, const char* hex_data);
RM126xResult RM126x_SendBytes(RM126xHandle* handle, const uint8_t* data,
                              size_t length);
RM126xResult RM126x_TimeRequest(RM126xHandle* handle);
RM126xResult RM126x_P2PEnable(RM126xHandle* handle);
RM126xResult RM126x_P2PStart(RM126xHandle* handle);
RM126xResult RM126x_P2PTransmit(RM126xHandle* handle, uint8_t peer_id,
                                const char* data);
RM126xResult RM126x_SioConfigure(RM126xHandle* handle, uint8_t sio_number,
                                 uint8_t function, int32_t sub_function);
RM126xResult RM126x_SioRead(RM126xHandle* handle, uint8_t sio_number,
                            char* response, size_t response_size);
RM126xResult RM126x_SioWrite(RM126xHandle* handle, uint8_t sio_number,
                             uint8_t value);
RM126xResult RM126x_SpiRead(RM126xHandle* handle, uint8_t device_id,
                            const char* hex_data, uint32_t out_length,
                            char* response, size_t response_size);
RM126xResult RM126x_SpiWrite(RM126xHandle* handle, uint8_t device_id,
                             const char* hex_data);
RM126xResult RM126x_ContinuousReceive(RM126xHandle* handle, uint8_t start_test,
                                      const char* ctx_dev_eui);
RM126xResult RM126x_ContinuousTransmit(RM126xHandle* handle);
RM126xResult RM126x_ReadTemperature(RM126xHandle* handle, char* response,
                                    size_t response_size);
RM126xResult RM126x_SetBatteryStatus(RM126xHandle* handle,
                                     uint8_t battery_status);
RM126xResult RM126x_Sleep(RM126xHandle* handle);
RM126xResult RM126x_SupplyCancel(RM126xHandle* handle);
RM126xResult RM126x_SupplyImmediate(RM126xHandle* handle);
RM126xResult RM126x_SupplyScheduled(RM126xHandle* handle);

RM126xResult RM126x_ReadApplicationState(RM126xHandle* handle,
                                         RM126xApplicationState* state);
RM126xResult RM126x_ReadConnectionStatus(RM126xHandle* handle,
                                         RM126xConnectionStatus* status);
RM126xResult RM126x_ReadPort(RM126xHandle* handle, uint8_t* fport);
RM126xResult RM126x_SetFPort(RM126xHandle* handle, uint8_t fport);
RM126xResult RM126x_RefreshState(RM126xHandle* handle);

bool RM126x_IsConnected(RM126xHandle* handle);
uint32_t RM126x_GetLastErrorCode(const RM126xHandle* handle);
RM126xAtErrorCode RM126x_GetLastAtError(const RM126xHandle* handle);
const RM126xAsyncEvent* RM126x_GetLastAsyncEvent(const RM126xHandle* handle);
bool RM126x_ShouldSendDiagnosticPort9(const RM126xHandle* handle);
void RM126x_SetAcceptedUplinkCount(RM126xHandle* handle, uint32_t count);

#ifdef __cplusplus
}
#endif

#endif /* SRC_RADIO_RM126X_H_ */
