#include "RM126x.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#define RM126X_UART_WAIT_READY_TIMEOUT_MS  20UL

enum {
    RM126X_INFO_ID_APPLICATION_STATE = 42U,
    RM126X_INFO_ID_CONNECTION_STATUS = 3001U,
    RM126X_SREG_ID_APPLICATION_PORT = 629U
};

typedef struct {
    bool saw_wake;
    bool saw_ok;
    bool saw_error;
    uint32_t error_code;
    bool has_payload;
    char payload[RM126X_MAX_RESPONSE_LINE_LENGTH];
} RM126xResponseState;

static RM126xResult RM126x_RefreshState(RM126xHandle* handle);
static bool rm126x_append_char(char* buffer, size_t buffer_size, size_t* offset,
                               char value);
static bool rm126x_append_cstr(char* buffer, size_t buffer_size, size_t* offset,
                               const char* value);
static bool rm126x_append_u32(char* buffer, size_t buffer_size, size_t* offset,
                              uint32_t value);
static bool rm126x_append_i32(char* buffer, size_t buffer_size, size_t* offset,
                              int32_t value);
static RM126xResult rm126x_execute_command(RM126xHandle* handle, char* response,
                                           size_t response_size,
                                           uint32_t timeout_ms,
                                           const char* command);
static RM126xResult rm126x_build_query_int_command(char* command,
                                                   size_t command_size,
                                                   char family,
                                                   uint16_t register_id);
static RM126xResult rm126x_build_set_numeric_command(char* command,
                                                     size_t command_size,
                                                     uint16_t register_id,
                                                     int32_t value);
static RM126xResult rm126x_build_send_hex_command(char* command,
                                                  size_t command_size,
                                                  const char* hex_data);

static void rm126x_update_pending_fport9(RM126xHandle* handle) {
    uint32_t next_tx_count;

    if (handle == NULL) {
        return;
    }

    next_tx_count = handle->accepted_uplink_count + 1UL;
    handle->pending_fport9 =
        ((next_tx_count % RM126X_DIAGNOSTIC_TX_INTERVAL) == 0UL);
}

static void rm126x_reset_last_status(RM126xHandle* handle) {
    if (handle == NULL) {
        return;
    }

    handle->last_result = RM126X_RESULT_OK;
    handle->last_error = RM126X_AT_ERROR_NONE;
    handle->last_error_code = 0UL;
}

static uint32_t rm126x_elapsed_since(uint32_t start_tick) {
    return HAL_GetTick() - start_tick;
}

static bool rm126x_wait_flag_set(__IO uint32_t* reg, uint32_t mask,
                                 uint32_t timeout_ms) {
    uint32_t start_tick = HAL_GetTick();

    while (((*reg) & mask) != mask) {
        if (rm126x_elapsed_since(start_tick) >= timeout_ms) {
            return false;
        }
    }

    return true;
}

static RM126xResult rm126x_configure_uart_clock(const RM126xHandle* handle) {
    RCC_PeriphCLKInitTypeDef clock_init = {0};

    if ((handle == NULL) || (handle->config.uart.instance == NULL)) {
        return RM126X_RESULT_INVALID_ARG;
    }

    if (handle->config.uart.instance == USART2) {
        clock_init.PeriphClockSelection = RCC_PERIPHCLK_USART2;
        clock_init.Usart2ClockSelection = RCC_USART2CLKSOURCE_HSI;
    } else if (handle->config.uart.instance == USART1) {
        clock_init.PeriphClockSelection = RCC_PERIPHCLK_USART1;
        clock_init.Usart1ClockSelection = RCC_USART1CLKSOURCE_HSI;
    } else {
        return RM126X_RESULT_UART_UNSUPPORTED;
    }

    if (HAL_RCCEx_PeriphCLKConfig(&clock_init) != HAL_OK) {
        return RM126X_RESULT_HAL_ERROR;
    }

    return RM126X_RESULT_OK;
}

static RM126xResult rm126x_enable_uart_clock(const RM126xHandle* handle) {
    if ((handle == NULL) || (handle->config.uart.instance == NULL)) {
        return RM126X_RESULT_INVALID_ARG;
    }

    if (handle->config.uart.instance == USART2) {
        __HAL_RCC_USART2_CLK_ENABLE();
        return RM126X_RESULT_OK;
    }

    if (handle->config.uart.instance == USART1) {
        __HAL_RCC_USART1_CLK_ENABLE();
        return RM126X_RESULT_OK;
    }

    return RM126X_RESULT_UART_UNSUPPORTED;
}

static void rm126x_disable_uart_clock(const RM126xHandle* handle) {
    if ((handle == NULL) || (handle->config.uart.instance == NULL)) {
        return;
    }

    if (handle->config.uart.instance == USART2) {
        __HAL_RCC_USART2_CLK_DISABLE();
    } else if (handle->config.uart.instance == USART1) {
        __HAL_RCC_USART1_CLK_DISABLE();
    }
}

static void rm126x_enable_gpio_clock(GPIO_TypeDef* port) {
    if (port == GPIOA) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    } else if (port == GPIOB) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    } else if (port == GPIOC) {
        __HAL_RCC_GPIOC_CLK_ENABLE();
#ifdef GPIOH
    } else if (port == GPIOH) {
        __HAL_RCC_GPIOH_CLK_ENABLE();
#endif
    }
}

static void rm126x_configure_uart_gpio(const RM126xHandle* handle) {
    GPIO_InitTypeDef gpio_init = {0};

    rm126x_enable_gpio_clock(handle->config.uart.tx_port);
    rm126x_enable_gpio_clock(handle->config.uart.rx_port);

    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;

    gpio_init.Pin = handle->config.uart.tx_pin;
    gpio_init.Alternate = handle->config.uart.tx_af;
    HAL_GPIO_Init(handle->config.uart.tx_port, &gpio_init);

    gpio_init.Pin = handle->config.uart.rx_pin;
    gpio_init.Alternate = handle->config.uart.rx_af;
    HAL_GPIO_Init(handle->config.uart.rx_port, &gpio_init);
}

static uint32_t rm126x_compute_brr(uint32_t clock_hz, uint32_t baud_rate) {
    return (clock_hz + (baud_rate / 2UL)) / baud_rate;
}

static void rm126x_clear_uart_errors(USART_TypeDef* uart) {
    uart->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_PECF |
                USART_ICR_NCF | USART_ICR_IDLECF | USART_ICR_TCCF;
}

static RM126xResult rm126x_uart_enable(RM126xHandle* handle) {
    USART_TypeDef* uart;
    uint32_t brr;

    if ((handle == NULL) || (handle->config.uart.instance == NULL) ||
        (handle->config.uart.baud_rate == 0UL) ||
        (handle->config.uart.kernel_clock_hz == 0UL)) {
        return RM126X_RESULT_INVALID_ARG;
    }

    uart = handle->config.uart.instance;
    brr = rm126x_compute_brr(handle->config.uart.kernel_clock_hz,
                             handle->config.uart.baud_rate);
    if (brr == 0UL) {
        return RM126X_RESULT_INVALID_ARG;
    }

    uart->CR1 = 0UL;
    uart->CR2 = 0UL;
    uart->CR3 = 0UL;
    uart->BRR = brr;
    rm126x_clear_uart_errors(uart);
    uart->CR1 = USART_CR1_TE | USART_CR1_RE;
    uart->CR1 |= USART_CR1_UE;

    if (!rm126x_wait_flag_set(&uart->ISR,
                              USART_ISR_TEACK | USART_ISR_REACK,
                              RM126X_UART_WAIT_READY_TIMEOUT_MS)) {
        return RM126X_RESULT_TIMEOUT;
    }

    handle->uart_ready = true;
    return RM126X_RESULT_OK;
}

static void rm126x_uart_flush_rx(USART_TypeDef* uart) {
    while ((uart->ISR & USART_ISR_RXNE) != 0UL) {
        (void)uart->RDR;
    }
    rm126x_clear_uart_errors(uart);
}

static RM126xResult rm126x_uart_write_byte(USART_TypeDef* uart, uint8_t value,
                                           uint32_t timeout_ms) {
    uint32_t start_tick = HAL_GetTick();

    while ((uart->ISR & USART_ISR_TXE) == 0UL) {
        if (rm126x_elapsed_since(start_tick) >= timeout_ms) {
            return RM126X_RESULT_TIMEOUT;
        }
    }

    uart->TDR = (uint32_t)value;
    return RM126X_RESULT_OK;
}

static RM126xResult rm126x_uart_wait_tc(USART_TypeDef* uart, uint32_t timeout_ms) {
    uint32_t start_tick = HAL_GetTick();

    while ((uart->ISR & USART_ISR_TC) == 0UL) {
        if (rm126x_elapsed_since(start_tick) >= timeout_ms) {
            return RM126X_RESULT_TIMEOUT;
        }
    }

    return RM126X_RESULT_OK;
}

static RM126xResult rm126x_uart_write(RM126xHandle* handle, const uint8_t* data,
                                      size_t length, uint32_t timeout_ms) {
    size_t index;

    if ((handle == NULL) || (data == NULL)) {
        return RM126X_RESULT_INVALID_ARG;
    }

    if (!handle->uart_ready) {
        return RM126X_RESULT_TIMEOUT;
    }

    for (index = 0U; index < length; ++index) {
        RM126xResult result = rm126x_uart_write_byte(
            handle->config.uart.instance, data[index], timeout_ms);
        if (result != RM126X_RESULT_OK) {
            return result;
        }
    }

    return rm126x_uart_wait_tc(handle->config.uart.instance, timeout_ms);
}

static bool rm126x_uart_read_byte(RM126xHandle* handle, uint8_t* value,
                                  uint32_t timeout_ms) {
    USART_TypeDef* uart = handle->config.uart.instance;
    uint32_t start_tick = HAL_GetTick();

    while (rm126x_elapsed_since(start_tick) < timeout_ms) {
        uint32_t isr = uart->ISR;

        if ((isr & USART_ISR_RXNE) != 0UL) {
            *value = (uint8_t)(uart->RDR & 0xFFU);
            return true;
        }

        if ((isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) != 0UL) {
            rm126x_clear_uart_errors(uart);
        }
    }

    return false;
}

static void rm126x_trim_line(char* line) {
    size_t start = 0U;
    size_t end;
    size_t length;

    if (line == NULL) {
        return;
    }

    length = strlen(line);
    while ((start < length) && isspace((unsigned char)line[start])) {
        start++;
    }

    end = length;
    while ((end > start) && isspace((unsigned char)line[end - 1U])) {
        end--;
    }

    if (start > 0U) {
        memmove(line, line + start, end - start);
    }
    line[end - start] = '\0';
}

static RM126xAsyncEventType rm126x_update_async_event(RM126xHandle* handle,
                                                      const char* line) {
    RM126xAsyncEventType type = RM126X_ASYNC_NONE;

    if ((handle == NULL) || (line == NULL)) {
        return RM126X_ASYNC_NONE;
    }

    if (strcmp(line, "WAKE") == 0) {
        type = RM126X_ASYNC_WAKE;
    } else if (strncmp(line, "CLASS", 5U) == 0) {
        type = RM126X_ASYNC_CLASS;
    } else if (strncmp(line, "JOIN", 4U) == 0) {
        type = RM126X_ASYNC_JOIN;
        if (strstr(line, "OK") != NULL) {
            handle->is_connected = true;
            handle->last_app_state = RM126X_APP_STATE_JOINED;
            handle->last_connection_status = RM126X_CONNECTION_STATUS_CONNECTED;
        } else if (strstr(line, "FAIL") != NULL) {
            handle->is_connected = false;
            handle->last_app_state = RM126X_APP_STATE_DISCONNECTED;
            handle->last_connection_status = RM126X_CONNECTION_STATUS_NOT_CONNECTED;
        }
    } else if (strncmp(line, "TX", 2U) == 0) {
        type = RM126X_ASYNC_TX;
        handle->is_connected = true;
        handle->last_app_state = RM126X_APP_STATE_TX;
        handle->last_connection_status = RM126X_CONNECTION_STATUS_CONNECTED;
    } else if (strncmp(line, "RX", 2U) == 0) {
        type = RM126X_ASYNC_RX;
        handle->is_connected = true;
        handle->last_app_state = RM126X_APP_STATE_RX;
        handle->last_connection_status = RM126X_CONNECTION_STATUS_CONNECTED;
    } else if (strncmp(line, "LC", 2U) == 0) {
        type = RM126X_ASYNC_LINK_CHECK;
    } else if (strncmp(line, "TIME", 4U) == 0) {
        type = RM126X_ASYNC_TIME;
    } else if (strncmp(line, "SUP", 3U) == 0) {
        type = RM126X_ASYNC_SUPPLY;
    }

    if (type != RM126X_ASYNC_NONE) {
        handle->last_async_event.type = type;
        strncpy(handle->last_async_event.line, line,
                sizeof(handle->last_async_event.line) - 1U);
        handle->last_async_event.line[sizeof(handle->last_async_event.line) - 1U] =
            '\0';
    }

    return type;
}

static void rm126x_store_payload(RM126xResponseState* state, const char* line) {
    if ((state == NULL) || (line == NULL) || state->has_payload) {
        return;
    }

    strncpy(state->payload, line, sizeof(state->payload) - 1U);
    state->payload[sizeof(state->payload) - 1U] = '\0';
    state->has_payload = true;
}

static void rm126x_process_line(RM126xHandle* handle, RM126xResponseState* state,
                                const char* line) {
    RM126xAsyncEventType async_type;
    char* end_ptr = NULL;

    if ((handle == NULL) || (state == NULL) || (line == NULL) || (line[0] == '\0')) {
        return;
    }

    async_type = rm126x_update_async_event(handle, line);
    if (async_type == RM126X_ASYNC_WAKE) {
        state->saw_wake = true;
        return;
    }

    if (async_type != RM126X_ASYNC_NONE) {
        return;
    }

    if (strcmp(line, "OK") == 0) {
        state->saw_ok = true;
        return;
    }

    if (strncmp(line, "ERROR", 5U) == 0) {
        state->saw_error = true;
        if (line[5] != '\0') {
            unsigned long parsed = strtoul(line + 5, &end_ptr, 10);
            if ((end_ptr != (line + 5)) && (parsed <= UINT32_MAX)) {
                state->error_code = (uint32_t)parsed;
            }
        }
        return;
    }

    rm126x_store_payload(state, line);
}

static RM126xResult rm126x_read_response(RM126xHandle* handle,
                                         RM126xResponseState* state,
                                         uint32_t timeout_ms) {
    char line_buffer[RM126X_MAX_RESPONSE_LINE_LENGTH];
    size_t line_length = 0U;
    uint32_t start_tick = HAL_GetTick();

    if ((handle == NULL) || (state == NULL)) {
        return RM126X_RESULT_INVALID_ARG;
    }

    memset(state, 0, sizeof(*state));
    line_buffer[0] = '\0';

    while (rm126x_elapsed_since(start_tick) < timeout_ms) {
        uint8_t byte = 0U;

        if (!rm126x_uart_read_byte(handle, &byte,
                                   handle->config.rx_poll_timeout_ms)) {
            continue;
        }

        if ((byte == 0xFFU) || (byte == 0x02U)) {
            continue;
        }

        if ((byte == '\r') || (byte == '\n')) {
            if (line_length > 0U) {
                line_buffer[line_length] = '\0';
                rm126x_trim_line(line_buffer);
                rm126x_process_line(handle, state, line_buffer);
                line_length = 0U;

                if (state->saw_ok || state->saw_error) {
                    return state->saw_error ? RM126X_RESULT_MODULE_ERROR
                                            : RM126X_RESULT_OK;
                }
            }
            continue;
        }

        if ((byte >= 0x20U) && (line_length < (sizeof(line_buffer) - 1U))) {
            line_buffer[line_length] = (char)byte;
            line_length++;
        }
    }

    if (line_length > 0U) {
        line_buffer[line_length] = '\0';
        rm126x_trim_line(line_buffer);
        rm126x_process_line(handle, state, line_buffer);
        if (state->saw_ok || state->saw_error) {
            return state->saw_error ? RM126X_RESULT_MODULE_ERROR
                                    : RM126X_RESULT_OK;
        }
    }

    return RM126X_RESULT_TIMEOUT;
}

static RM126xResult rm126x_send_line(RM126xHandle* handle, const char* command) {
    static const uint8_t line_end[] = {'\r'};
    RM126xResult result;

    if ((handle == NULL) || (command == NULL)) {
        return RM126X_RESULT_INVALID_ARG;
    }

    result = rm126x_uart_write(handle, (const uint8_t*)command, strlen(command),
                               handle->config.command_timeout_ms);
    if (result != RM126X_RESULT_OK) {
        return result;
    }

    return rm126x_uart_write(handle, line_end, sizeof(line_end),
                             handle->config.command_timeout_ms);
}

static bool rm126x_append_char(char* buffer, size_t buffer_size, size_t* offset,
                               char value) {
    if ((buffer == NULL) || (offset == NULL) || (*offset >= (buffer_size - 1U))) {
        return false;
    }

    buffer[*offset] = value;
    (*offset)++;
    buffer[*offset] = '\0';
    return true;
}

static bool rm126x_append_cstr(char* buffer, size_t buffer_size, size_t* offset,
                               const char* value) {
    if (value == NULL) {
        return false;
    }

    while (*value != '\0') {
        if (!rm126x_append_char(buffer, buffer_size, offset, *value)) {
            return false;
        }
        value++;
    }

    return true;
}

static bool rm126x_append_u32(char* buffer, size_t buffer_size, size_t* offset,
                              uint32_t value) {
    char digits[10];
    size_t digit_count = 0U;

    do {
        digits[digit_count] = (char)('0' + (value % 10U));
        digit_count++;
        value /= 10U;
    } while ((value != 0U) && (digit_count < sizeof(digits)));

    while (digit_count > 0U) {
        digit_count--;
        if (!rm126x_append_char(buffer, buffer_size, offset, digits[digit_count])) {
            return false;
        }
    }

    return true;
}

static bool rm126x_append_i32(char* buffer, size_t buffer_size, size_t* offset,
                              int32_t value) {
    uint32_t magnitude;

    if (value < 0) {
        if (!rm126x_append_char(buffer, buffer_size, offset, '-')) {
            return false;
        }
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }

    return rm126x_append_u32(buffer, buffer_size, offset, magnitude);
}

static RM126xResult rm126x_execute_command(RM126xHandle* handle, char* response,
                                           size_t response_size,
                                           uint32_t timeout_ms,
                                           const char* command) {
    RM126xResponseState response_state;
    RM126xResult result;

    if ((handle == NULL) || (command == NULL)) {
        return RM126X_RESULT_INVALID_ARG;
    }

    if (!handle->uart_ready) {
        return RM126X_RESULT_TIMEOUT;
    }

    rm126x_reset_last_status(handle);
    rm126x_uart_flush_rx(handle->config.uart.instance);

    result = rm126x_send_line(handle, command);
    if (result != RM126X_RESULT_OK) {
        handle->last_result = result;
        return result;
    }

    result = rm126x_read_response(handle, &response_state, timeout_ms);
    handle->last_result = result;

    if ((response != NULL) && (response_size > 0U)) {
        response[0] = '\0';
        if (response_state.has_payload) {
            strncpy(response, response_state.payload, response_size - 1U);
            response[response_size - 1U] = '\0';
        }
    }

    if (result == RM126X_RESULT_MODULE_ERROR) {
        handle->last_error_code = response_state.error_code;
        handle->last_error = (RM126xAtErrorCode)response_state.error_code;
    }

    return result;
}

static RM126xResult rm126x_build_query_int_command(char* command,
                                                   size_t command_size,
                                                   char family,
                                                   uint16_t register_id) {
    size_t offset = 0U;

    if ((family != 'I') && (family != 'S')) {
        return RM126X_RESULT_INVALID_ARG;
    }

    command[0] = '\0';
    if (!rm126x_append_cstr(command, command_size, &offset, "AT") ||
        !rm126x_append_char(command, command_size, &offset, family) ||
        !rm126x_append_char(command, command_size, &offset, ' ') ||
        !rm126x_append_u32(command, command_size, &offset, register_id)) {
        return RM126X_RESULT_BUFFER_TOO_SMALL;
    }

    if ((family != 'I') &&
        !rm126x_append_char(command, command_size, &offset, '?')) {
        return RM126X_RESULT_BUFFER_TOO_SMALL;
    }

    return RM126X_RESULT_OK;
}

static RM126xResult rm126x_build_set_numeric_command(char* command,
                                                     size_t command_size,
                                                     uint16_t register_id,
                                                     int32_t value) {
    size_t offset = 0U;

    command[0] = '\0';
    if (!rm126x_append_cstr(command, command_size, &offset, "ATS ") ||
        !rm126x_append_u32(command, command_size, &offset, register_id) ||
        !rm126x_append_char(command, command_size, &offset, '=') ||
        !rm126x_append_i32(command, command_size, &offset, value)) {
        return RM126X_RESULT_BUFFER_TOO_SMALL;
    }

    return RM126X_RESULT_OK;
}

static RM126xResult rm126x_build_send_hex_command(char* command,
                                                  size_t command_size,
                                                  const char* hex_data) {
    size_t offset = 0U;

    if (hex_data == NULL) {
        return RM126X_RESULT_INVALID_ARG;
    }

    command[0] = '\0';
    if (!rm126x_append_cstr(command, command_size, &offset, "AT+SEND \"") ||
        !rm126x_append_cstr(command, command_size, &offset, hex_data) ||
        !rm126x_append_char(command, command_size, &offset, '"')) {
        return RM126X_RESULT_BUFFER_TOO_SMALL;
    }

    return RM126X_RESULT_OK;
}

static RM126xResult rm126x_query_int(RM126xHandle* handle, char family,
                                     uint16_t register_id, int32_t* value) {
    char command[RM126X_MAX_COMMAND_LENGTH];
    char response[RM126X_MAX_RESPONSE_LINE_LENGTH];
    char* end_ptr = NULL;
    long parsed;
    RM126xResult result;

    if ((handle == NULL) || (value == NULL)) {
        return RM126X_RESULT_INVALID_ARG;
    }

    result = rm126x_build_query_int_command(command, sizeof(command), family,
                                            register_id);
    if (result != RM126X_RESULT_OK) {
        return result;
    }

    result = rm126x_execute_command(handle, response, sizeof(response),
                                    handle->config.command_timeout_ms, command);
    if (result != RM126X_RESULT_OK) {
        return result;
    }

    parsed = strtol(response, &end_ptr, 10);
    if ((end_ptr == response) || (*end_ptr != '\0')) {
        return RM126X_RESULT_PARSE_ERROR;
    }

    *value = (int32_t)parsed;
    return RM126X_RESULT_OK;
}

static RM126xResult rm126x_validate_hex_string(RM126xHandle* handle,
                                               const char* hex_data) {
    size_t index;
    size_t length;

    if ((handle == NULL) || (hex_data == NULL)) {
        return RM126X_RESULT_INVALID_ARG;
    }

    length = strlen(hex_data);
    if (((length % 2U) != 0U) || (length == 0U)) {
        handle->last_result = RM126X_RESULT_INVALID_ARG;
        handle->last_error = RM126X_AT_ERROR_HEX_STRING_INVALID;
        handle->last_error_code = (uint32_t)RM126X_AT_ERROR_HEX_STRING_INVALID;
        return RM126X_RESULT_INVALID_ARG;
    }

    for (index = 0U; index < length; ++index) {
        if (!isxdigit((unsigned char)hex_data[index])) {
            handle->last_result = RM126X_RESULT_INVALID_ARG;
            handle->last_error = RM126X_AT_ERROR_HEX_STRING_INVALID;
            handle->last_error_code =
                (uint32_t)RM126X_AT_ERROR_HEX_STRING_INVALID;
            return RM126X_RESULT_INVALID_ARG;
        }
    }

    return RM126X_RESULT_OK;
}

static void rm126x_encode_bytes_to_hex(const uint8_t* data, size_t length,
                                       char* hex_out) {
    static const char hex_digits[] = "0123456789ABCDEF";
    size_t index;

    for (index = 0U; index < length; ++index) {
        hex_out[index * 2U] = hex_digits[(data[index] >> 4U) & 0x0FU];
        hex_out[(index * 2U) + 1U] = hex_digits[data[index] & 0x0FU];
    }
    hex_out[length * 2U] = '\0';
}

void RM126x_Init(RM126xHandle* handle, const RM126xConfig* config) {
    if ((handle == NULL) || (config == NULL)) {
        return;
    }

    memset(handle, 0, sizeof(*handle));
    handle->config = *config;

    if (handle->config.uart.baud_rate == 0UL) {
        handle->config.uart.baud_rate = RM126X_DEFAULT_BAUD_RATE;
    }
    if (handle->config.wake_timeout_ms == 0UL) {
        handle->config.wake_timeout_ms = RM126X_DEFAULT_WAKE_TIMEOUT_MS;
    }
    if (handle->config.command_timeout_ms == 0UL) {
        handle->config.command_timeout_ms = RM126X_DEFAULT_COMMAND_TIMEOUT_MS;
    }
    if (handle->config.rx_poll_timeout_ms == 0UL) {
        handle->config.rx_poll_timeout_ms = RM126X_DEFAULT_RX_POLL_TIMEOUT_MS;
    }

    handle->current_fport = RM126X_DEFAULT_FPORT;
    handle->last_app_state = RM126X_APP_STATE_UNKNOWN;
    handle->last_connection_status = RM126X_CONNECTION_STATUS_UNKNOWN;
    handle->last_async_event.type = RM126X_ASYNC_NONE;
    rm126x_reset_last_status(handle);
    rm126x_update_pending_fport9(handle);
}

RM126xResult RM126x_HostUartResume(RM126xHandle* handle) {
    RM126xResult result;

    if (handle == NULL) {
        return RM126X_RESULT_INVALID_ARG;
    }

    result = rm126x_configure_uart_clock(handle);
    if (result != RM126X_RESULT_OK) {
        handle->last_result = result;
        return result;
    }

    result = rm126x_enable_uart_clock(handle);
    if (result != RM126X_RESULT_OK) {
        handle->last_result = result;
        return result;
    }

    rm126x_configure_uart_gpio(handle);
    result = rm126x_uart_enable(handle);
    handle->last_result = result;
    return result;
}

RM126xResult RM126x_HostUartSuspend(RM126xHandle* handle) {
    if ((handle == NULL) || (handle->config.uart.instance == NULL)) {
        return RM126X_RESULT_INVALID_ARG;
    }

    if (handle->uart_ready) {
        (void)rm126x_uart_wait_tc(handle->config.uart.instance,
                                  RM126X_UART_WAIT_READY_TIMEOUT_MS);
        CLEAR_BIT(handle->config.uart.instance->CR1, USART_CR1_UE);
        handle->uart_ready = false;
    }

    rm126x_disable_uart_clock(handle);
    handle->last_result = RM126X_RESULT_OK;
    return RM126X_RESULT_OK;
}

RM126xResult RM126x_WakeAndPing(RM126xHandle* handle) {
    RM126xResponseState response_state;
    RM126xResult result;

    if (handle == NULL) {
        return RM126X_RESULT_INVALID_ARG;
    }

    if (!handle->uart_ready) {
        result = RM126x_HostUartResume(handle);
        if (result != RM126X_RESULT_OK) {
            return result;
        }
    }

    rm126x_reset_last_status(handle);
    rm126x_uart_flush_rx(handle->config.uart.instance);

    result = rm126x_send_line(handle, "AT");
    if (result != RM126X_RESULT_OK) {
        handle->last_result = result;
        return result;
    }

    result = rm126x_read_response(handle, &response_state,
                                  handle->config.wake_timeout_ms);
    if (result == RM126X_RESULT_OK) {
        handle->last_result = RM126X_RESULT_OK;
        return RM126X_RESULT_OK;
    }

    if ((result == RM126X_RESULT_TIMEOUT) && response_state.saw_wake) {
        result = rm126x_execute_command(handle, NULL, 0U,
                                        handle->config.command_timeout_ms,
                                        "AT");
        handle->last_result = result;
        return result;
    }

    handle->last_result = result;
    return result;
}

RM126xResult RM126x_At(RM126xHandle* handle) {
    return rm126x_execute_command(handle, NULL, 0U,
                                  handle->config.command_timeout_ms, "AT");
}

RM126xResult RM126x_Reset(RM126xHandle* handle) {
    return rm126x_execute_command(handle, NULL, 0U,
                                  handle->config.command_timeout_ms, "ATZ");
}

static RM126xResult RM126x_GetInfoInt(RM126xHandle* handle, uint16_t info_id,
                                      int32_t* value) {
    return rm126x_query_int(handle, 'I', info_id, value);
}

static RM126xResult RM126x_SetNumericRegister(RM126xHandle* handle,
                                              uint16_t register_id,
                                              int32_t value) {
    char command[RM126X_MAX_COMMAND_LENGTH];
    RM126xResult result = rm126x_build_set_numeric_command(
        command, sizeof(command), register_id, value);

    if (result != RM126X_RESULT_OK) {
        return result;
    }

    return rm126x_execute_command(handle, NULL, 0U,
                                  handle->config.command_timeout_ms, command);
}

static RM126xResult RM126x_GetNumericRegister(RM126xHandle* handle,
                                              uint16_t register_id,
                                              int32_t* value) {
    return rm126x_query_int(handle, 'S', register_id, value);
}

RM126xResult RM126x_Join(RM126xHandle* handle) {
    RM126xResult result = rm126x_execute_command(handle, NULL, 0U,
                                                 handle->config.command_timeout_ms,
                                                 "AT+JOIN");
    if (result == RM126X_RESULT_OK) {
        handle->is_connected = false;
        handle->last_app_state = RM126X_APP_STATE_JOINING;
        return result;
    }

    if (result == RM126X_RESULT_MODULE_ERROR) {
        if (handle->last_error == RM126X_AT_ERROR_LORA_ALREADY_JOINED) {
            handle->is_connected = true;
            handle->last_connection_status = RM126X_CONNECTION_STATUS_CONNECTED;
            handle->last_app_state = RM126X_APP_STATE_JOINED;
        } else if (handle->last_error == RM126X_AT_ERROR_LORA_NOT_JOINED) {
            handle->is_connected = false;
            handle->last_connection_status =
                RM126X_CONNECTION_STATUS_NOT_CONNECTED;
            handle->last_app_state = RM126X_APP_STATE_DISCONNECTED;
        } else if (handle->last_error ==
                   RM126X_AT_ERROR_COMMAND_CANNOT_BE_PROCESSED) {
            (void)RM126x_RefreshState(handle);
        }
    }

    return result;
}

static RM126xResult RM126x_SendHex(RM126xHandle* handle, const char* hex_data) {
    char command[RM126X_MAX_COMMAND_LENGTH];
    RM126xResult result;

    result = rm126x_validate_hex_string(handle, hex_data);
    if (result != RM126X_RESULT_OK) {
        return result;
    }

    result = rm126x_build_send_hex_command(command, sizeof(command), hex_data);
    if (result != RM126X_RESULT_OK) {
        return result;
    }

    result = rm126x_execute_command(handle, NULL, 0U,
                                    handle->config.command_timeout_ms, command);
    if (result == RM126X_RESULT_OK) {
        handle->accepted_uplink_count++;
        rm126x_update_pending_fport9(handle);
        return result;
    }

    if (result == RM126X_RESULT_MODULE_ERROR) {
        if (handle->last_error == RM126X_AT_ERROR_LORA_NOT_JOINED) {
            handle->is_connected = false;
            handle->last_connection_status =
                RM126X_CONNECTION_STATUS_NOT_CONNECTED;
            handle->last_app_state = RM126X_APP_STATE_DISCONNECTED;
        } else if (handle->last_error ==
                   RM126X_AT_ERROR_LORA_ALREADY_JOINED) {
            handle->is_connected = true;
            handle->last_connection_status =
                RM126X_CONNECTION_STATUS_CONNECTED;
            handle->last_app_state = RM126X_APP_STATE_JOINED;
        } else if (handle->last_error ==
                   RM126X_AT_ERROR_COMMAND_CANNOT_BE_PROCESSED) {
            (void)RM126x_RefreshState(handle);
        }
    }

    return result;
}

RM126xResult RM126x_SendBytes(RM126xHandle* handle, const uint8_t* data,
                              size_t length) {
    char hex_buffer[(RM126X_MAX_SEND_PAYLOAD_BYTES * 2U) + 1U];

    if ((handle == NULL) || (data == NULL) || (length == 0U)) {
        return RM126X_RESULT_INVALID_ARG;
    }

    if (length > RM126X_MAX_SEND_PAYLOAD_BYTES) {
        return RM126X_RESULT_BUFFER_TOO_SMALL;
    }

    rm126x_encode_bytes_to_hex(data, length, hex_buffer);
    return RM126x_SendHex(handle, hex_buffer);
}

RM126xResult RM126x_ReadTemperature(RM126xHandle* handle, char* response,
                                    size_t response_size) {
    return rm126x_execute_command(handle, response, response_size,
                                  handle->config.command_timeout_ms, "AT+TEMP");
}

RM126xResult RM126x_Sleep(RM126xHandle* handle) {
    return rm126x_execute_command(handle, NULL, 0U,
                                  handle->config.command_timeout_ms,
                                  "AT+SLEEP");
}

static RM126xResult RM126x_ReadApplicationState(RM126xHandle* handle,
                                                RM126xApplicationState* state) {
    int32_t value = 0;
    RM126xResult result;

    if ((handle == NULL) || (state == NULL)) {
        return RM126X_RESULT_INVALID_ARG;
    }

    result = RM126x_GetInfoInt(handle, RM126X_INFO_ID_APPLICATION_STATE, &value);
    if (result != RM126X_RESULT_OK) {
        return result;
    }

    if ((value < (int32_t)RM126X_APP_STATE_IDLE) ||
        (value > (int32_t)RM126X_APP_STATE_DISCONNECTED)) {
        return RM126X_RESULT_PARSE_ERROR;
    }

    *state = (RM126xApplicationState)value;
    handle->last_app_state = *state;
    return RM126X_RESULT_OK;
}

static RM126xResult RM126x_ReadConnectionStatus(RM126xHandle* handle,
                                                RM126xConnectionStatus* status) {
    int32_t value = 0;
    RM126xResult result;

    if ((handle == NULL) || (status == NULL)) {
        return RM126X_RESULT_INVALID_ARG;
    }

    result = RM126x_GetInfoInt(handle, RM126X_INFO_ID_CONNECTION_STATUS, &value);
    if (result != RM126X_RESULT_OK) {
        return result;
    }

    if ((value < (int32_t)RM126X_CONNECTION_STATUS_NOT_CONNECTED) ||
        (value > (int32_t)RM126X_CONNECTION_STATUS_CONNECTED)) {
        return RM126X_RESULT_PARSE_ERROR;
    }

    *status = (RM126xConnectionStatus)value;
    handle->last_connection_status = *status;
    handle->is_connected = (*status == RM126X_CONNECTION_STATUS_CONNECTED);
    return RM126X_RESULT_OK;
}

static RM126xResult RM126x_ReadPort(RM126xHandle* handle, uint8_t* fport) {
    int32_t value = 0;
    RM126xResult result;

    if ((handle == NULL) || (fport == NULL)) {
        return RM126X_RESULT_INVALID_ARG;
    }

    result = RM126x_GetNumericRegister(handle, RM126X_SREG_ID_APPLICATION_PORT,
                                       &value);
    if (result != RM126X_RESULT_OK) {
        return result;
    }

    if ((value < 1) || (value > 198)) {
        return RM126X_RESULT_PARSE_ERROR;
    }

    *fport = (uint8_t)value;
    handle->current_fport = *fport;
    return RM126X_RESULT_OK;
}

RM126xResult RM126x_SetFPort(RM126xHandle* handle, uint8_t fport) {
    RM126xResult result;

    if ((handle == NULL) || (fport < 1U) || (fport > 198U)) {
        return RM126X_RESULT_INVALID_ARG;
    }

    result = RM126x_SetNumericRegister(handle, RM126X_SREG_ID_APPLICATION_PORT,
                                       (int32_t)fport);
    if (result == RM126X_RESULT_OK) {
        handle->current_fport = fport;
    } else if ((result == RM126X_RESULT_MODULE_ERROR) &&
               (handle->last_error ==
                RM126X_AT_ERROR_COMMAND_CANNOT_BE_PROCESSED)) {
        (void)RM126x_RefreshState(handle);
    }

    return result;
}

static RM126xResult RM126x_RefreshState(RM126xHandle* handle) {
    RM126xApplicationState app_state = RM126X_APP_STATE_UNKNOWN;
    RM126xConnectionStatus connection_status =
        RM126X_CONNECTION_STATUS_UNKNOWN;
    uint8_t current_port;
    RM126xResult result;
    RM126xResult app_result;
    RM126xResult connection_result;

    if (handle == NULL) {
        return RM126X_RESULT_INVALID_ARG;
    }

    current_port = handle->current_fport;

    result = RM126x_WakeAndPing(handle);
    if (result != RM126X_RESULT_OK) {
        return result;
    }

    app_result = RM126x_ReadApplicationState(handle, &app_state);
    connection_result = RM126x_ReadConnectionStatus(handle, &connection_status);
    (void)RM126x_ReadPort(handle, &current_port);

    if (connection_result == RM126X_RESULT_OK) {
        handle->is_connected =
            (connection_status == RM126X_CONNECTION_STATUS_CONNECTED);
    } else if (app_result == RM126X_RESULT_OK) {
        handle->is_connected = (app_state == RM126X_APP_STATE_JOINED) ||
                               (app_state == RM126X_APP_STATE_TX) ||
                               (app_state == RM126X_APP_STATE_RX);
    }

    if ((connection_result == RM126X_RESULT_OK) || (app_result == RM126X_RESULT_OK)) {
        handle->last_result = RM126X_RESULT_OK;
        return RM126X_RESULT_OK;
    }

    return app_result;
}

bool RM126x_IsConnected(RM126xHandle* handle) {
    RM126xConnectionStatus status = RM126X_CONNECTION_STATUS_UNKNOWN;

    if (handle == NULL) {
        return false;
    }

    if (RM126x_WakeAndPing(handle) != RM126X_RESULT_OK) {
        handle->is_connected = false;
        return false;
    }

    if (RM126x_ReadConnectionStatus(handle, &status) != RM126X_RESULT_OK) {
        handle->is_connected = false;
        return false;
    }

    return (status == RM126X_CONNECTION_STATUS_CONNECTED);
}

RM126xAtErrorCode RM126x_GetLastAtError(const RM126xHandle* handle) {
    return (handle != NULL) ? handle->last_error : RM126X_AT_ERROR_NONE;
}

const RM126xAsyncEvent* RM126x_GetLastAsyncEvent(const RM126xHandle* handle) {
    return (handle != NULL) ? &handle->last_async_event : NULL;
}

bool RM126x_ShouldSendDiagnosticPort9(const RM126xHandle* handle) {
    return (handle != NULL) ? handle->pending_fport9 : false;
}
