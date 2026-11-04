#ifndef MIDI_UART_H
#define MIDI_UART_H

#include "pico/stdlib.h"
#include "hardware/uart.h"

// ==========================================
// UART Configuration (MIDI Controller Pico <-> DSP Pico 2)
// ==========================================
#define MIDI_UART_ID        uart0
#define MIDI_UART_BAUD_RATE 115200
#define MIDI_UART_TX_PIN    0   // GP0 (UART0 TX -> MIDI Controller GP1 RX)
#define MIDI_UART_RX_PIN    1   // GP1 (UART0 RX <- MIDI Controller GP0 TX)

#define MIDI_CC_START       20  // CC number for parameter index 0 (Param i = CC 20 + i)
#define MIDI_CC_SYNC_REQ    127 // CC 127 (value 127) requests full parameter list sync

// Non-blocking UART TX ring buffer to avoid stalling the real-time I2S DMA loop
#define UART_TX_BUF_SIZE    512
static uint8_t  uart_tx_buf[UART_TX_BUF_SIZE];
static uint16_t uart_tx_head = 0;
static uint16_t uart_tx_tail = 0;

// State for incremental parameter sync (sends 1 parameter per audio block on CC 127)
static RNBO::ParameterIndex sync_send_index = 0xFFFF;

static inline void uart_tx_enqueue(uint8_t b) {
    uint16_t next = (uart_tx_head + 1) & (UART_TX_BUF_SIZE - 1);
    if (next != uart_tx_tail) {
        uart_tx_buf[uart_tx_head] = b;
        uart_tx_head = next;
    }
}

static inline void drain_uart_tx_fifo(void) {
    while (uart_tx_tail != uart_tx_head && uart_is_writable(MIDI_UART_ID)) {
        uart_get_hw(MIDI_UART_ID)->dr = uart_tx_buf[uart_tx_tail];
        uart_tx_tail = (uart_tx_tail + 1) & (UART_TX_BUF_SIZE - 1);
    }
}

// Fast lightweight string/number formatting helpers (avoids slow printf/snprintf in audio loop)
static int append_str(char* dst, int pos, int max_len, const char* s) {
    while (*s && pos < max_len - 1) {
        dst[pos++] = *s++;
    }
    dst[pos] = '\0';
    return pos;
}

static int append_int(char* dst, int pos, int max_len, int val) {
    if (val < 0) {
        if (pos < max_len - 1) dst[pos++] = '-';
        val = -val;
    }
    char tmp[12];
    int n = 0;
    do {
        tmp[n++] = (char)('0' + (val % 10));
        val /= 10;
    } while (val > 0 && n < 12);
    while (n > 0 && pos < max_len - 1) {
        dst[pos++] = tmp[--n];
    }
    dst[pos] = '\0';
    return pos;
}

static int append_fixed2(char* dst, int pos, int max_len, float val) {
    if (val < 0.0f) {
        if (pos < max_len - 1) dst[pos++] = '-';
        val = -val;
    }
    int scaled = (int)(val * 100.0f + 0.5f);
    int int_part = scaled / 100;
    int frac_part = scaled % 100;
    pos = append_int(dst, pos, max_len, int_part);
    if (pos < max_len - 1) dst[pos++] = '.';
    if (pos < max_len - 1) dst[pos++] = (char)('0' + (frac_part / 10));
    if (pos < max_len - 1) dst[pos++] = (char)('0' + (frac_part % 10));
    dst[pos] = '\0';
    return pos;
}

/**
 * Wrap an ASCII payload string in a MIDI SysEx frame (0xF0, 0x7D, <ASCII>, 0xF7)
 * and push it to the non-blocking UART TX ring buffer.
 * Modeled after CustomAudioProcessor::createMessage in JUCE_Pico_Bridge.
 */
static void createMessage(const char* dataStr) {
    uart_tx_enqueue(0xF0);
    uart_tx_enqueue(0x7D); // 0x7D: Educational / Prototype Manufacturer ID
    for (size_t i = 0; dataStr[i] != '\0'; i++) {
        uart_tx_enqueue((uint8_t)dataStr[i] & 0x7F);
    }
    uart_tx_enqueue(0xF7);
}

/**
 * Format parameter value as string for the MIDI controller LCD.
 * If the parameter defines enum/discrete steps, sends the step name or integer;
 * otherwise sends a 2-decimal-place number with optional unit.
 */
template <typename RNBOT>
static int append_param_display_value(RNBOT& rnboObj, RNBO::ParameterIndex index,
                                      const RNBO::ParameterInfo& info, char* buf, int pos, int max_len) {
    RNBO::ParameterValue val = rnboObj.getParameterValue(index);
    if (info.enumValues && info.steps > 0) {
        int step = (int)(val + 0.5f);
        if (step >= 0 && step < info.steps && info.enumValues[step]) {
            return append_str(buf, pos, max_len, info.enumValues[step]);
        }
    }
    if (info.steps > 0) {
        return append_int(buf, pos, max_len, (int)(val + 0.5f));
    }
    pos = append_fixed2(buf, pos, max_len, (float)val);
    if (info.unit && info.unit[0] != '\0') {
        pos = append_str(buf, pos, max_len, " ");
        pos = append_str(buf, pos, max_len, info.unit);
    }
    return pos;
}

/**
 * Send initial parameter configuration ("CONF,<cc>,<paramId>,<valStr>,<range>,<midiVal>")
 */
template <typename RNBOT>
static void send_parameter_config(RNBOT& rnboObj, RNBO::ParameterIndex index) {
    RNBO::ParameterInfo info;
    rnboObj.getParameterInfo(index, &info);

    int ccNumber = MIDI_CC_START + (int)index;
    const char* paramId = rnboObj.getParameterId(index);
    int range = (info.steps > 0) ? info.steps : 0;

    RNBO::ParameterValue normVal = rnboObj.getParameterNormalized(index);
    int midiVal = (int)(normVal * 127.0f + 0.5f);
    if (midiVal < 0)   midiVal = 0;
    if (midiVal > 127) midiVal = 127;

    char msg[128];
    int pos = 0;
    pos = append_str(msg, pos, sizeof(msg), "CONF,");
    pos = append_int(msg, pos, sizeof(msg), ccNumber);
    pos = append_str(msg, pos, sizeof(msg), ",");
    pos = append_str(msg, pos, sizeof(msg), paramId);
    pos = append_str(msg, pos, sizeof(msg), ",");
    pos = append_param_display_value(rnboObj, index, info, msg, pos, sizeof(msg));
    pos = append_str(msg, pos, sizeof(msg), ",");
    pos = append_int(msg, pos, sizeof(msg), range);
    pos = append_str(msg, pos, sizeof(msg), ",");
    pos = append_int(msg, pos, sizeof(msg), midiVal);

    createMessage(msg);
}

/**
 * Send parameter value update ("UPDT,<paramId>,<valStr>,<midiVal>")
 */
template <typename RNBOT>
static void send_parameter_update(RNBOT& rnboObj, RNBO::ParameterIndex index, int midiVal) {
    RNBO::ParameterInfo info;
    rnboObj.getParameterInfo(index, &info);

    const char* paramId = rnboObj.getParameterId(index);

    char msg[96];
    int pos = 0;
    pos = append_str(msg, pos, sizeof(msg), "UPDT,");
    pos = append_str(msg, pos, sizeof(msg), paramId);
    pos = append_str(msg, pos, sizeof(msg), ",");
    pos = append_param_display_value(rnboObj, index, info, msg, pos, sizeof(msg));
    pos = append_str(msg, pos, sizeof(msg), ",");
    pos = append_int(msg, pos, sizeof(msg), midiVal);

    createMessage(msg);
}

/**
 * Initialize UART0 for MIDI communication and transmit all initial parameter configurations
 * before the real-time I2S DMA loop starts.
 */
template <typename RNBOT>
static void init_midi_uart(RNBOT& rnboObj) {
    uart_init(MIDI_UART_ID, MIDI_UART_BAUD_RATE);
    gpio_set_function(MIDI_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(MIDI_UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_fifo_enabled(MIDI_UART_ID, true);

    // Clear any startup bytes sitting in the UART RX FIFO
    while (uart_is_readable(MIDI_UART_ID)) {
        (void)uart_getc(MIDI_UART_ID);
    }

    // Send all initial parameter CONF messages and wait for completion before audio DMA starts
    RNBO::ParameterIndex numParams = rnboObj.getNumParameters();
    for (RNBO::ParameterIndex i = 0; i < numParams; i++) {
        send_parameter_config(rnboObj, i);
        drain_uart_tx_fifo();
    }
    while (uart_tx_tail != uart_tx_head) {
        drain_uart_tx_fifo();
    }
    uart_tx_wait_blocking(MIDI_UART_ID);
}

/**
 * Non-blocking UART MIDI parser called once per audio block.
 * - Parses incoming 3-byte Control Change messages (0xB0, CC#, Value)
 * - Updates RNBO parameters via setParameterValue() and queues UPDT SysEx responses
 * - Streams CONF SysEx messages 1 parameter per block if CC 127 sync is requested
 */
template <typename RNBOT>
static void poll_midi_uart(RNBOT& rnboObj) {
    static uint8_t midi_status = 0;
    static uint8_t midi_data1  = 0;
    static uint8_t midi_state  = 0; // 0: wait status/data1, 1: wait data2

    // If a sync request (CC 127) is active, queue at most 1 parameter per audio block
    // only when the software TX buffer is empty.
    RNBO::ParameterIndex numParams = rnboObj.getNumParameters();
    if (sync_send_index < numParams && uart_tx_tail == uart_tx_head) {
        send_parameter_config(rnboObj, sync_send_index);
        sync_send_index++;
    }

    // Drain outgoing software TX ring buffer into hardware UART FIFO
    drain_uart_tx_fifo();

    // Process up to 4 incoming bytes per audio block (1 full MIDI CC message = 3 bytes)
    int max_rx_bytes = 4;
    while (max_rx_bytes-- > 0 && uart_is_readable(MIDI_UART_ID)) {
        uint8_t b = (uint8_t)uart_getc(MIDI_UART_ID);

        if (b & 0x80) {
            // Status byte received
            if (b >= 0xF8) {
                continue; // Ignore MIDI real-time messages
            }
            if ((b & 0xF0) == 0xB0) {
                midi_status = b;
                midi_state  = 0;
            } else {
                midi_status = 0;
                midi_state  = 0;
            }
        } else if (midi_status != 0) {
            // Data byte received (0x00 - 0x7F)
            if (midi_state == 0) {
                midi_data1 = b;
                midi_state = 1;
            } else if (midi_state == 1) {
                uint8_t cc  = midi_data1;
                uint8_t val = b;
                midi_state  = 0; // Running status ready for next CC pair

                if (cc == MIDI_CC_SYNC_REQ && val == 127) {
                    // Start incremental parameter sync (1 parameter per audio block)
                    sync_send_index = 0;
                } else if (cc >= MIDI_CC_START) {
                    RNBO::ParameterIndex index = (RNBO::ParameterIndex)(cc - MIDI_CC_START);
                    if (index < numParams) {
                        RNBO::ParameterValue normVal = (RNBO::ParameterValue)val / 127.0f;
                        RNBO::ParameterValue newValue = rnboObj.convertFromNormalizedParameterValue(index, normVal);
                        rnboObj.setParameterValue(index, newValue, RNBO::TimeNow);
                        send_parameter_update(rnboObj, index, (int)val);
                    }
                }
            }
        }
    }

    drain_uart_tx_fifo();
}

#endif // MIDI_UART_H
