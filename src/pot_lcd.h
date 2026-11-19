#ifndef POT_LCD_H
#define POT_LCD_H

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"

// ==========================================
// Potentiometer (ADC) Configuration
// ==========================================
#define OUTPUT_POT_GPIO     26  // GP26 = ADC0 (Post-RNBO Output Gain)
#define OUTPUT_POT_ADC_CH   0
#define INPUT_POT_GPIO      27  // GP27 = ADC1 (Pre-RNBO Input Gain)
#define INPUT_POT_ADC_CH    1

// Pre-RNBO gain: moderate boost to bring passive guitar pickup level into optimal DSP operating range
// without over-driving internal non-linearities (e.g. atan / clamp inside gen~)
#define MAX_INPUT_GAIN      2.0f
// Post-RNBO gain: primary volume boost after RNBO processing to compensate for
// internal reverb/allpass attenuation before the PCM5102A DAC limiter
#define MAX_OUTPUT_GAIN     2.0f

// ==========================================
// LCD1602 (I2C1) Configuration
// ==========================================
#define LCD_I2C_INST        i2c1
#define LCD_SDA_PIN         14  // GP14 (I2C1 SDA)
#define LCD_SCL_PIN         15  // GP15 (I2C1 SCL)
#define LCD_I2C_BAUD        100000
#define LCD_DEFAULT_ADDR    0x27

#define LCD_EN_BIT          0x04
#define LCD_BL_BIT          0x08
#define LCD_MODE_CMD        0x00
#define LCD_MODE_CHR        0x01
#define LCD_LINE1_ADDR      0x80
#define LCD_LINE2_ADDR      0xC0

// Shared state between Core 1 (Potentiometers + LCD) and Core 0 (Real-Time Audio DSP)
static volatile float shared_target_in_gain  = 0.0f;
static volatile float shared_target_out_gain = 0.0f;
static volatile float shared_pre_peak        = 0.0f;
static volatile float shared_post_peak       = 0.0f;

static uint8_t lcd_i2c_addr = LCD_DEFAULT_ADDR;

// ==========================================
// SRAM-Resident Helpers for Core 1
// (Executed 100% from RAM with zero Flash XIP contention and zero timer IRQs on Core 0)
// ==========================================
static void __not_in_flash_func(core1_wait_us)(uint32_t us) {
    uint32_t start = timer_hw->timerawl;
    while ((uint32_t)(timer_hw->timerawl - start) < us) {
        __asm volatile ("nop");
    }
}

static inline float __not_in_flash_func(core1_fast_sqrtf)(float x) {
    if (x <= 0.0f) return 0.0f;
    float res;
    __asm volatile ("vsqrt.f32 %0, %1" : "=t"(res) : "t"(x));
    return res;
}

static int __not_in_flash_func(read_pot_averaged)(uint adc_channel) {
    hw_write_masked(&adc_hw->cs, adc_channel << ADC_CS_AINSEL_LSB, ADC_CS_AINSEL_BITS);
    // Dummy read right after switching channels to prevent mux crosstalk
    hw_set_bits(&adc_hw->cs, ADC_CS_START_ONCE_BITS);
    while (!(adc_hw->cs & ADC_CS_READY_BITS)) {
        __asm volatile ("nop");
    }
    (void)adc_hw->result;

    uint32_t sum = 0;
    for (int i = 0; i < 8; i++) {
        hw_set_bits(&adc_hw->cs, ADC_CS_START_ONCE_BITS);
        while (!(adc_hw->cs & ADC_CS_READY_BITS)) {
            __asm volatile ("nop");
        }
        sum += adc_hw->result;
    }
    return (int)(sum >> 3); // 0 to 4095
}

/**
 * Read potentiometer ADC value and update target gain while ignoring small jitter (noise).
 * - Deadzone: Force mute (0.0f) when near minimum (< 16)
 * - Hysteresis: Only update target gain when the reading changes by more than +-12
 */
static float __not_in_flash_func(update_pot_target_gain)(uint adc_channel, float max_gain, int *last_adc, float current_target_gain) {
    int adc_val = read_pot_averaged(adc_channel);

    if (adc_val < 16) {
        *last_adc = adc_val;
        return 0.0f;
    }
    if (adc_val - *last_adc > 12 || *last_adc - adc_val > 12) {
        *last_adc = adc_val;
        return ((float)adc_val / 4095.0f) * max_gain;
    }
    return current_target_gain;
}

// ==========================================
// LCD1602 I2C Driver & Bar Graph Meter Functions (SRAM-Resident)
// ==========================================
static void __not_in_flash_func(lcd_i2c_send_nibble)(uint8_t nibble_with_mode) {
    i2c_hw_t *hw = i2c_get_hw(LCD_I2C_INST);
    uint8_t b0 = nibble_with_mode | LCD_EN_BIT | LCD_BL_BIT;
    uint8_t b1 = nibble_with_mode | LCD_BL_BIT;

    uint32_t start = timer_hw->timerawl;
    while (!(hw->status & I2C_IC_STATUS_TFNF_BITS)) {
        if ((uint32_t)(timer_hw->timerawl - start) > 2000) return;
    }
    hw->data_cmd = b0;

    while (!(hw->status & I2C_IC_STATUS_TFNF_BITS)) {
        if ((uint32_t)(timer_hw->timerawl - start) > 2000) return;
    }
    hw->data_cmd = b1 | I2C_IC_DATA_CMD_STOP_BITS;

    while ((hw->status & (I2C_IC_STATUS_TFE_BITS | I2C_IC_STATUS_ACTIVITY_BITS)) != I2C_IC_STATUS_TFE_BITS) {
        if ((uint32_t)(timer_hw->timerawl - start) > 2000) return;
    }
    core1_wait_us(50);
}

static void __not_in_flash_func(lcd_write_byte)(uint8_t bits, uint8_t mode) {
    lcd_i2c_send_nibble((bits & 0xF0) | mode);
    lcd_i2c_send_nibble(((bits << 4) & 0xF0) | mode);
}

/**
 * Load 5 custom characters (CGRAM 1..5) representing 1/5 to 5/5 vertical bar segments
 * so a 12-character meter has 60 smooth horizontal pixel steps.
 */
static void lcd_load_bargraph_chars(void) {
    static const uint8_t bar_patterns[5] = {
        0x10, // 1/5 column:  |....
        0x18, // 2/5 columns: ||...
        0x1C, // 3/5 columns: |||..
        0x1E, // 4/5 columns: ||||.
        0x1F  // 5/5 columns: |||||
    };
    // Write to CGRAM slot 1..5 (CGRAM address 0x40 + slot * 8)
    lcd_write_byte(0x40 + 8, LCD_MODE_CMD);
    for (int c = 0; c < 5; c++) {
        for (int row = 0; row < 8; row++) {
            lcd_write_byte(bar_patterns[c], LCD_MODE_CHR);
        }
    }
}

static void lcd_init_hardware(void) {
    i2c_init(LCD_I2C_INST, LCD_I2C_BAUD);
    gpio_set_function(LCD_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(LCD_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(LCD_SDA_PIN);
    gpio_pull_up(LCD_SCL_PIN);

    core1_wait_us(50000);

    // Probe 0x27 first, then fallback to 0x3F if needed
    uint8_t dummy = LCD_BL_BIT;
    if (i2c_write_timeout_us(LCD_I2C_INST, 0x27, &dummy, 1, false, 2000) >= 0) {
        lcd_i2c_addr = 0x27;
    } else if (i2c_write_timeout_us(LCD_I2C_INST, 0x3F, &dummy, 1, false, 2000) >= 0) {
        lcd_i2c_addr = 0x3F;
    }

    // Configure hardware I2C target address once for direct register writes
    i2c_hw_t *hw = i2c_get_hw(LCD_I2C_INST);
    hw->enable = 0;
    hw->tar = lcd_i2c_addr;
    hw->enable = 1;

    // Standard HD44780 4-bit initialization sequence (same as code.py)
    const uint8_t init_cmds[] = {0x33, 0x32, 0x06, 0x0C, 0x28, 0x01};
    for (size_t i = 0; i < sizeof(init_cmds); i++) {
        lcd_write_byte(init_cmds[i], LCD_MODE_CMD);
        if (init_cmds[i] == 0x01) {
            core1_wait_us(2000);
        } else {
            core1_wait_us(150);
        }
    }

    lcd_load_bargraph_chars();

    // Draw initial static labels on Line 1 ("IN :") and Line 2 ("OUT:")
    lcd_write_byte(LCD_LINE1_ADDR, LCD_MODE_CMD);
    const char *label1 = "IN :            ";
    for (int i = 0; i < 16; i++) lcd_write_byte((uint8_t)label1[i], LCD_MODE_CHR);

    lcd_write_byte(LCD_LINE2_ADDR, LCD_MODE_CMD);
    const char *label2 = "OUT:            ";
    for (int i = 0; i < 16; i++) lcd_write_byte((uint8_t)label2[i], LCD_MODE_CHR);
}

/**
 * Draw a 12-character (60-subpixel) bar graph starting at column 4 of the given line.
 */
static void __not_in_flash_func(lcd_draw_meter_bar)(uint8_t line_addr, int segments) {
    if (segments < 0)  segments = 0;
    if (segments > 60) segments = 60;

    // Position cursor right after "IN :" / "OUT:" (column 4)
    lcd_write_byte(line_addr + 4, LCD_MODE_CMD);

    for (int col = 0; col < 12; col++) {
        int rem = segments - (col * 5);
        uint8_t ch;
        if (rem >= 5) {
            ch = 5;   // Full 5/5 custom block character
        } else if (rem > 0) {
            ch = (uint8_t)rem; // Partial 1/5..4/5 custom bar character (CGRAM 1..4)
        } else {
            ch = ' '; // Empty space
        }
        lcd_write_byte(ch, LCD_MODE_CHR);
    }
}

// ==========================================
// Core 1 Entry: Potentiometer Polling & LCD Level Meter (100% SRAM-Resident)
// ==========================================
static void __not_in_flash_func(core1_pot_and_lcd_entry)(void) {
    int last_in_adc  = -100;
    int last_out_adc = -100;
    float local_target_in  = shared_target_in_gain;
    float local_target_out = shared_target_out_gain;

    float display_pre_level  = 0.0f;
    float display_post_level = 0.0f;
    int last_pre_segments  = 0;
    int last_post_segments = 0;

    uint32_t loop_count = 0;

    while (true) {
        // 1. Read both potentiometers and publish target gains to Core 0
        local_target_in  = update_pot_target_gain(INPUT_POT_ADC_CH,  MAX_INPUT_GAIN,  &last_in_adc,  local_target_in);
        local_target_out = update_pot_target_gain(OUTPUT_POT_ADC_CH, MAX_OUTPUT_GAIN, &last_out_adc, local_target_out);
        shared_target_in_gain  = local_target_in;
        shared_target_out_gain = local_target_out;

        // 2. Update LCD bar graphs every 6 iterations (~30 ms = ~33 FPS)
        if (++loop_count >= 6) {
            loop_count = 0;

            // Grab and reset peak accumulators from Core 0
            float raw_pre  = shared_pre_peak;
            shared_pre_peak = 0.0f;
            float raw_post = shared_post_peak;
            shared_post_peak = 0.0f;

            // Fast attack, smooth exponential decay for readable meter ballistics
            if (raw_pre > display_pre_level) {
                display_pre_level = raw_pre;
            } else {
                display_pre_level *= 0.80f;
                if (display_pre_level < 0.001f) display_pre_level = 0.0f;
            }

            if (raw_post > display_post_level) {
                display_post_level = raw_post;
            } else {
                display_post_level *= 0.80f;
                if (display_post_level < 0.001f) display_post_level = 0.0f;
            }

            // Only transmit over I2C when the visible bar segment count actually changes
            float pre_clamped  = (display_pre_level  > 1.0f) ? 1.0f : display_pre_level;
            float post_clamped = (display_post_level > 1.0f) ? 1.0f : display_post_level;
            int cur_pre_seg  = (int)(core1_fast_sqrtf(pre_clamped)  * 60.0f + 0.5f);
            int cur_post_seg = (int)(core1_fast_sqrtf(post_clamped) * 60.0f + 0.5f);

            if (cur_pre_seg != last_pre_segments) {
                lcd_draw_meter_bar(LCD_LINE1_ADDR, cur_pre_seg);
                last_pre_segments = cur_pre_seg;
            }
            if (cur_post_seg != last_post_segments) {
                lcd_draw_meter_bar(LCD_LINE2_ADDR, cur_post_seg);
                last_post_segments = cur_post_seg;
            }
        }

        core1_wait_us(5000);
    }
}

/**
 * Initialize ADC potentiometers and I2C1 LCD1602 before audio starts.
 */
static void init_pot_and_lcd(void) {
    adc_init();
    adc_gpio_init(OUTPUT_POT_GPIO);
    adc_gpio_init(INPUT_POT_GPIO);

    // Perform an initial potentiometer read before audio starts
    int dummy_in  = -100;
    int dummy_out = -100;
    shared_target_in_gain  = update_pot_target_gain(INPUT_POT_ADC_CH,  MAX_INPUT_GAIN,  &dummy_in,  0.0f);
    shared_target_out_gain = update_pot_target_gain(OUTPUT_POT_ADC_CH, MAX_OUTPUT_GAIN, &dummy_out, 0.0f);

    lcd_init_hardware();
}

/**
 * Launch Core 1 background task after I2S DMA has started cleanly.
 */
static void start_pot_and_lcd_core1(void) {
    multicore_launch_core1(core1_pot_and_lcd_entry);
}

/**
 * Called by Core 0 once per 64-sample audio block to report peak Pre-RNBO and Post-RNBO levels.
 */
static inline void submit_meter_peaks(float block_pre_peak, float block_post_peak) {
    if (block_pre_peak < 0.0f)  block_pre_peak  = -block_pre_peak;
    if (block_post_peak < 0.0f) block_post_peak = -block_post_peak;
    if (block_pre_peak > shared_pre_peak) {
        shared_pre_peak = block_pre_peak;
    }
    if (block_post_peak > shared_post_peak) {
        shared_post_peak = block_post_peak;
    }
}

#endif // POT_LCD_H
