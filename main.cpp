//
//  main.cpp
//  baremetal
//
//  Created by Stefan Brunner on 15.03.24.
//


//#define RNBO_NOSTL          // do not use any C++ standard library objects and functionality
#define RNBO_USE_FLOAT32    // use 32 bit floats instead of 64 bit double values (degrades audio performance a little)
#define RNBO_NOTHROW        // no exceptopns

//#define RNBO_USECUSTOMPLATFORM    // completely replace the RNBO platform methods wit your own

#define RNBO_USECUSTOMPLATFORMPRINT // only replace the print out methods
#define RNBO_USECUSTOMALLOCATOR     // use a custom memory allocator

#define RNBO_FIXEDLISTSIZE 128      // when you use lists and want to avoid allocations, define this
                                    // to set the maximum allowed list length

//#define RNBO_MINENGINEQUEUESIZE 256 // the built in minimal engine works with a fixed queue size for
                                    // scheduled events, it defaults to 128 and you can set it via this define


#define SAMPLERATE  44100                   // arbitrary sample rate chosen for this demo
#define VECTORSIZE  64                      // arbitrary vector size chosen for this demo

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "pico/binary_info.h"

#include "tlsf.h"   // a nice and simple pool allocator https://github.com/mattconte/tlsf
#include "audio_i2s_full_duplex.pio.h"

tlsf_t myPool;      // the memory pool

// we are defining our custom memory allocation functions BEFORE we include the RNBO header
// so we have them available all over the place

namespace RNBO {
	namespace Platform {

		void *malloc(size_t size) {
			return tlsf_malloc(myPool, size);
		}

		void  free(void * ptr)
		{
			tlsf_free(myPool, ptr);
		}

		void *realloc(void * ptr, size_t size) {
			return tlsf_realloc(myPool, ptr, size);
		}

		void *calloc(size_t count, size_t size);
	}
}

// you can include the rnbo source header directly

// if you do not want to add the include dir in your poject/cmake/makefile, you can uncomment the following line to set it
//#define RNBO_LIB_PREFIX common

#include "export/rnbo_source.cpp.h"

// more custom platform definitions

namespace RNBO {
	namespace Platform {
		void *calloc(size_t count, size_t size) {
			auto mem = malloc(count * size);
			if (mem) {
				memset(mem, 0, count * size);
			}
			return mem;
		}

        /**
         * @brief Print a message to stdout or some kind of log
         */
        static void printMessage(const char* message) {
            printf("%s\n", message);
        }

        /**
         * @brief Prints an error message
         *
         * Defaults to printMessage.
         */
        static void printErrorMessage(const char* message) {
            printMessage(message);
        }
	}
}

// the minimal export provides a built in "Engine", with very minimal capabilities - it is mainly used
// to schedule clock events

// if you want to get any MIDI notifications out of your RNBO patcher you will have to provide your own
// engine call that overloads the MIDI out handlers, same for messages etc.
class MyEngine : public RNBO::MinimalEngine<>
{
public:
    MyEngine(RNBO::PatcherInterface* patcher)
    : RNBO::MinimalEngine<>(patcher)
    {}
};

// ==========================================
// Hardware Pin Configuration (PCM1808 + PCM5102A + Potentiometers)
// ==========================================
#define I2S_BCK_PIN         10  // GP10 (Shared BCK for PCM1808 & PCM5102A)
#define I2S_LRCK_PIN        11  // GP11 (Shared LRCK/LCK for PCM1808 & PCM5102A)
#define I2S_DIN_PIN         12  // GP12 (Pico 2 -> PCM5102A DIN)
#define I2S_DOUT_PIN        9   // GP9  (PCM1808 OUT -> Pico 2)
#define I2S_SCK_PIN         21  // GP21 (Pico 2 -> PCM1808 SCK, 256fs MCLK)

#define OUTPUT_POT_GPIO     26  // GP26 = ADC0 (Output Gain)
#define OUTPUT_POT_ADC_CH   0
#define INPUT_POT_GPIO      27  // GP27 = ADC1 (Input Gain)
#define INPUT_POT_ADC_CH    1

// Number of samples per audio block = RNBO VECTORSIZE (64 samples)
#define SAMPLES_PER_BUFFER  VECTORSIZE
// Number of 32-bit words per stereo block (L + R)
#define WORDS_PER_BUFFER    (SAMPLES_PER_BUFFER * 2)

#define MAX_INPUT_GAIN      1.0f
#define MAX_OUTPUT_GAIN     1.0f

bi_decl(bi_3pins_with_names(I2S_DIN_PIN, "I2S DIN (DAC)", I2S_BCK_PIN, "I2S BCK", I2S_LRCK_PIN, "I2S LRCK"));
bi_decl(bi_2pins_with_names(I2S_DOUT_PIN, "I2S DOUT (ADC)", I2S_SCK_PIN, "I2S SCK (MCLK)"));

// DMA ping-pong (double) buffers (interleaved 32-bit L/R)
static int32_t rx_buffers[2][WORDS_PER_BUFFER];
static int32_t tx_buffers[2][WORDS_PER_BUFFER];

// RNBO instance placed in static storage to prevent stack overflow on the microcontroller
static RNBO::rnbomatic<MyEngine> rnbo;

static int dma_chan_tx;
static int dma_chan_rx;

// ==========================================
// ADC (Potentiometer) Reading & Full-Duplex I2S Initialization
// ==========================================
static int read_pot_averaged(uint adc_channel) {
    adc_select_input(adc_channel);
    (void)adc_read(); // Dummy read right after switching channels to prevent mux crosstalk
    uint32_t sum = 0;
    for (int i = 0; i < 8; i++) {
        sum += adc_read();
    }
    return (int)(sum >> 3); // 0 to 4095
}

/**
 * Read potentiometer ADC value and update target gain while ignoring small jitter (noise).
 * - Deadzone: Force mute (0.0f) when near minimum (< 16)
 * - Hysteresis: Only update target gain when the reading changes by more than +-12
 */
static float update_pot_target_gain(uint adc_channel, float max_gain, int *last_adc, float current_target_gain) {
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

static void init_i2s_full_duplex(void) {
    PIO pio = pio0;

    // Step 1: Claim 3 state machines on PIO0
    //   sm_tx (0)  : BCK & LRCK generation + audio transmission (DIN) to PCM5102A
    //   sm_rx (1)  : Synchronized audio reception (DOUT) from PCM1808
    //   sm_mclk (2): System clock (SCK = 256fs) generation for PCM1808
    uint sm_tx   = 0;
    uint sm_rx   = 1;
    uint sm_mclk = 2;
    pio_sm_claim(pio, sm_tx);
    pio_sm_claim(pio, sm_rx);
    pio_sm_claim(pio, sm_mclk);

    // Step 2: Load PIO programs into PIO instruction memory and bind GPIO pins
    uint offset_mclk = pio_add_program(pio, &mclk_output_program);
    uint offset_tx   = pio_add_program(pio, &i2s_tx_master_program);
    uint offset_rx   = pio_add_program(pio, &i2s_rx_sync_program);

    mclk_output_program_init(pio, sm_mclk, offset_mclk, I2S_SCK_PIN);
    i2s_tx_master_program_init(pio, sm_tx, offset_tx, I2S_DIN_PIN, I2S_BCK_PIN);
    i2s_rx_sync_program_init(pio, sm_rx, offset_rx, I2S_DOUT_PIN);

    // Step 3: Configure integer clock dividers to maintain exact MCLK:BCK = 4:1 (256fs:64fs) phase lock for PCM1808
    uint16_t mclk_div_int = 6;
    uint16_t bclk_div_int = mclk_div_int * 4;
    pio_sm_set_clkdiv_int_frac(pio, sm_mclk, mclk_div_int, 0);
    pio_sm_set_clkdiv_int_frac(pio, sm_tx,   bclk_div_int, 0);
    pio_sm_set_clkdiv_int_frac(pio, sm_rx,   bclk_div_int, 0);

    // Step 4: Claim and configure 2 DMA channels (TX and RX) to transfer audio data without CPU intervention
    dma_chan_tx = dma_claim_unused_channel(true);
    dma_chan_rx = dma_claim_unused_channel(true);

    // TX DMA: tx_buffers[0] (RAM) -> PIO TX FIFO (txf), 32-bit words
    dma_channel_config cfg_tx = dma_channel_get_default_config(dma_chan_tx);
    channel_config_set_transfer_data_size(&cfg_tx, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg_tx, true);
    channel_config_set_write_increment(&cfg_tx, false);
    channel_config_set_dreq(&cfg_tx, pio_get_dreq(pio, sm_tx, true));
    dma_channel_configure(dma_chan_tx, &cfg_tx, &pio->txf[sm_tx], tx_buffers[0], WORDS_PER_BUFFER, false);

    // RX DMA: PIO RX FIFO (rxf) -> rx_buffers[0] (RAM), 32-bit words
    dma_channel_config cfg_rx = dma_channel_get_default_config(dma_chan_rx);
    channel_config_set_transfer_data_size(&cfg_rx, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg_rx, false);
    channel_config_set_write_increment(&cfg_rx, true);
    channel_config_set_dreq(&cfg_rx, pio_get_dreq(pio, sm_rx, false));
    dma_channel_configure(dma_chan_rx, &cfg_rx, rx_buffers[0], &pio->rxf[sm_rx], WORDS_PER_BUFFER, false);

    // Step 5: Start DMA channels and enable all 3 PIO state machines on the exact same system clock cycle
    dma_start_channel_mask((1u << dma_chan_tx) | (1u << dma_chan_rx));
    pio_clkdiv_restart_sm_mask(pio, (1u << sm_mclk) | (1u << sm_tx) | (1u << sm_rx));
    pio_enable_sm_mask_in_sync(pio, (1u << sm_mclk) | (1u << sm_tx) | (1u << sm_rx));
}

static inline int32_t float_to_int32(float x) {
    if (x > 1.0f)  x = 1.0f;
    if (x < -1.0f) x = -1.0f;
    return (int32_t)(x * 2147483520.0f);
}

int main()
{
    stdio_init_all();

	// a memory pool based custom allocator- in no way necessary on this Desktop machine
	// but quite handy for embedded systems
    const size_t poolSize = 256 * 1024;
    void* mem = ::malloc(poolSize);
    myPool = tlsf_create_with_pool(mem, poolSize);

	/* some notes regarding HEAP and allocations: what do we allocate on the HEAP:

	 - sample buffers (data and buffer~) - allocated in initialize, unless you resize them
	 - lists (ONLY if their length exceeds the RNBO_FIXEDLISTSIZE)
	 - signal buffers (you can avoid that by exporting with a fixed audio vector size), but these will all be allocated when you
       you call prepareToProcess, no allocations later on
	 */

	rnbo.initialize();	// after initialize all sample buffers are allocated

    // really simple and stupid way to set some user data on your engine for example for callbacks
//    static_cast<MyEngine*>(rnbo.getEngine())->setData(nullptr);

    const auto numInputs = rnbo.getNumInputChannels();
    const auto numOutputs = rnbo.getNumOutputChannels();

    std::vector<std::vector<RNBO::SampleValue>> inChannels(numInputs);
    for (auto& c : inChannels) c.resize(VECTORSIZE, 0.0f);
    std::vector<std::vector<RNBO::SampleValue>> outChannels(numOutputs);
    for (auto& c : outChannels) c.resize(VECTORSIZE, 0.0f);

    std::vector<RNBO::SampleValue*> ins;
    for (auto& c : inChannels) ins.push_back(c.data());
    std::vector<RNBO::SampleValue*> outs;
    for (auto& c : outChannels) outs.push_back(c.data());

    // this could allocate if:
    // - you did not export with a fixed sample block size
    // - some (rare) objects might allocate buffers based on the sample rate
    //   if you must avoid this, you will need to check for your specific patcher
    rnbo.prepareToProcess(SAMPLERATE, VECTORSIZE, true);

	// from here on out there should be no allocations, unless you exceed the fixed list size,
	// resize audio buffers or change samplerate or vector size by calling prepareToProcess

    // process one buffer, this will already set intial values etc.
    // processing will also advance the internal millisecond time (and beattime if transport is started)
    rnbo.process(ins.data(), numInputs, outs.data(), numOutputs, VECTORSIZE);

    if (rnbo.getNumParameters() > 0) {
        rnbo.setParameterValue(0, 40.0f, RNBO::TimeNow);
        rnbo.setParameterValue(1, 40.0f, RNBO::TimeNow);
        rnbo.setParameterValue(2, 40.0f, RNBO::TimeNow);
    }

    // Initialize ADC for potentiometers
    adc_init();
    adc_gpio_init(OUTPUT_POT_GPIO);
    adc_gpio_init(INPUT_POT_GPIO);

    // Start full-duplex I2S audio engine
    init_i2s_full_duplex();

    float current_in_gain  = 0.0f;
    float target_in_gain   = 0.0f;
    float current_out_gain = 0.0f;
    float target_out_gain  = 0.0f;
    int last_in_adc  = -100;
    int last_out_adc = -100;

    int dma_buf_idx = 0;

    while (true) {
        int next_dma_idx = dma_buf_idx ^ 1;

        // 1. Wait for current 64-sample DMA block transfer to finish and immediately swap buffers
        dma_channel_wait_for_finish_blocking(dma_chan_tx);
        dma_channel_transfer_from_buffer_now(dma_chan_tx, tx_buffers[next_dma_idx], WORDS_PER_BUFFER);

        dma_channel_wait_for_finish_blocking(dma_chan_rx);
        dma_channel_transfer_to_buffer_now(dma_chan_rx, rx_buffers[next_dma_idx], WORDS_PER_BUFFER);

        int cpu_buf_idx = dma_buf_idx;
        dma_buf_idx = next_dma_idx;

        // 2. Read potentiometers (Input Gain on GP27 / Output Gain on GP26) and update target gains
        target_in_gain  = update_pot_target_gain(INPUT_POT_ADC_CH,  MAX_INPUT_GAIN,  &last_in_adc,  target_in_gain);
        target_out_gain = update_pot_target_gain(OUTPUT_POT_ADC_CH, MAX_OUTPUT_GAIN, &last_out_adc, target_out_gain);

        const int32_t *in_samples  = rx_buffers[cpu_buf_idx];
        int32_t       *out_samples = tx_buffers[cpu_buf_idx];
        const float norm_scale = 1.0f / 2147483648.0f;

        // 3. Sum L/R I2S inputs to mono, apply smoothed input gain, and feed into both RNBO input channels.
        //    Note: When a mono (TS) guitar plug is inserted into a stereo (TRS) jack, Ring (R) is grounded (0)
        //    and only Tip (L) carries signal. Summing (L + R) routes the mono signal into both stereo inputs.
        for (int i = 0; i < VECTORSIZE; i++) {
            current_in_gain += (target_in_gain - current_in_gain) * 0.005f;
            float in_l = (float)in_samples[i * 2 + 0] * norm_scale;
            float in_r = (float)in_samples[i * 2 + 1] * norm_scale;
            float mono_in = (in_l + in_r) * current_in_gain;
            if (numInputs >= 1) ins[0][i] = mono_in;
            if (numInputs >= 2) ins[1][i] = mono_in;
        }

        // 4. Execute RNBO DSP block processing
        rnbo.process(ins.data(), numInputs, outs.data(), numOutputs, VECTORSIZE);

        // 5. Apply smoothed output gain to RNBO float output buffers and convert back to 32-bit I2S integers
        for (int i = 0; i < VECTORSIZE; i++) {
            current_out_gain += (target_out_gain - current_out_gain) * 0.005f;
            float out_l = (numOutputs >= 1) ? outs[0][i] : 0.0f;
            float out_r = (numOutputs >= 2) ? outs[1][i] : out_l;
            out_samples[i * 2 + 0] = float_to_int32(out_l * current_out_gain);
            out_samples[i * 2 + 1] = float_to_int32(out_r * current_out_gain);
        }
    }

	return 0;
}
