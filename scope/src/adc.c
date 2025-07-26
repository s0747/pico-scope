
#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/adc.h"
#include "hardware/dma.h"

// We are putting the ADC in free-running capture mode at 0.5 Msps
//
// A DMA channel will be attached to the ADC sample FIFO
//
// Configure the ADC to right-shift samples to 8 bits of significance, so we
// can DMA into a byte buffer
//
// This could be extended to use the ADC's round robin feature to sample two
// channels concurrently at 0.25 Msps each.
//

// Channel 0 is 
#define CAPTURE_CHANNEL 0 // GPIO26
#define CAPTURE_DEPTH 4096  // always power of 2, also in scope.c and in channel_config_set_ring

//capture outputs
uint8_t capture_buf[CAPTURE_DEPTH] __attribute__ ((aligned(CAPTURE_DEPTH)));
uint32_t trigger_index = 0;         // index in capture_buf where triggering occured
uint32_t posttriggercount = 1000;   // # captures after triggering
uint32_t pretriggercount = 200;     // # captures before triggering
uint32_t extra1; 
uint32_t extra2;

// capture inputs
uint8_t triglev = 128;
bool trigger_pos_slope = true;      // true = trigger on positive slope 
float samp_speed_divider = 0;       // sample speed divider, sample speed = 48MHz / (divider+1)
uint8_t trigger_channel;            // trigger channel, 0=ch1, 1=ch2
uint8_t dma_multiplier = 4;         // dma_count multiplier, multiple of CAPTURE_DEPTH == max acquiring time
uint8_t round_robin;                // 1 = ch1, 2=ch2, 3=ch1+ch2
uint32_t hor_trig_pct;               // horizontal trigger % (10..90)

// get dma channel
uint8_t  dma_chan;
volatile uint32_t * trans_remain_count;  // remaining # of dma transfers
uint32_t dma_transfer_count;

void capture_init(){
    dma_chan = (uint8_t) dma_claim_unused_channel(true);
    trans_remain_count = (uint32_t*)(DMA_BASE +  dma_chan * 0x040 + 0x08);
    dma_transfer_count = 4 * CAPTURE_DEPTH; //dma_multiplier * CAPTURE_DEPTH;
}

void capture() {

    // Init GPIO for analogue use: hi-Z, no pulls, disable digital input buffer.
    adc_gpio_init(26 + CAPTURE_CHANNEL);
    adc_gpio_init(27 + CAPTURE_CHANNEL);
    adc_init();
    adc_select_input(CAPTURE_CHANNEL);
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // We won't see the ERR bit because of 8 bit reads; disable.
        true     // Shift each sample to 8 bits when pushing to FIFO
    );
    
    adc_set_round_robin(round_robin);
    
    // Divisor of 0 -> full speed. Free-running capture with the divider is
    // equivalent to pressing the ADC_CS_START_ONCE button once per `div + 1`
    // cycles (div not necessarily an integer). Each conversion takes 96
    // cycles, so in general you want a divider of 0 (hold down the button
    // continuously) or > 95 (take samples less frequently than 96 cycle
    // intervals). This is all timed by the 48 MHz ADC clock.
    adc_set_clkdiv(samp_speed_divider);

    // Set up the DMA to start transferring data as soon as it appears in FIFO
    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);

	// Reading from constant address, writing to incrementing byte addresses
	channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
	channel_config_set_read_increment(&cfg, false);
	channel_config_set_write_increment(&cfg, true);
    channel_config_set_ring(&cfg,true,12); // 11= 2048 size ring buffer 

	// Pace transfers based on availability of ADC samples
	channel_config_set_dreq(&cfg, DREQ_ADC);

	dma_channel_configure(dma_chan, &cfg,
		capture_buf,    // dst
		&adc_hw->fifo,  // src
		dma_transfer_count,  // # of dma transfers
		true            // start immediately
	);

    for (uint32_t i=0; i < CAPTURE_DEPTH; i++) {capture_buf[i]=(uint8_t)i % 255;}   

    uint32_t last_index = 0;
    uint8_t hyspos =  triglev + 10;
    uint8_t hysneg =  triglev - 10;
    bool hys_ok = false;
    trigger_index = 0;
    pretriggercount = hor_trig_pct * CAPTURE_DEPTH / 100;
    extra1 = pretriggercount;
    posttriggercount = CAPTURE_DEPTH - pretriggercount ; // # transfers after triggering
	adc_run(true); // start adc
    bool singlechannel = round_robin != 3; // 1= ch1, 2 = ch2, 3 = ch1+ch2
    // do until dma transfer is completed	
    do {
        uint32_t index = dma_transfer_count - *trans_remain_count;
        uint32_t next_index = index  & (uint32_t)(CAPTURE_DEPTH -1); // next dma address
        if (last_index != next_index) { // dma transfer occured 
            uint8_t value = capture_buf[last_index];  // get the value
            if (trigger_index == 0 && index > pretriggercount) {  // not yet triggered and more than pretriggercount samples
                if ((index & 1) == trigger_channel  || singlechannel) {
                    if (trigger_pos_slope) {
                        if (value < hysneg) hys_ok=true;
                        if (hys_ok && (value > triglev)) {trigger_index = last_index; extra2 = index;}
                    }
                    else { // trigger on negative slope
                        if (value > hyspos) hys_ok= true;
                        if (hys_ok && value < triglev) {trigger_index = last_index; extra2 = index;}
                    }
                }
            }
            else if (trigger_index > 0) posttriggercount --;
            last_index = next_index;
        }
    } while (*trans_remain_count != 0 && posttriggercount > 0);
   	// Once DMA finishes, stop any new conversions from starting, and clean up
	// the FIFO in case the ADC was still mid-conversion.
    adc_run(false);
    adc_fifo_drain();    
    dma_channel_abort(dma_chan); // abort remaining transfers

}
