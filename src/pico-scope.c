#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "bsp/board.h"
#include "tusb.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pwm.h"


#define CAPTURE_DEPTH 4096

static void cdc_task(void);
void capture();
void capture_init();
void core1_main();
void setpwm(uint32_t freq, uint32_t dutycycle);
 
extern uint8_t capture_buf[];
extern uint32_t trigger_index;
extern uint8_t triglev;         
extern bool trigger_pos_slope;  
extern uint8_t trigger_channel; 
extern uint32_t dma_multiplier;  
extern uint8_t round_robin;     
extern uint32_t samp_speed_divider;
extern uint32_t hor_trig_pct;
extern uint32_t extra1;
extern uint32_t extra2;

//----------
// MAIN
//----------
int main(void) {
  //multicore_launch_core1(core1_main);
  setpwm(1000,50);
  board_init();
  tusb_init();
  capture_init();

  while (1) {
    tud_task(); 
    cdc_task(); 
  }
  return 0;
}

//----------
// USB CDC 
//----------
int32_t txcount = 0;
uint8_t hexdigits[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
bool blockdone = false;

uint32_t decodeHex(uint8_t * p, uint8_t len) { // convert hex to int
    uint32_t result = 0;
	for (int i = 0; i < len; i++) {
		uint8_t x = *p++ - 48;
        if (x > 9) x -= 7;
		result = result * 16 + x; 
	}
	return result;
}

void write_hex(uint8_t x) {
	tud_cdc_n_write_char(0, hexdigits[x >> 4]);
	tud_cdc_n_write_char(0, hexdigits[x & 0x0f]);
}

void write_hex4(uint32_t x) {
	tud_cdc_n_write_char(0, hexdigits[x >> 12 & 0x0f]);
	tud_cdc_n_write_char(0, hexdigits[x >> 8  & 0x0f]);
	tud_cdc_n_write_char(0, hexdigits[x >> 4  & 0x0f]);
	tud_cdc_n_write_char(0, hexdigits[x & 0x0f]);
}

void write_block(){
	if (txcount >= CAPTURE_DEPTH) {
		tud_cdc_n_write_char(0, '*');
		tud_cdc_n_write_char(0, '/');
	    tud_cdc_n_write_flush(0);
		blockdone=true;
	}
    int32_t len = CAPTURE_DEPTH - txcount;
	if (len > 128) len=128;
    for (int i=0; i < len; i++) write_hex(capture_buf[txcount + i]);	
    txcount += len;
    tud_cdc_n_write_flush(0);
}

void tud_cdc_tx_complete_cb(uint8_t itf) {
	(void) itf; // ignore itif parameter
	if (!blockdone) write_block();
}

static void cdc_task(void) {
    if ( tud_cdc_n_available(0) )
    {
        uint8_t buf[256];
        uint32_t count = tud_cdc_n_read(0, buf, sizeof(buf));
		
		// request format: '*',command,data,'/' 
		
		// command 'm' 
		// byte 0 = '*'
		// byte 1 = 'm'  start measuring cycle
		// byte 2 = '0'..'9', sequence code
		// byte 3 = '/'
		
		// command 'p'
		// byte 0 = '*
		// byte 1 = 'p' // parameters for capturing analog data
		// byte 2,3 = trigger level in hex2
		// byte 4,5,6,7 = sample speed divider in hex4 (96=500kS/s, 47999 = 1kS/s)
		// byte 8 = trigger slope '0' = neg, '1' = pos
		// byte 9 = trigger channel '0' = ch1, '1' = ch2
		// byte 10,11 = dma_count multiplier, multiple of CAPTURE_DEPTH in hex2 == max acquiring time
        // byte 12 = channelselect : '1' = ch1, '2' = ch2, '3' = ch1 + ch2
		// byte 13,14 = horTrigPct, horizontal trigger position, 10..90% from sample range, value 10..90 in hex2
		// byte 15 = '/'

		// command 'f'  
		// byte 0 = '*'
		// byte 1 = 'f'  // pwm setting
		// byte 2,3,4,5 = frequency in Hz
		// byte 6,7 = duty cycle hex 2 0..100 % 
		// byte 8 = '/'


		if (count >= 4 && buf[0] == '*' && buf[count-1] == '/') {
			uint8_t command = buf[1];
			if (command == 'm'){
				capture();
				blockdone=false;
				txcount = 0;
				tud_cdc_n_write_char(0, '/');
				tud_cdc_n_write_char(0, '*');
				tud_cdc_n_write_char(0, 'm');
				tud_cdc_n_write_char(0, buf[2]);
				write_hex4(trigger_index);
				write_hex4(extra1);  // extra data 1 
				write_hex4(extra2);  // extra data 2
			    tud_cdc_n_write_flush(0);
			}
			else if (command =='p'){
				triglev = (uint8_t)decodeHex(&buf[2],2); 
				samp_speed_divider = decodeHex(&buf[4],4); 
				trigger_pos_slope = buf[8] == '1';
				trigger_channel = buf[9] =='1';
				dma_multiplier = decodeHex(&buf[10],2);
				round_robin = buf[12] - 48; // 1,2, or 3
				hor_trig_pct = decodeHex(&buf[13],2) ;
			}
			else if (command =='f'){
				uint32_t freq = decodeHex(&buf[2],4); 
				uint32_t dutycycle = decodeHex(&buf[6],2); 
				setpwm(freq, dutycycle);
			}
		}
    }
}


//----------
// PWM GEN
//----------
void setpwm(uint32_t freq, uint32_t dutycycle) {
    uint32_t periods = 125000000 / freq;
    uint32_t clkdiv = periods / 65000 + 1;
    if (clkdiv > 255) clkdiv = 255;
    uint32_t pwmdiv= periods / clkdiv;
    if (pwmdiv > 65535) pwmdiv= 65535;
    uint32_t  dc = pwmdiv * dutycycle / 100;	
    gpio_set_function(22, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(22);
	
    // Get some sensible defaults for the slice configuration. By default, the
    // counter is allowed to wrap over its maximum range (0 to 2**16-1)
    pwm_config config = pwm_get_default_config();
    
    // Set divider, reduces counter clock to sysclock/this value
    pwm_config_set_clkdiv_int(&config, clkdiv);
    
    // Load the configuration into our PWM slice, and set it running.
    pwm_init(slice_num, &config, true);

    // Set period of 4 cycles (0 to 3 inclusive)
    pwm_set_wrap(slice_num, (uint16_t) pwmdiv);

    // Set channel A output high for one cycle before dropping
    pwm_set_chan_level(slice_num, PWM_CHAN_A, (uint16_t) dc);
    
    // Set the PWM running
    pwm_set_enabled(slice_num, true);
}
