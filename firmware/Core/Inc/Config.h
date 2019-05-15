#ifndef _CONFIG_H_
#define _CONFIG_H_

#define SEG_DP_BIT 		0x20
#define LED_TIME_BIT	0x01
#define LED_TEMP_BIT	0x02

// ADC
#define ADCn_MAX_SAMPLES	         50
#define ADC_RESOLUTION		     4096.0
#define IN_ADC_VREF				     3300.0

#define MIN_TEMP		  60 
#define MAX_TEMP		  250

#define MIN_TIMER		   10   // in Seconds
#define MAX_TIMER		  300   // in Seconds

#define DISPLAY_TIMER  10   // in Seconds
//#define STOPPING_TIMER 30   // in 0.5 Seconds
#define MENU_TIMMING   20

typedef enum { 
	ADC1_Channel_1    = 0x00, 
	ADC1_Channel_2    = 0x01, 
	ADC1_Channel_3    = 0x02,
	
	ADC1_THERMOCOUPLE = 0x00, 
	ADC1_NTC          = 0x01,
	ADC1_KEYPAD       = 0x02
} ADC1_Channel_t;



#endif
