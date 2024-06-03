#include "main.h"
#include "milis.h"
#include <stdbool.h>
#include <stdio.h>
#include <stm8s.h>
//#include "delay.h"
#include "adc_helper.h"
#include "daughterboard.h"
#include "uart1.h"

void init(void) {
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1); // taktovani MCU na 16MHz

    GPIO_Init(LED_PORT, LED_PIN, GPIO_MODE_OUT_PP_LOW_SLOW);

    GPIO_Init(BTN_PORT, BTN_PIN, GPIO_MODE_IN_FL_NO_IT);

    ADC2_SchmittTriggerConfig(ADC2_SCHMITTTRIG_CHANNEL14, DISABLE);
    ADC2_SchmittTriggerConfig(ADC2_SCHMITTTRIG_CHANNEL15, DISABLE);

    // nastavíme clock pro ADC2 (16MHz / 4 = 4MHz)
    ADC2_PrescalerConfig(ADC2_PRESSEL_FCPU_D4);
    // volíme zarovnání výsledku -- typicky do prava
    ADC2_AlignConfig(ADC2_ALIGN_RIGHT);
    // nastavíme multiplexer na některý kanál
    ADC2_Select_Channel(ADC2_CHANNEL_14);
    // rozběhnenem ADC
    ADC2_Cmd(ENABLE);
    // počkáme až se rozběhne
    ADC2_Startup_Wait();

    init_milis();
    init_uart1();
}

int main(void) {
    uint32_t time = 0;
    uint16_t vref, vtemp, temp;
    init();

    while (1) {
        if (milis() - time > 1111) {
            time = milis();

            vref = ADC_get(CHANNEL_VREF) * (uint32_t)(5000 + 512)/ 1023;
            ADC_get(CHANNEL_VTEMP);
            ADC_get(CHANNEL_VTEMP);
            ADC_get(CHANNEL_VTEMP);
            ADC_get(CHANNEL_VTEMP);
            ADC_get(CHANNEL_VTEMP);
            ADC_get(CHANNEL_VTEMP);
            ADC_get(CHANNEL_VTEMP);
            ADC_get(CHANNEL_VTEMP);
            vtemp = (uint32_t)ADC_get(CHANNEL_VTEMP) * (5000L + 512) / 1023;
           
            temp = (100L*vtemp -40000L)/195;
            printf("%u mV, %u mV,%u,%u ˚C\n", vref, vtemp,temp/10,temp%10);
        }
    }
}
/*-------------------------------  Assert -----------------------------------*/
#include "__assert__.h"
