/*
 * ECE 3849 Lab2 starter project
 *
 * Gene Bogdanov    9/13/2017
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/*Lab 1 header files*/
#include <stdint.h>
#include <stdbool.h>
#include "driverlib/interrupt.h"
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "Crystalfontz128x128_ST7735.h"
#include <stdio.h>
#include "buttons.h"
#include "sampling.h"
#include "driverlib/timer.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"

#include <math.h>
#include "kiss_fft.h"
#include "_kiss_fft_guts.h"

#define PI 3.14159265358979f
#define NFFT 1024         // FFT length
#define KISS_FFT_CFG_SIZE (sizeof(struct kiss_fft_state)+sizeof(kiss_fft_cpx)*(NFFT-1))

#define PWM_FREQUENCY 20000 // PWM frequency = 20 kHz
#define MAX_BUTTON_PRESS 10 //FIFO
#define FIFO_SIZE 10        // Maximum items in FIFO

volatile uint32_t gTime = 8345; // time in hundredths of a second
extern volatile uint32_t gButtons; // from buttons.h
float scale;
extern volatile int32_t gADCBufferIndex;                // latest sample index
extern volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];   // circular buffer
extern volatile uint32_t gADCErrors;
const char * const gVoltageScaleStr[] = {"100 mV", "200 mV", "500 mV", " 1 V", " 2 V"};
volatile bool checkTrigger;
float fVoltsPerDiv[] = {0.1, 0.2, 0.5, 1, 2};
int triggerSlope = 1;
int voltsperDiv = 4;
uint32_t gSystemClock = 120000000; // [Hz] system clock frequency
char mode = 1;
volatile uint16_t sample[LCD_HORIZONTAL_MAX];
volatile int sampledisp[LCD_HORIZONTAL_MAX];
static char kiss_fft_cfg_buffer[KISS_FFT_CFG_SIZE]; // Kiss FFT config memory
size_t buffer_size = KISS_FFT_CFG_SIZE;
kiss_fft_cfg cfg;                        // Kiss FFT config
static kiss_fft_cpx in[NFFT], out[NFFT]; // complex waveform and spectrum buffers
volatile uint16_t samplefft[NFFT];
float out_db[NFFT];
/*
 *  ======== main ========
 */
int main(void)
{
    IntMasterDisable();

    Crystalfontz128x128_Init(); // Initialize the LCD display driver
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation

    tContext sContext;
    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8); // select font


    // hardware initialization goes here
    ButtonInit();
    adcInit();
    // configure M0PWM2, at GPIO PF2, BoosterPack 1 header C1 pin 2
            // configure M0PWM3, at GPIO PF3, BoosterPack 1 header C1 pin 3
            SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
            GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3);
            GPIOPinConfigure(GPIO_PF2_M0PWM2);
            GPIOPinConfigure(GPIO_PF3_M0PWM3);
            GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3,
                             GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

            // configure the PWM0 peripheral, gen 1, outputs 2 and 3
            SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
            PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1); // use system clock without division
            PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
            PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, roundf((float)gSystemClock/PWM_FREQUENCY));
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, roundf((float)gSystemClock/PWM_FREQUENCY*0.4f));
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, roundf((float)gSystemClock/PWM_FREQUENCY*0.4f));
            PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
            PWMGenEnable(PWM0_BASE, PWM_GEN_1);

    /* Start BIOS */
    BIOS_start();

    return (0);
}

void WaveformTask(UArg arg1, UArg arg2) //highest priority
{
    IntMasterEnable();
    int i, trig;
    while (true) {
        Semaphore_pend(semaphoreWaveform, BIOS_WAIT_FOREVER);
        if(mode){
            trig = triggerSlope ? RisingTrigger(): FallingTrigger();//either rise or fall
            for (i = 0; i < LCD_HORIZONTAL_MAX - 1; i++){

                // Copies waveform into the local buffer
                sample[i] = gADCBuffer[ADC_BUFFER_WRAP(trig - LCD_HORIZONTAL_MAX / 2 + i)];
            }
            Semaphore_post(semaphoreProcessing);
        }
        else{
            for (i = 0; i < NFFT; i++) { // generate an input waveform
                samplefft[i] = gADCBuffer[ADC_BUFFER_WRAP(i)];

            }//end for
            Semaphore_post(semaphoreProcessing);
        }//end if
    }//end while 1
}//end func

void ProcessingTask(UArg arg1, UArg arg2)
{
    int i;
    cfg = kiss_fft_alloc(NFFT, 0, kiss_fft_cfg_buffer, &buffer_size); // init Kiss FFT
    while (true){
        Semaphore_pend(semaphoreProcessing, BIOS_WAIT_FOREVER);
        if(mode){
            scale = (VIN_RANGE * PIXELS_PER_DIV) / ((1 << ADC_BITS) * fVoltsPerDiv[voltsperDiv]);
        for (i = 0; i < LCD_HORIZONTAL_MAX - 1; i++){
            sampledisp[i] = LCD_VERTICAL_MAX / 2 - (int)roundf(scale * ((int)sample[i] - ADC_OFFSET));
          }//end for

        Semaphore_post(semaphoreDisplay);
        Semaphore_post(semaphoreWaveform);

        }
        else{
         for (i = 0; i < NFFT; i++) {    // generate an input waveform
         in[i].r = samplefft[i]; // real part of waveform
         in[i].i = 0;                  // imaginary part of waveform
         }
         kiss_fft(cfg, in, out); //compute FFT
         // convert first 128 bins of out[] to dB for display
         for (i = 0; i < LCD_HORIZONTAL_MAX - 1; i++){
          out_db[i] = 160 - ((10.0f) * log10f((out[i].r * out[i].r) + (out[i].i * out[i].i)));
         }//end for
         Semaphore_post(semaphoreDisplay);
         Semaphore_post(semaphoreWaveform);
    }
  }//end while 1
}//end function

void DisplayTask(UArg arg1, UArg arg2)
{
        tContext sContext;
        GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
        GrContextFontSet(&sContext, &g_sFontFixed6x8);     // Select font
        int i, y, yP;
        char str1[50];
        tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1, GrContextDpyHeightGet(&sContext)-1};
        while(true) {
            Semaphore_pend(semaphoreDisplay, BIOS_WAIT_FOREVER);
            if(mode){
                                GrContextForegroundSet(&sContext, ClrBlack);
                                GrRectFill(&sContext, &rectFullScreen); // fill screen with black

                                //blue grid
                                GrContextForegroundSet(&sContext, ClrBlue);
                                for(i = -3; i < 4; i++) {
                                    GrLineDrawH(&sContext, 0, LCD_HORIZONTAL_MAX - 1, LCD_VERTICAL_MAX/2 + i * PIXELS_PER_DIV);
                                    GrLineDrawV(&sContext, LCD_VERTICAL_MAX/2 + i * PIXELS_PER_DIV, 0, LCD_HORIZONTAL_MAX - 1);
                                }

                                //waveform
                                GrContextForegroundSet(&sContext, ClrYellow);
                                 for(i = 0; i < LCD_HORIZONTAL_MAX - 1; i++) {
                                    y = sampledisp[i];
                                    GrLineDraw(&sContext, i, yP, i + 1, y);
                                    yP = y;
                                }
                                //trigger direction, volts per div, cpu load
                                GrContextForegroundSet(&sContext, ClrWhite); //white text
                                if(triggerSlope){
                                    GrLineDraw(&sContext, 105, 10, 115, 10);
                                    GrLineDraw(&sContext, 115, 10, 115, 0);
                                    GrLineDraw(&sContext, 115, 0, 125, 0);
                                    GrLineDraw(&sContext, 112, 6, 115, 2);
                                    GrLineDraw(&sContext, 115, 2, 118, 6);
                                }else{
                                    GrLineDraw(&sContext, 105, 10, 115, 10);
                                    GrLineDraw(&sContext, 115, 10, 115, 0);
                                    GrLineDraw(&sContext, 115, 0, 125, 0);
                                    GrLineDraw(&sContext, 112, 3, 115, 7);
                                    GrLineDraw(&sContext, 115, 7, 118, 3);
                                }

                                GrContextForegroundSet(&sContext, ClrWhite); //white text
                                GrStringDraw(&sContext, "20 us", -1, 4, 0, false);
                                GrStringDraw(&sContext, gVoltageScaleStr[voltsperDiv], -1, 50, 0, false);

                                //Draw Missing Trigger indicator
                                GrContextForegroundSet(&sContext, ClrRed); //Red text
                                                while (checkTrigger == true){
                                                    snprintf(str1, sizeof(str1), "Missing Trigger");
                                                    GrStringDraw(&sContext, str1, -1, 20, 64, false);
                                                    break;
                                                }

                                GrFlush(&sContext); // flush the frame buffer to the LCD
            }
            else{
                GrContextForegroundSet(&sContext, ClrBlack);
                GrRectFill(&sContext, &rectFullScreen); // fill screen with black

                //blue grid
                GrContextForegroundSet(&sContext, ClrBlue);
                for(i = -3; i < 4; i++) {
                GrLineDrawH(&sContext, 0, LCD_HORIZONTAL_MAX - 1, LCD_VERTICAL_MAX/2 + i * PIXELS_PER_DIV);
                }
                for (i = 0; i < 7; i++) {
                GrLineDrawV(&sContext, i * PIXELS_PER_DIV, 0, LCD_HORIZONTAL_MAX - 1);
                }

                //waveform
                GrContextForegroundSet(&sContext, ClrYellow);
                for(i = 0; i < LCD_HORIZONTAL_MAX - 1; i++) {
                y = out_db[i];
                GrLineDraw(&sContext, i, yP, i + 1, y);
                yP = y;
                }
                //frequency and decibel
                GrContextForegroundSet(&sContext, ClrWhite); //white text
                GrStringDraw(&sContext, "20 kHz", -1, 4, 0, false);
                GrStringDraw(&sContext, "10 dBV", -1, 50, 0, false);

                GrFlush(&sContext); // flush the frame buffer to the LCD

            }//end else
      }//end while 2
}//end function

void ButtonTask(UArg arg1, UArg arg2)
{
    char button;
    while(1){
        Semaphore_pend(semaphoreButton, BIOS_WAIT_FOREVER);
        uint32_t gpio_buttons =
                    (~GPIOPinRead(GPIO_PORTJ_BASE, 0xff) & (GPIO_PIN_1 | GPIO_PIN_0)) | //EK-TM4C1294XL buttons in positions 0 and 1
                    (~GPIOPinRead(GPIO_PORTH_BASE, 0xff) & (GPIO_PIN_1)) << 1 |         //BoosterPack button 1
                    (~GPIOPinRead(GPIO_PORTK_BASE, 0xff) & (GPIO_PIN_6)) >> 3 |         //BoosterPack button 2
                    ~GPIOPinRead(GPIO_PORTD_BASE, 0xff) & (GPIO_PIN_4);                 //BoosterPack buttons Joystick select


            uint32_t old_buttons = gButtons;    // save previous button state
            ButtonDebounce(gpio_buttons);       // Run the button debouncer. The result is in gButtons.
            ButtonReadJoystick();               // Convert joystick state to button presses. The result is in gButtons.
            uint32_t presses = ~old_buttons & gButtons;   // detect button presses (transitions from not pressed to pressed)
            presses |= ButtonAutoRepeat();      // autorepeat presses if a button is held long enough

            if (presses & 1) { // EK-TM4C1294XL button 1 pressed
                button = 'a';
                Mailbox_post(mailbox0, &button, BIOS_NO_WAIT);
            }
            if (presses & 2) { // EK-TM4C1294XL button 2 pressed
                button = 'b';
                Mailbox_post(mailbox0, &button, BIOS_NO_WAIT);
                }
            if (presses & 4) { //S1
                button = 'c';
                Mailbox_post(mailbox0, &button, BIOS_NO_WAIT);
            }
            if (presses & 8) { //S2
                button = 'd';
                Mailbox_post(mailbox0, &button, BIOS_NO_WAIT);
            }
        }//end while 1
    }//end function

void UserInputTask(UArg arg1, UArg arg2)
{
    char button;
    while(true){
        if(Mailbox_pend(mailbox0, &button, BIOS_WAIT_FOREVER)){
        switch(button){
         case 'a':
             voltsperDiv = voltsperDiv >= 4 ? 4 : voltsperDiv++; //Extra credit lab 1
             break;
         case 'b':
             voltsperDiv = voltsperDiv <= 0 ? 0 : voltsperDiv--; //Extra credit lab 1
             break;
         case 'c':
             mode = !mode; //spectrum mode
             break;
         case'd':
             triggerSlope = !triggerSlope; //Trigger slope change
             break;
      }
         Semaphore_post(semaphoreDisplay);
    }
  }//end while
}//end function

void Clock0Task(UArg arg1, UArg arg2)
{
    Semaphore_post(semaphoreButton);
}

