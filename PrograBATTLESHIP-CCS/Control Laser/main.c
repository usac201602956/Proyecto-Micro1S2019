/*----------------------------------------------LIBRERÍAS DE C-------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
/*----------------------------------------------LIBRERÍAS TIVA C-----------------------------------------------*/
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/uart.h"
/*-----------------------------------------------DEFINICIONES--------------------------------------------------*/
#define F_Hz 50  //Frecuencia a la que trabajan los servomotores
/*--------------------------------------------VARIABLES GLOBALES-----------------------------------------------*/
char TextOne[] = "POSICION: ";
uint32_t T_cycles;
uint32_t V_HighTime, H_HighTime = 0;  //Variables que almacenan el tiempo en alto del pulso de la señal PWM.

uint32_t V_Sample[4];  //Vector que almacena las muestras de voltaje del mov. vertical.
uint32_t H_Sample[4];  //Vector que almacena las muestras de voltaje del mov. horizontal.
int H_Angle, V_Angle, H_Average, V_Average, H_Pos, V_Pos, i = 0;
/*--------------------------------------------CONFIGURACIÓN GPIO-----------------------------------------------*/
void GPIO(void){
    //PARA ADC
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);  //Habilitar puerto D.
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1);  //Pin PD0 y PD1 como tipo ADC.
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_ANALOG);  //PD0 y PD1 como pines analógicos.
    //PARA PWM
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);  //Habilitar puerto C
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4|GPIO_PIN_5);  //Pin PC4 y PC5 como tipo PWM
}
/*--------------------------------------------CONFIGURACIÓN ADC------------------------------------------------*/
void ADC(void){
    //ADC0 MOVIMIENTO HORIZONTAL
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);  //Habilitar ADC0.
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);  //Se configura secuenciador 2 de ADC0.
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH6);  //Se lee canal 6 --> PD1.
    ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_CH6);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 2, ADC_CTL_CH6);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 3, ADC_CTL_CH6|ADC_CTL_IE|ADC_CTL_END);  //Termina de realizar la última muestra del ADC0 y levanta un interrupción.
    ADCSequenceEnable(ADC0_BASE, 2);  //Habilitar secuenciador 2 de ADC0.
    //ADC1 MOVIMIENTO VERTICAL
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);  //Habilitar ADC1.
    ADCSequenceConfigure(ADC1_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);  //Se configura secuenciador 2 de ADC1.
    ADCSequenceStepConfigure(ADC1_BASE, 2, 0, ADC_CTL_CH7);  //Se lee canal 7 --> PD0.
    ADCSequenceStepConfigure(ADC1_BASE, 2, 1, ADC_CTL_CH7);
    ADCSequenceStepConfigure(ADC1_BASE, 2, 2, ADC_CTL_CH7);
    ADCSequenceStepConfigure(ADC1_BASE, 2, 3, ADC_CTL_CH7|ADC_CTL_IE|ADC_CTL_END);  //Termina de realizar la última muestra del ADC1 y levanta un interrupción.
    ADCSequenceEnable(ADC1_BASE, 2);  //Habilitar secuenciador 2 de ADC1.
}
/*--------------------------------------------CONFIGURACIÓN PWM-----------------------------------------------*/
void PWM(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);  //Habilitar PWM0.
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);  //Reloj del módulo PWM.
    //PWM MOVIMIENTO HORIZONTAL
    GPIOPinConfigure(GPIO_PE4_M0PWM4);  //Pin PC4 como salida 6 de PWM0.
    //PWM MOVIMIENTO VERTICAL
    GPIOPinConfigure(GPIO_PE5_M0PWM5);  //Pin PC5 como salida 7 de PWM0.
}
void PWM_Init(void){
    T_cycles =  ((SysCtlClockGet()/64)/F_Hz) - 1;  //Periodo de la señal de PWM0.
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN);  //Tipo de señal de Generador 3 de PWM0.
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, T_cycles);  //Se establece el periodo de señal a Generador 3 de PWM0.
    //PWM MOVIMIENTO HORIZONTAL
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);  //Switch para activar/desactivar salida 6 de PWM0.
    //PWM MOVIMIENTO VERTICAL
    PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, true);  //Switch para activar/desactivar salida 7 de PWM0.

    PWMGenEnable(PWM0_BASE, PWM_GEN_2);  //Habilitar Generador 3 de modulo 1 PWM.
}
/*--------------------------------------------CONFIGURACION UART-----------------------------------------------*/
void Config_UART(void){
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0); //Habilitar Módulo 1 del periférico UART
        GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0|GPIO_PIN_1); //Configurar pines 0 y 1 de Puerto B como tipo UART
        GPIOPinConfigure(GPIO_PA0_U0RX); //Configurar PB0 como RX
        GPIOPinConfigure(GPIO_PA1_U0TX); //Configurar PB1 como TX
        UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE);
}
/*-------------------------------------------------MAIN--------------------------------------------------------*/
int main(void){
    SysCtlClockSet(SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ|SYSCTL_SYSDIV_5); //Reloj de Tiva C a 40MHz
    GPIO();
    //TIMER();
    ADC();
    PWM();
    PWM_Init();
    Config_UART();

    while(1){
     /*_____________________________________________SERVO HORIZONTAL____________________________________________*/
        ADCIntClear(ADC0_BASE, 2);
        ADCProcessorTrigger(ADC0_BASE, 2);

            ADCSequenceDataGet(ADC0_BASE,2,H_Sample);
            H_Average = (H_Sample[0] + H_Sample[1] + H_Sample[2] + H_Sample[3])/4;
            H_Angle = (0.01954*H_Average)+100;
                if (H_Angle > 179){
                    H_Pos = 7;
                }else{
                    H_Pos = (H_Angle*0.1) -10;
                }
            H_HighTime = (125*(H_Angle*4+180))/72;

        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, H_HighTime);
     /*_____________________________________________SERVO VERTICAL_______________________________________________*/
        ADCIntClear(ADC1_BASE, 2);
        ADCProcessorTrigger(ADC1_BASE, 2);

            ADCSequenceDataGet(ADC1_BASE,2,V_Sample);
            V_Average = ( V_Sample[0] +V_Sample[1] + V_Sample[2] + V_Sample[3])/4;
            V_Angle = (0.014652014*V_Average)+100;
                if (V_Angle > 179){
                    V_Pos = 7;
                }else{
                      V_Pos = 0.133333344*(V_Angle-100);
                }
            V_HighTime = (125*(V_Angle*4+180))/72;

        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, V_HighTime);
        /*_____________________________________________ENVIO DE DATOS POR UART_______________________________________________*/
                        for(i = 0; i < 10; i++) { //Enviar texto de "POSICION: "
                          UARTCharPut(UART0_BASE, TextOne[i]);
                            }
                          UARTCharPut(UART0_BASE, (65+(7 - H_Pos)));
                          UARTCharPut(UART0_BASE, (48+V_Pos));   //Equivalente en ASCII
                          UARTCharPut(UART0_BASE, '\n');       //Línea nueva
                          UARTCharPut(UART0_BASE, '\r');       //Retornar cursor
    }
}
