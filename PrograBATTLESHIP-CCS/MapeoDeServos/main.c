/**********************************LIBRERIAS DE C************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
/***********************************LIBRERIAS****************************************/
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"         //Para desbloquear pines
#include "driverlib/pin_map.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/uart.h"
//*********************************DEFINICIONES*************************************//
#define Frequency 50 //Se define la freq = 50Hz a la que trabaja el servo SG90.

//**********************************VARIABLES***************************************//
float PromedioPotH; //Variable que guarda el promedio de la muestra 1 y 2 tomadas por el ADC0.
float PromedioPotV; //Variable que guarda el promedio de la muestra 3 y 4 tomadas por el ADC0.
int PromedioPH;
int PromedioPV;

float DutyCycleServoH; //Variable que guarda el valor de ciclo de trabajo calculado para la señal PWM0.
float DutyCycleServoV; //Variable que guarda el valor de ciclo de trabajo calculado para la señal PWM0.

float AnchoPulsoH; //Variable que guarda el valor calculado del ancho de pulso para el PWM del servo horizontal.
float AnchoPulsoV; //Variable que guarda el valor calculado del ancho de pulso para el PWM del servo vertical.

float AnguloServoH; //Variable que guarda el valor calculado del ángulo que tiene el Servo horizontal.
float AnguloServoV; //Variable que guarda el valor calculado del ángulo que tiene el Servo vertical.
int AnguloSH;
int AnguloSV;

uint32_t SamplesADC0[4]; //Variable que guarda las 4 muestras tomadas por el ADC0.
uint32_t PWMFreq; //Variable que guarda el valor de freq. calculada para el módulo PWM.
uint32_t LoadCycles; //Variable que guarda el valor de carga calculado para periodo de la señal PWM a usar.

volatile uint8_t Dato;
char Buffer[4];
char PromedioH[] = "PromedioPotH: ";
char PromedioV[] = "PromedioPotV: ";
char AnguloH[] = "AnguloServoH: ";
char AnguloV[] = "AnguloServoV: ";

//********************************PROCEDIMIENTOS************************************//
void ConfigGPIO(void){
    //ADC --> POTENCIÓMETROS.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);  //Se habilita puerto D.
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1); //Se establece pin PD0 y PD1 como ADC.
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_ANALOG); //Se establece pin PD0 y PD1 como analógico.
    //PWM --> SERVOMOTORES.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); //Se habilita puerto E.
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4|GPIO_PIN_5); //Se establece pin PE4 y PE5 con PWM.
    GPIOPinConfigure(GPIO_PE4_M0PWM4); //Se establece la salida PWM correspondiente a pin PE4.
    GPIOPinConfigure(GPIO_PE5_M0PWM5); //Se establece la salida PWM correspondiente a pin PE5.
    //UART.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); //Se habilita el puerto A.
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0|GPIO_PIN_1); //Se establece pin PA0 y PA1 como UART.
    GPIOPinConfigure(GPIO_PA0_U0RX); //Se establece que el pin de UART0 RX = PA0.
    GPIOPinConfigure(GPIO_PA1_U0TX); //Se establece que el pin de UART0 TX = PA1.
    //INTERRUPCIONES.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //Se habilita el puerto F.
        //Desbloqueo de Pin F0
        HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
        HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
        HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0); //Se establece el pin PF0 como entrada.
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); //Se establece que el pin PF0 usará resistencia PULL-UP.
    IntMasterEnable(); //Se habiitan las interrupciones del periférico NVIC.
    IntEnable(INT_GPIOF); //Se habilitan las interrupciones por el puerto F.
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_0); //Se establece que PF0 generará la interrupción.
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_FALLING_EDGE); //Se establece que la interrupción será por flanco de bajada de PF0.
}

void ConfigPWM(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); //Se habilita módulo PWM 0.
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64); //Se establece la freq. a la que trabajará el PWM 0.

    PWMFreq = SysCtlClockGet()/64; //Se calcula la freq. a la que trabajará el PWM 0.
    LoadCycles = (PWMFreq/Frequency) - 1; //Se calcula la carga para el PWM0 = Periódo se la señal PWM a usar.

    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN); //Se configura el PWM 0, se establece generador a usar y tipo de señal.
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, LoadCycles); //Se establece la carga para el generador 2 del PWM 0.
    //PWM SERVO MOVIMIENTO HORIZONTAL.
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true); //Se habilita salida 4 del PWM 0 = PE4.
    //PWM SERVO MOVIMIENTO VERTICAL.
    PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, true); //Se habilita salida 5 del PWM 0 = PE5.

    PWMGenEnable(PWM0_BASE, PWM_GEN_2); //Se habilita(Activa) generador 2 del PWM 0.
}

void ConfigADC(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); //Se habilita módulo ADC 0.
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0); //Se configura el ADC0, se establece secuenciador a usar y tipo de disparo.
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH6); //Se establece la toma de 1er muestra --> Se lee canal 6 = PD1.
    ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_CH6); //Se establece la toma de 2da muestra --> Se lee canal 6 = PD1.
    ADCSequenceStepConfigure(ADC0_BASE, 2, 2, ADC_CTL_CH7); //Se establece la toma de 3er muestra --> Se lee canal 7 = PD0.
    ADCSequenceStepConfigure(ADC0_BASE, 2, 3, ADC_CTL_CH7|ADC_CTL_END|ADC_CTL_IE); //Se establece la toma de 4ta y última muestra, y se interrumpe --> Se lee canal 7 = PD0.
    ADCSequenceEnable(ADC0_BASE, 2); //Se habilita la secuencia del ADC0.
}

void ConfigUART(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0); //Se habilita módulo 0 de UART.
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE);
}

void SendData(int numero, char Texto[14]){
    int i = 0;

    for(i = 0; i < 14; i++){
        UARTCharPut(UART0_BASE, Texto[i]); //Se imprime el Texto correspondiente por UART.
    }
    i = 0;
    ltoa(numero, Buffer); //Se convierte el dato de tipo int a tipo char.
    for(i = 0; i < 4; i++){
        UARTCharPut(UART0_BASE, Buffer[i]); //Se imprime el número correspondiente por UART.
    }
    UARTCharPut(UART0_BASE, '\n'); //Se envía la instrucción de nueva línea.
    UARTCharPut(UART0_BASE, '\r'); //Se envía la instrucción de retornar el cursor.
    Buffer[0] = '\0';
    Buffer[1] = '\0';
    Buffer[2] = '\0';
    Buffer[3] = '\0';
}

void IntGPIOF(void){
    int PF0;
    PF0 = GPIOIntStatus(GPIO_PORTF_BASE, true); //Se lee y guarda que el pin PF0 interrumpió.
    GPIOIntClear(GPIO_PORTF_BASE, PF0); //Se limpia la interrupción hecha por PF0.

    SendData(PromedioPH, PromedioH);
    SendData(AnguloSH, AnguloH);
    SendData(PromedioPV, PromedioV);
    SendData(AnguloSV, AnguloV);
    UARTCharPut(UART0_BASE, '\n');  //Se envía la instrucción de nueva línea.
}

//***************************************MAIN********************************************//
int main(void){
    SysCtlClockSet(SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ|SYSCTL_SYSDIV_5); //Se establece freq. de TivaC a 40MHz.
    //Se hace llamada a métodos de configuraciones.
    ConfigGPIO();
    ConfigPWM();
    ConfigADC();
    ConfigUART();

    while(true){
        ADCIntClear(ADC0_BASE, 2); //Se limpia interrupción dada por secuenciador 2 del ADC 0.
        ADCProcessorTrigger(ADC0_BASE, 2); //Se dispara el secuenciador 2 del ADC 0.

            while(!ADCIntStatus(ADC0_BASE, 2, false)){ //Se verifica si hay nueva lectura en ADC0. Si no la hay, se permanece en el while.
            }

            ADCSequenceDataGet(ADC0_BASE, 2, SamplesADC0); //Se guardan las 4 muestras tomadas por el ADC0 en vector SamplesADC0.

            PromedioPotH = (SamplesADC0[0] + SamplesADC0[1])/2; //Se calcula el promedio entre la muestra 1 y 2.
                PromedioPH = (int)PromedioPotH;
            PromedioPotV = (SamplesADC0[2] + SamplesADC0[3])/2; //Se calcula el promedio entre la muestra 3 y 4.
                PromedioPV = (int)PromedioPotV;

            DutyCycleServoH = ((10*PromedioPotH)/4096) + 2.5; //Se calcula el ciclo de trabajo para la señal PWM del servo horizontal.
            DutyCycleServoV = ((10*PromedioPotV)/4096) + 2.5; //Se calcula el ciclo de trabajo para la señal PWM del servo vertical.

            AnchoPulsoH = (DutyCycleServoH/100)*LoadCycles; //Se calcula el ancho de pulso para determinar los grados de movimiento del servo horizontal.
            AnchoPulsoV = (DutyCycleServoV/100)*LoadCycles; //Se calcula el ancho de puslo para determinar los grados de movimiento del servo vertical.

            AnguloServoH = (180*(AnchoPulsoH - 312.5))/1250;
                AnguloSH = (int)AnguloServoH;
            AnguloServoV = (180*(AnchoPulsoV - 312.5))/1250;
                AnguloSV = (int)AnguloServoV;

            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, AnchoPulsoH); //Se establece el ancho de pulso de la señal PWM del servo horizontal.
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, AnchoPulsoV); //Se establece el ancho de pulso de la señal PWM del servo vertical.

            //SysCtlDelay((0.1*SysCtlClockGet())/3); //Se hace una pausa de 0.1 seg.
    }
}
