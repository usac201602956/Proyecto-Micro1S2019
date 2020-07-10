//*******************************************LIBRERIAS DE C**********************************************//
#include <stdint.h>
#include <stdbool.h>
//*******************************************LIBRERIAS TIVA C********************************************//
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"       //Periféricos de entrada y salida.
#include "inc/hw_gpio.h"          //Para desbloquear pines.
#include "inc/hw_memmap.h"        //Macros defining the memory map of the device.
#include "inc/hw_types.h"         //Common types and macros.
#include "inc/tm4c123gh6pm.h"     //TM4C123GH6PM Register Definitions.
#include "driverlib/pin_map.h"    //Mapping of peripherals to pins for all parts. Configurar pines de GPIO
#include "driverlib/interrupt.h"  //Prototypes for the NVIC Interrupt Controller Driver. Para interrupciones
#include "driverlib/uart.h"       //Para protocolo UART
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
//********************************************DEFINICIONES***********************************************//
#define F_Hz 50  //Frecuencia a la que trabajan los servomotores

#define ReceiveData 0
#define SendData    1

#define SI  632
#define SOL 797
#define DO  1194

//********************************************ESTRUCTURAS************************************************//
struct Juego{
    int ActualEstadoDeJuego;
    int SigEstadoDeJuego;
};

struct Ships2{
    int Dato_Row;
    int Dato_Column;
    int Dato_i;
    int Dato_Limit;
    int Dato_a;
    int Dato_b;
    int Dato_cte;
};

typedef const struct Juego TipoEstado0;
typedef const struct Ships2 TipoPrueba;
//*************************************************FSM*****************************************************//
TipoEstado0 BATTLESHIP[2] = {
                             {ReceiveData, SendData},
                             {SendData, ReceiveData}
                            };

TipoPrueba BARCOS[4] = {
                    {0, 0, 0, 5, 0, 1, 1},
                    {3, 2, 2, 5, 0, 1, 1},
                    {4, 6, 4, 6, 1, 0, 1},
                    {6, 2, 2, 4, 0, 1, 1}
                   };
//*********************************************VARIABLES*************************************************//
uint32_t T_cycles;
uint32_t V_Sample[4];                 //Vector que almacena las muestras de voltaje del mov. vertical.
uint32_t H_Sample[4];                 //Vector que almacena las muestras de voltaje del mov. horizontal.
uint32_t V_HighTime, H_HighTime = 0;  //Variables que almacenan el tiempo en alto del pulso de la señal PWM.
int H_Angle, V_Angle, H_Average, V_Average, H_Pos, V_Pos;

volatile uint8_t Dato;
char FIFO[4] = "\0";
int NumCaracterRecibido = 0;
int CaracterFin = 0;
int EstadoDeJuego = ReceiveData;
int Board[8][8] = {{1,1,1,1,1,0,0,0},
                   {0,0,0,0,0,0,0,0},
                   {0,0,0,0,0,0,0,0},
                   {0,0,1,1,1,0,0,0},
                   {0,0,0,0,0,0,1,0},
                   {0,0,0,0,0,0,1,0},
                   {0,0,1,1,0,0,0,0},
                   {0,0,0,0,0,0,0,0}};
int Row, Column, Limit, a, b, i, cte;  //Variables para la orientación del barco. Definen la característica de cada barco.
int DisparosBG = 0, DisparosBM = 0, DisparosBP1 = 0, DisparosBP2 = 0;
//*******************************************CONFIGURACIÓN GPIO**********************************************//
void GPIO(void){
    //PARA BOTONES.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);  //Habilitar puerto F.
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;            // Desbloquear PF0.
    GPIO_PORTF_CR_R = 0x0f;
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0);                                       //Pin PF0 como entrada.
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); //PF0 con resistencia PULL-UP.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);                    //Habilitar puerto A.
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3);  //Pines PA2 y PA3 como salidas.
    //PARA UART.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);              //Habilitar puerto B.
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1);  //Configurar pines PB0 y PB1 como tipo UART.
    GPIOPinConfigure(GPIO_PB0_U1RX);                          //Configurar pin PB0 como RX.
    GPIOPinConfigure(GPIO_PB1_U1TX);                          //Configurar pin PB1 como TX.
    //PARA ADC.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);                                                         //Habilitar puerto D.
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1);                                              //Pin PD0 y PD1 como tipo ADC.
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_ANALOG);  //PD0 y PD1 como pines analógicos.
    //PARA PWM.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);             //Habilitar puerto E.
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4|GPIO_PIN_5);  //Pin PE4 y PE5 como tipo PWM.
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);             //Pin PB4 como tipo PWM.
}
//*******************************************CONFIGURACIÓN ADC**********************************************//
void ADC(void){
    //ADC0 MOVIMIENTO HORIZONTAL.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);                //Habilitar ADC0.
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);  //Se configura secuenciador 2 de ADC0.
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH6);    //Se lee canal 6 --> PD1.
    ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_CH6);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 2, ADC_CTL_CH6);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 3, ADC_CTL_CH6|ADC_CTL_IE|ADC_CTL_END);  //Termina de realizar la última muestra del ADC0 y levanta un interrupción.
    ADCSequenceEnable(ADC0_BASE, 2);                                                //Habilitar secuenciador 2 de ADC0.
    //ADC1 MOVIMIENTO VERTICAL.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);                //Habilitar ADC1.
    ADCSequenceConfigure(ADC1_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);  //Se configura secuenciador 2 de ADC1.
    ADCSequenceStepConfigure(ADC1_BASE, 2, 0, ADC_CTL_CH7);    //Se lee canal 7 --> PD0.
    ADCSequenceStepConfigure(ADC1_BASE, 2, 1, ADC_CTL_CH7);
    ADCSequenceStepConfigure(ADC1_BASE, 2, 2, ADC_CTL_CH7);
    ADCSequenceStepConfigure(ADC1_BASE, 2, 3, ADC_CTL_CH7|ADC_CTL_IE|ADC_CTL_END);  //Termina de realizar la última muestra del ADC1 y levanta un interrupción.
    ADCSequenceEnable(ADC1_BASE, 2);                                                //Habilitar secuenciador 2 de ADC1.
}
//*******************************************CONFIGURACIÓN PWM**********************************************//
void PWM(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);  //Habilitar módulo 0 de PWM.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);  //Habilitar módulo 1 de PWM.

    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);                       //Reloj del módulo PWM.
    T_cycles =  (SysCtlClockGet()/32)/F_Hz;                    //Periodo de la señal de módulo 0 de PWM0.
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN);  //Tipo de señal de Generador 2 de PWM0.
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, T_cycles);           //Se establece el periodo de señal del Generador 2 de PWM0.
    //PWM MOVIMIENTO HORIZONTAL.
    GPIOPinConfigure(GPIO_PE4_M0PWM4);               //Pin PE4 como salida 4 de PWM0.
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);  //Switch para activar/desactivar salida 4 de PWM0.
    //PWM MOVIMIENTO VERTICAL.
    GPIOPinConfigure(GPIO_PE5_M0PWM5);               //Pin PE5 como salida 5 de PWM0.
    PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, true);  //Switch para activar/desactivar salida 5 de PWM0.
    //PWM BUZZER.
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);  //Tipo de señal de Generador 3 de PWM1.
    GPIOPinConfigure(GPIO_PF3_M1PWM7);               //Pin PF3 como salida 7 de PWM1.
    //GENERADORES
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);  //Habilitar Generador 2 de modulo 0 PWM.
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);  //Habilitar Generador 3 de modulo 1 PWM.
}
//*******************************************CONFIGURACIÓN UART**********************************************//
void UART(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);  //Habilitar Módulo 1 del periférico UART.
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 9600, UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE);  //Cadena de 8 bits, 1 bit de parada, paridad par.
}
//*******************************************CONFIGURACIÓN NVIC**********************************************//
void NVIC(void){
    IntMasterEnable();  //Habilitar interrupciones globales del periférico NVIC.

    IntEnable(INT_GPIOF);                                           //Habilitar las interrupciones del puerto F.
    IntPrioritySet(INT_GPIOF, 1);                                   //Prioridad de interrupción pora PF0.
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_0);                     //Habilitar las interrupciones por pin PF0.
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_FALLING_EDGE); //Configuración de las condiciones para que pin PF0 interrumpa.

    IntEnable(INT_UART1);                                //Habilitar las interrupciones del Módulo 1 de UART.
    IntPrioritySet(INT_UART1, 0);                        //Prioridad de interrupción para UART1.
    UARTIntEnable(UART1_BASE, UART_INT_RX|UART_INT_RT);  //Configuración de las condiciones para que UART1 interrumpa.
}
//*******************************************PROCEDIMIENTOS***********************************************//
//OBTENER A CUÁL DE MIS BARCOS LE DISPARÓ EL RIVAL.
bool GetBarco(int B, int Fi, int Co){
    int z = 0, GetRow, GetColumn;

    GetRow = BARCOS[B].Dato_Row;
    GetColumn = BARCOS[B].Dato_Column;
    i = BARCOS[B].Dato_i;
    Limit = BARCOS[B].Dato_Limit;
    a = BARCOS[B].Dato_a;
    b = BARCOS[B].Dato_b;
    cte = BARCOS[B].Dato_cte;

    for(z = i; z != Limit; z = z + cte){
        if(GetRow == Fi & GetColumn == Co){
            break;
        }else{
            GetRow = GetRow + a;
            GetColumn = GetColumn + b;
        }
    }

    if(GetRow == Fi & GetColumn == Co){
        return true;
    }else{
        return false;
    }
}
//ESTADO DEL JUEGO EN QUE SE ENVIAN DATOS ---> SE DISPARA AL RIVAL.
void Send_Data(void){
    char Caracter[4] = "\0";
    bool blink = true;;
    int t = 0, u = 0;

    if(FIFO[CaracterFin] == ' ' ){
        NumCaracterRecibido = 0;
        CaracterFin = 0;
        Caracter[0] = FIFO[0];
        Caracter[1] = FIFO[1];
        Caracter[2] = FIFO[2];
        Caracter[3] = FIFO[3];
        FIFO[0] = '\0';
        FIFO[1] = '\0';
        FIFO[2] = '\0';
        FIFO[3] = '\0';

        if(Caracter[0] == 'A' && Caracter[1] == ' ' && Caracter[2] == '\0' && Caracter[3] == '\0'){
            PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, SI);                                    //Se establece el periodo de señal del Generador 3 de PWM1.
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, SI/2);                                 //Se establece el ancho de pulso del Generador 3 de PWM1. D = 50%
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_2);
            PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);                               //Activar salida 7 de PWM1 con nota SI.
            SysCtlDelay((SysCtlClockGet())/3);  //Retardo de 1s.
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, 0);
            PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, false);                               //Desactivar salida 7 de PWM1.
        }else if(Caracter[0] == 'F' && Caracter[1] == ' ' && Caracter[2] == '\0' && Caracter[3] == '\0'){
            PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, DO);                                    //Se establece el periodo de señal del Generador 3 de PWM1.
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, DO/2);                                 //Se establece el ancho de pulso del Generador 3 de PWM1. D = 50%
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_3);
            PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);                               //Activar salida 7 de PWM1 con nota DO.
            SysCtlDelay((SysCtlClockGet())/3);  //Retardo de 1s.
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, 0);
            PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, false);                               //Desactivar salida 7 de PWM1.
        }else if(Caracter[0] == 'H' && Caracter[1] == 'B' && Caracter[2] == ' ' && Caracter[3] == '\0'){
            PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, SI);                                    //Se establece el periodo de señal del Generador 3 de PWM1.
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, SI/2);                                 //Se establece el ancho de pulso del Generador 3 de PWM1. D = 50%
            for(t = 0; t < 6; t++){
                SysCtlDelay((0.5*SysCtlClockGet())/3);  //Retardo de 0.5s.
                if(blink){
                    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_2);
                    PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, blink);                               //Activar salida 7 de PWM1 con nota SI.
                    blink = false;
                }else{
                    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, 0);
                    PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, blink);                               //Desactivar salida 7 de PWM1.
                    blink = true;
                }
            }
        }else if(Caracter[0] == 'H' && Caracter[1] == 'T' && Caracter[2] == 'B' && Caracter[3] == ' '){
            for(u = 0; u < 6; u++){
                SysCtlDelay((0.5*SysCtlClockGet())/3);  //Retardo de 0.5s.
                if(blink){
                    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, SI);                                    //Se establece el periodo de señal del Generador 3 de PWM1.
                    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, SI/2);                                 //Se establece el ancho de pulso del Generador 3 de PWM1. D = 50%
                    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_2);
                    PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);                               //Activar salida 7 de PWM1 con nota SI.
                    blink = false;
                }else{
                    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, SOL);                                    //Se establece el periodo de señal del Generador 3 de PWM1.
                    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, SOL/2);                                 //Se establece el ancho de pulso del Generador 3 de PWM1. D = 50%
                    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_3);
                    PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);                                //Activar salida 7 de PWM1 con nota SOL.
                    blink = true;
                }
            }
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, 0);
            PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, false);                     //Desactivar salida 7 de PWM1.
        }else{
        }
        UARTCharPut(UART1_BASE, '\n');  //Línea nueva.
        UARTCharPut(UART1_BASE, '\r');  //Retornar cursor.
        UARTCharPut(UART1_BASE, 48 + EstadoDeJuego);
        UARTCharPut(UART1_BASE, ' ');  //Espacio.
        EstadoDeJuego = BATTLESHIP[EstadoDeJuego].SigEstadoDeJuego;
        UARTCharPut(UART1_BASE, 48 + EstadoDeJuego);
        UARTCharPut(UART1_BASE, '\n');  //Línea nueva.
        UARTCharPut(UART1_BASE, '\r');  //Retornar cursor.
    }
}
//ESTADO DEL JUEGO EN QUE SE RECIBEN LOS DATOS ---> SE RECIBE DISPARO DEL RIVAL.
void Receive_Data(void){
    int n = 0, f = 0, c = 0, ElBarco = -1;
    bool BarcoHundido = false;

    if(FIFO[2] == ' '){
        NumCaracterRecibido = 0;
        CaracterFin = 0;
        f = FIFO[0] - 65;
        c = FIFO[1] - 48;
        FIFO[0] = '\0';
        FIFO[1] = '\0';
        FIFO[2] = '\0';

        if(Board[f][c] == 1){
            for(n = 0; n < 4; n++){
                if(GetBarco(n, f, c)){
                    ElBarco = n;
                    break;
                }
            }
            //Board[f][c] = 0;

            switch(ElBarco){
                case 0:
                    DisparosBG++;
                    if(DisparosBG == 5){
                        BarcoHundido = true;
                    }
                    break;
                case 1:
                    DisparosBM++;
                    if(DisparosBM == 3){
                        BarcoHundido = true;
                    }
                    break;
                case 2:
                    DisparosBP1++;
                    if(DisparosBP1 == 2){
                        BarcoHundido = true;
                    }
                    break;
                case 3:
                    DisparosBP2++;
                    if(DisparosBP2 == 2){
                        BarcoHundido = true;
                    }
                    break;
                default:
                    break;
            }

            if((DisparosBG + DisparosBM + DisparosBP1 + DisparosBP2) == 12){
                UARTCharPut(UART1_BASE, '\n');  //Línea nueva.
                UARTCharPut(UART1_BASE, '\r');  //Retornar cursor.
                UARTCharPut(UART1_BASE, 'H');
                UARTCharPut(UART1_BASE, 'T');
                UARTCharPut(UART1_BASE, 'B');
                UARTCharPut(UART1_BASE, ' ');  //espacio
                UARTCharPut(UART1_BASE, 48 + EstadoDeJuego);
                UARTCharPut(UART1_BASE, ' ');  //espacio
                EstadoDeJuego = BATTLESHIP[EstadoDeJuego].SigEstadoDeJuego;
                UARTCharPut(UART1_BASE, 48 + EstadoDeJuego);
                UARTCharPut(UART1_BASE, '\n');  //Línea nueva.
                UARTCharPut(UART1_BASE, '\r');  //Retornar cursor.
                DisparosBG = 0;
                DisparosBM = 0;
                DisparosBP1 = 0;
                DisparosBP2 = 0;
                BarcoHundido = false;
            }else if(BarcoHundido){
                UARTCharPut(UART1_BASE, '\n');  //Línea nueva.
                UARTCharPut(UART1_BASE, '\r');  //Retornar cursor.
                UARTCharPut(UART1_BASE, 'H');
                UARTCharPut(UART1_BASE, 'B');
                UARTCharPut(UART1_BASE, ' ');  //espacio
                UARTCharPut(UART1_BASE, 48 + EstadoDeJuego);
                UARTCharPut(UART1_BASE, ' ');  //espacio
                EstadoDeJuego = BATTLESHIP[EstadoDeJuego].SigEstadoDeJuego;
                UARTCharPut(UART1_BASE, 48 + EstadoDeJuego);
                UARTCharPut(UART1_BASE, '\n');  //Línea nueva.
                UARTCharPut(UART1_BASE, '\r');  //Retornar cursor.
                BarcoHundido = false;
            }else{
                UARTCharPut(UART1_BASE, '\n');  //Línea nueva.
                UARTCharPut(UART1_BASE, '\r');  //Retornar cursor.
                UARTCharPut(UART1_BASE, 'A');
                UARTCharPut(UART1_BASE, ' ');  //espacio
                UARTCharPut(UART1_BASE, 48 + EstadoDeJuego);
                UARTCharPut(UART1_BASE, ' ');  //espacio
                EstadoDeJuego = BATTLESHIP[EstadoDeJuego].SigEstadoDeJuego;
                UARTCharPut(UART1_BASE, 48 + EstadoDeJuego);
                UARTCharPut(UART1_BASE, '\n');  //Línea nueva.
                UARTCharPut(UART1_BASE, '\r');  //Retornar cursor.
            }

        }else{
            UARTCharPut(UART1_BASE, '\n');  //Línea nueva.
            UARTCharPut(UART1_BASE, '\r');  //Retornar cursor.
            UARTCharPut(UART1_BASE, 'F');
            UARTCharPut(UART1_BASE, ' ');  //espacio
            UARTCharPut(UART1_BASE, 48 + EstadoDeJuego);
            UARTCharPut(UART1_BASE, ' ');  //espacio
            EstadoDeJuego = BATTLESHIP[EstadoDeJuego].SigEstadoDeJuego;
            UARTCharPut(UART1_BASE, 48 + EstadoDeJuego);
            UARTCharPut(UART1_BASE, '\n');  //Línea nueva.
            UARTCharPut(UART1_BASE, '\r');  //Retornar cursor.
        }
    }
}
//**************************************RUTINAS DE INTERRUPCIÓN******************************************//
//INTERRUPCIÓN POR PF0 ---> AL PRESIONAR EL BOTÓN "ENTER".
void BOTONES(void){
    SysCtlDelay((0.3*SysCtlClockGet())/3);  //Retardo de 300ms.
    int ReadPin = 0;
    ReadPin = GPIOIntStatus(GPIO_PORTF_BASE, true);  //Se lee si se interrupió por pulsador PF0 o PF4.
    GPIOIntClear(GPIO_PORTF_BASE, ReadPin);          //Se limpia la interrupción por pulsador PF0.

    switch(BATTLESHIP[EstadoDeJuego].ActualEstadoDeJuego){
        case ReceiveData:
            break;
        case SendData:
            UARTCharPut(UART1_BASE, (65 + Row));
            UARTCharPut(UART1_BASE, (48 + Column));  //Envío de número. Columna
            UARTCharPut(UART1_BASE, '\n');           //Línea nueva.
            UARTCharPut(UART1_BASE, '\r');           //Retornar cursor.
            break;
    }
}
//INTERRUPCIÓN POR UART ---> AL RECIBIR UN DATO POR BLUETOOTH.
void UARTInt(void){
    uint32_t Status;
    Status = UARTIntStatus(UART1_BASE, true);  //Se lee si se interrumpió por RX o RT.
    UARTIntClear(UART1_BASE, Status);          //Se limpia la interrupción por RX o RT.

    while(UARTCharsAvail(UART1_BASE)){
        Dato = UARTCharGetNonBlocking(UART1_BASE);  //Se almacena el dato recibido por UART.
    }
    UARTCharPut(UART1_BASE, Dato);  //Se devuelve el dato recibido por UART.
    FIFO[NumCaracterRecibido] = Dato;
    CaracterFin = NumCaracterRecibido;
    NumCaracterRecibido++;
}
//***********************************************MAIN*****************************************************//
int main(void){
    SysCtlClockSet(SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ|SYSCTL_SYSDIV_5); //Reloj de Tiva C a 40MHz.
    GPIO();
    ADC();
    PWM();
    UART();
    NVIC();

    while(1){
        if(EstadoDeJuego == ReceiveData){
            Receive_Data();
        }else{
            Send_Data();
        }
        /*_________________________________________SERVO HORIZONTAL_________________________________________*/
        ADCIntClear(ADC0_BASE, 2);
        ADCProcessorTrigger(ADC0_BASE, 2);

            ADCSequenceDataGet(ADC0_BASE,2,H_Sample);
            H_Average = (H_Sample[0] + H_Sample[1] + H_Sample[2] + H_Sample[3])/4;
            H_Angle = (0.01954*H_Average)+100;
                if (H_Angle > 179){
                    H_Pos = 7;
                    Column = 7 - H_Pos;
                }else{
                    H_Pos = (H_Angle*0.1) -10;
                    Column = 7 - H_Pos;
                }
            H_HighTime = (125*(H_Angle*4+180))/72;

        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, H_HighTime);
        /*_________________________________________SERVO VERTICAL___________________________________________*/
        ADCIntClear(ADC1_BASE, 2);
        ADCProcessorTrigger(ADC1_BASE, 2);

            ADCSequenceDataGet(ADC1_BASE,2,V_Sample);
            V_Average = ( V_Sample[0] +V_Sample[1] + V_Sample[2] + V_Sample[3])/4;
            V_Angle = (0.014652014*V_Average)+100;
                if (V_Angle > 179){
                    V_Pos = 7;
                    Row = V_Pos;
                }else{
                      V_Pos = 0.133333344*(V_Angle-100);
                      Row = V_Pos;
                }
            V_HighTime = (125*(V_Angle*4+180))/72;

        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, V_HighTime);
    }
}
