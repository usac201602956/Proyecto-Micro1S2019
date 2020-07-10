//******************************************* LIBRERIAS DE C **********************************************//
#include <stdint.h>
#include <stdbool.h>

//******************************************* LIBRERIAS TIVA C ********************************************//
#include "driverlib/sysctl.h"    //Para utilizar definiciones de periféricos.
#include "driverlib/gpio.h"      //Periféricos de entrada y salida
#include "inc/hw_gpio.h"         //Para desbloquear pines.
#include "inc/hw_memmap.h"       //Macros defining the memory map of the device.
#include "inc/hw_types.h"        //Common types and macros.
#include "inc/tm4c123gh6pm.h"    //TM4C123GH6PM Register Definitions.
#include "driverlib/pin_map.h"   //Mapping of peripherals to pins for all parts. Configurar pines de GPIO
#include "driverlib/interrupt.h" //Prototypes for the NVIC Interrupt Controller Driver. Para interrupciones
#include "driverlib/uart.h"      //Para protocolo UART
#include "driverlib/pwm.h"       //Para utilizar PWM.
#include "driverlib/adc.h"       //Para utilizar ADC.
#include "driverlib/timer.h"     //Para utilizar TIMER.

//******************************************** DEFINICIONES ***********************************************//

//Se define la frecuencia = 50Hz a la que trabajan los servos SG90.
#define ServoMotorFrequency 50
//Orden en que se colocarán los 4 barcos en el trabero del juego.
#define BigShip      0
#define MediumShip   1
#define LittleShip1  2
#define LittleShip2  3
//Cantidad de casillas que ocupa cada tamaño de barco que se puede colocar en el tablero.
#define BigShipSize     5
#define MediumShipSize  3
#define LittleShipSize  2
//Identificador para fila y columna.
#define Fila     'F'
#define Columna  'C'
//Maneras en que se puede recorrer el número de "Fila" en el tablero.
#define HaciaArriba  -1
#define HaciaAbajo    1
#define MantenerFila  0
//Maneras en que se puede recorrer el número de "Columna" en el tablero.
#define HaciaDerecha     1
#define HaciaIzquierda  -1
#define MantenerColumna  0
//Orden de las direcciones a la que apuntará la cruz de flechas.
#define Right 0
#define Up    1
#define Left  2
#define Down  3
//Valor del peso del bit necesario para encender cada flecha de la cruz de flechas.
#define ArrowRight  16  //2^4 --> peso de bit 4.
#define ArrowUp     32  //2^5 --> peso de bit 5.
#define ArrowLeft   64  //2^6 --> peso de bit 6.
#define ArrowDown   128 //2^7 --> peso de bit 7.
//Orden de los estados que manejará el juego.
#define Storage     0
#define SendData    1
#define ReceiveData 2
//Valor del peso del bit necesario para encender los Leds que indicarán el estado del juego.
#define LEDRojo  2 //2^1 --> peso de bit 1.
#define LEDAzul  4 //2^2 --> peso de bit 2.
#define LEDVerde 8 //2^3 --> peso de bit 3.
//Valor del peso del bit necesario para encender los Leds indicadores.
#define Rojo  128 //2^7 --> peso de bit 7.
#define Azul  64  //2^6 --> peso de bit 6.
//Valor de carga correspondiente al periodo deseado para la señal PWM que genera cada nota. Se utilizó la 7ma OCTAVA.
#define SI  158
#define SOL 199
#define DO  298
/* Valor del peso del bit necesario para poner en ALTO o BAJO las señales utilizadas en los registros 74LS164 que se
 * usaron para controlar la matriz de leds.
 */
#define ClockRegistroDeFilas     4 //2^2 --> peso de bit 2.
#define ClearRegistroDeFilas     8 //2^3 --> peso de bit 3.
#define DataRegistroDeColumnas   4 //2^2 --> peso de bit 2.
#define ClockRegistroDeColumnas  8 //2^3 --> peso de bit 3.

//******************************************** ESTRUCTURAS ************************************************//

//Estructura que contiene los estados de juego que maneja el juego Battleship en general.
struct Game{
    int ActualEstadoDeJuego;
    int LedDeEstadoDeJuego;
    int SigEstadoDeJuego;
    int PosibleEstadoDeJuego;
};

/* Estructura que contiene el tamaño del barco que se va a colocar en el tablero y el paso que sigue
 * después de almacenar dicho barco. Si falta colocar un barco, el siguiente paso es otro barco. Si ya
 * se terminó de colocar los 4 barcos, el siguiente paso es el estado de juego "ReceiveData".
 */
struct Ships{
    int TamañoDeBarco;
    int SiguientePaso;
};

/* Estructura que contiene las variables utilizadas en los calculos que se realizan en
 * los métodos de Space_Of_Ships, Intersection_Of_Ships y Storage_Of_Ships.
 */
struct Datos{
    int RecorrerFilaOColumna;
    int ComoRecorrerFila;
    int ComoRecorrerColumna;
    int QueFlechaEncender;
    int SigDireccionDeFlecha;
};

/* Estructura que contiene las variables que guardan los datos que se utilizan para colocar un barco
 * en el tablero en donde desee el jugador.
 */
struct Data_Of_Ships{
    int FilaApuntada;
    int ColumnaApuntada;
    int SeRecorrioFilaOColumna;
    int UltimaCasillaOcupada;
    int ComoSeRecorrioFila;
    int ComoSeRecorrioColumna;
    int OperacionSumaOResta;
};

typedef const struct Game  TipoGame;
typedef const struct Ships TipoShips;
typedef const struct Datos TipoDatos;

//Arreglo que guarda los datos utilizados en cada barco al colocarlo o leerlo en el tablero.
struct Data_Of_Ships BARCOS[4];

//************************************************* FSM *****************************************************//
TipoGame BATTLESHIP[3] = {{Storage,     LEDRojo,  SendData,    ReceiveData},  //EstadoDeJuego 0 = Colocar barcos.
                          {SendData,    LEDVerde, ReceiveData,     Storage},  //EstadoDeJuego 1 = Enviar datos al rival.
                          {ReceiveData, LEDAzul,  SendData,        Storage}}; //EstadoDeJuego 2 = Recibir datos del rival.

TipoShips BARCO_ALMACENAR[4] = {{BigShipSize,     MediumShip},  //BarcoQueSeColoca 0 = Barco grande de 5 casillas.
                                {MediumShipSize, LittleShip1},  //BarcoQueSeColoca 1 = Barco mediano de 3 casillas.
                                {LittleShipSize, LittleShip2},  //BarcoQueSeColoca 2 = Barco pequeño de 2 casillas.
                                {LittleShipSize, ReceiveData}}; //BarcoQueSeColoca 3 = Barco pequeño de 2 casillas.

TipoDatos ORIENTACION[4] = {{Columna, MantenerFila, HaciaDerecha,    ArrowRight,  Up},   //Dirección 0 = Colocar/Leer barco hacia derecha
                            {Fila,    HaciaArriba,  MantenerColumna, ArrowUp,   Left},   //Dirección 1 = Colocar/Leer barco hacia arriba
                            {Columna, MantenerFila, HaciaIzquierda,  ArrowLeft, Down},   //Dirección 2 = Colocar/Leer barco hacia izquierda
                            {Fila,    HaciaAbajo,   MantenerColumna, ArrowDown, Right}}; //Dirección 3 = Colocar/Leer barco hacia abajo

//********************************************* VARIABLES *************************************************//
float PromedioPotH, PromedioPotV; //Variables que guardan el promedio de la muestras tomadas por el ADC0.
float AnchoPulsoH, AnchoPulsoV;   //Variables que guardan el valor calculado del ancho de pulso para el PWM del servo mov. horizontal y vertical.
uint32_t SamplesADC0[4];          //Variable que guarda las 4 muestras tomadas por el ADC0.
uint32_t PWMFreq;                 //Variable que guarda el valor de frecuencia calculada para el módulo PWM.
uint32_t LoadCycles;              //Variable que guarda el valor de carga calculado para el periodo de la señal PWM a usar.
float PosicionH, PosicionV;       //Variables que guardan el valor de fila y columna en que se apunta con el laser al tablero.
int Row, Column;                  //Variables que guardan el valor entero de fila y columna en que se apunta con el laser al tablero.

//Matriz 8x8 que contiene el valor de ángulo que tiene el servo de movimiento horizontal en cada posición del tablero.
int AngulosDeServoH[8][8] = {{106,99,89,81,71,67,57,51},
                             {106,98,89,80,72,67,56,50},
                             {105,98,87,79,70,67,56,50},
                             {105,97,88,80,72,64,56,49},
                             {106,97,88,81,72,64,55,49},
                             {105,97,88,79,71,62,54,48},
                             {106,98,88,78,71,62,55,48},
                             {106,98,87,78,71,61,54,47}};

//Matriz 8x8 que contiene el valor de ángulo que tiene el servo de movimiento vertical en cada posición del tablero.
int AngulosDeServoV[8][8] = {{53,49,48,48,49,50,55,57},
                             {61,60,60,60,59,60,64,65},
                             {73,71,71,70,71,73,72,74},
                             {85,84,84,83,82,83,84,83},
                             {98,99,98,99,100,98,94,93},
                             {107,111,113,113,111,110,106,104},
                             {117,121,122,123,122,119,117,112},
                             {125,130,131,131,131,126,125,120}};

volatile uint8_t Dato, Bits;          //Variables para enviar datos por UART.
int Board[8][8];                      //Matriz que almacena los barcos colocados en el tablero.
int BarcoQueSeColoca = BigShip;       //Variable para hacer el cambio de estado en SM de BARCO_ALMACENAR.
int DireccionFlecha = Right;                //Variable para hacer el cambio de estado en SM de ORIENTACIÓN.
int Limite, ComoRecorrerF, ComoRecorrerC, QueRecorrer, SumaOResta; //Variables para la orientación del barco. Definen la característica de cada barco.
char DatoRecorrer;                                                 //Variable que almacena qué dato se recorrerá, ya sea Row o Column.

char FIFO[4] = "\0";          //Vector que guarda los caracteres que se reciben por UART con el Bluetooth.
int NumCaracterRecibido = 0;  //Variable que indica en qué posición de la FIFO almacenar el caracter recibido por UART.
int UltimoCaracter = 0;       //Variable que almacena la posición de la FIFO en que se guardó el último caracter recibido por UART.
int EstadoDeJuego = Storage;  //Variable para hacer el cambio de estado en SM de BATTLESHIP.
int DisparosBG = 0, DisparosBM = 0, DisparosBP1 = 0, DisparosBP2 = 0;  //Variables que cuentan los disparos que recibe cada barco.

bool HabilitarColumna = true; //Variable utilizada para habilitar y deshabilitar las columnas de la matriz de LEDs.

//******************************************* CONFIGURACIÓN GPIO **********************************************//
void GPIO(void){
    //Se habilitan los puertos A, B, C, D, E, F respectivamente.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //Se desbloquean los pines PF0 Y PD7.
    HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE+GPIO_O_CR) |= GPIO_PIN_0;

    HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE+GPIO_O_CR) |= GPIO_PIN_7;

    //Se establecen los pines PF0 y PF4 como entradas y con resistencias PULL-UP.
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_4); //PF0 = ENTER; PF4 = DIRECCIÓN.
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //Se establecen los pines PC4, PC5, PC6 y PC7; PD2, PD3 y PD6; PE1, PE2 y PE3; PF1, PF2 y PF3 como salidas respectivamente.
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7); //PC4; PC5; PC6; PC7 flechas.
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3);                       //PD2; PD3 matriz.
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6|GPIO_PIN_7);                       //PD6 = INDICADOR AZUL; PD7 = INDICADOR ROJO.
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);            //PE1; PE2; PE3 matriz.
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);            //PF1 = LED ROJO; PF2 = LED AZUL; PF3 = LED VERDE.

    //Se establece pines PB0 y PB1 como tipo UART; PD0 y PD1 como tipo ADC; PE4, PE5 y PA6 como tipo PWM respectivamente.
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1);
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_ANALOG); //PD0 y PD1 como pines analógicos.
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4|GPIO_PIN_5);
    GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6);

    //Se establece el pin de UART correspondiente a PB0 y PB1, y la salida de PWM correspondiente a pines PE4, PE5 y PA6 respectivamente.
    GPIOPinConfigure(GPIO_PB0_U1RX);   //PB0 = RX.
    GPIOPinConfigure(GPIO_PB1_U1TX);   //PB1 = TX.
    GPIOPinConfigure(GPIO_PE4_M0PWM4); //PE4 = SALIDA 4 DE PWM0 PARA SERVO...
    GPIOPinConfigure(GPIO_PE5_M0PWM5); //PE4 = SALIDA 5 DE PWM0 PARA SERVO...
    GPIOPinConfigure(GPIO_PA6_M1PWM2); //PA6 = SALIDA 2 DE PWM1 PARA BUZZER.
}

//******************************************* CONFIGURACIÓN ADC **********************************************//
void ADC(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);                   //Se habilita módulo ADC 0.
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0); //Se configura el ADC0, se establece secuenciador a usar y tipo de disparo.
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH6);       //Se establece la toma de 1er muestra --> Se lee canal 6 = PD1.
    ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_CH6);       //Se establece la toma de 2da muestra --> Se lee canal 6 = PD1.
    ADCSequenceStepConfigure(ADC0_BASE, 2, 2, ADC_CTL_CH7);       //Se establece la toma de 3er muestra --> Se lee canal 7 = PD0.
    ADCSequenceStepConfigure(ADC0_BASE, 2, 3, ADC_CTL_CH7|ADC_CTL_END|ADC_CTL_IE); //Se establece la toma de 4ta y última muestra, y se interrumpe --> Se lee canal 7 = PD0.
    ADCSequenceEnable(ADC0_BASE, 2);                                               //Se habilita la secuencia del ADC0.
}

//******************************************* CONFIGURACIÓN PWM **********************************************//
void PWM(void){
    //Se habilita el módulo 0 y 1 de PWM respectivamente.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

    //Se establece el reloj a usarse por los módulos PWM.
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    PWMFreq = SysCtlClockGet()/64;                  //Se calcula la freq. a la que trabajará el PWM 0.
    LoadCycles = (PWMFreq/ServoMotorFrequency) - 1; //Se calcula la carga para el PWM0 = Periódo se la señal PWM a usar.

    //Se establece el tipo de señal a usarse en cada módulo con su respectivo generador.
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);

    //Se establece el periodo de señal del Generador 2 de PWM0.
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, LoadCycles);

    //Se habilitan las salidas 4 y 5 del PWM0 respectivamente.
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, true);

    //Se habilitan los generadores de cada PWM. Para PWM0 generador 2, para PWM1 generador 1 respectivamente.
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
    PWMGenEnable(PWM1_BASE, PWM_GEN_1);
}

//******************************************* CONFIGURACIÓN UART **********************************************//
void UART(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1); //Habilitar Módulo 1 del periférico UART.
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 9600, UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE); //Cadena de 8 bits, 1 bit de parada, paridad par.
}

//******************************************* CONFIGURACIÓN NVIC **********************************************//
void NVIC(void){
    //Se habilitan las interrupciones del periférico NVIC.
    IntMasterEnable();
    //Se habilitan las interrupciones por puerto F y por módulo 1 de UART respectivamente.
    IntEnable(INT_GPIOF);
    IntEnable(INT_UART1);

    //Se establecen las prioridad de interrución del puerto F y UART1.
    IntPrioritySet(INT_GPIOF, 1);
    IntPrioritySet(INT_UART1, 0);

    //Se habilitan las interrupciones por PF0, PF4 y UART1. Se indica la forma en que PF0, PF4 y UART1 interrumpirán.
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_4);
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_4, GPIO_FALLING_EDGE); //PF0 y PF4 interrumpen por flanco de bajada.
    UARTIntEnable(UART1_BASE, UART_INT_RX|UART_INT_RT);                        //UART1 interrumpe por recepción y tiempo de recepción.
}

//******************************************* PROCEDIMIENTOS ***********************************************//

//Se enciende o apaga el buzzer con le nota que se le indique.
void Suena_Buzzer(int Nota, bool NotaSuena){
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, Nota);         //Se establece el periodo de señal del Generador 1 de PWM1
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, Nota/2);      //Se establece el ancho de pulso de la salida 2 del PWM1. D = 50%
    PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, NotaSuena); //Habilitar/Deshabilitar la salida 2 del PWM1.
}
//Se enciende o apaga en LED. Según el valor puede encender el led rojo o azul.
void Indicador(int ValorLEDS){
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6|GPIO_PIN_7, ValorLEDS); //Encender/Apagar LED azul o rojo.
}
/* Se enciende el led azul y el buzzer con la nota SI, y luego se apagan. Con esto se le indica al jugador que
 * acertó el disparo hecho al rival.
 */
void Acierto(void){
    Suena_Buzzer(SI, true);
    Indicador(Azul);
        SysCtlDelay(SysCtlClockGet()/3); //Retardo de 1s.
    Suena_Buzzer(SI, false);
    Indicador(0);
}
/* Se enciende el led rojo y el buzzer con la nota DO, y luego se apaga. Con esto se le indica al jugador que falló
 * el disparo hecho al rival o que hubo algún error en el juego.
 */
void Fallo_Error(void){
    Suena_Buzzer(DO, true);
    Indicador(Rojo);
        SysCtlDelay(SysCtlClockGet()/3); //Retardo de 1s.
    Suena_Buzzer(DO, false);
    Indicador(0);
}
/* Se enciede el led azul y el buzzer con la nota SI, y luego se apagan. Esto se repite cierta cantidad de veces,
 * como un blinking. Con esto se le indica al jugador que hundió un barco del rival.
 */
void Barco_Hundido(void){
    bool blink = true;
    int Conteo = 0;

    for(Conteo = 0; Conteo < 4; Conteo++){
        SysCtlDelay((0.5*SysCtlClockGet())/3);
        if(blink){
            Suena_Buzzer(SI, true);
            Indicador(Azul);
            blink = false;
        }else{
            Suena_Buzzer(SI, false);
            Indicador(0);
            blink = true;
        }
    }
}
/* Se intercala el encendido de los leds rojo y azul, y si intercala la nota con la que suena el buzzer entre SI y SOL.
 * Esto se repite cierta cantidad de veces. Con esto se le indica al jugador que gana la partida.
 */
void Ganar(void){
    bool blink = true;
    int Conteo = 0;

    for(Conteo = 0; Conteo < 4; Conteo++){
        SysCtlDelay((0.5*SysCtlClockGet())/3);
        if(blink){
            Suena_Buzzer(SI, true);
            Indicador(Azul);
            blink = false;
        }else{
            Suena_Buzzer(SOL, true);
            Indicador(Rojo);
        }
    }

    Suena_Buzzer(SOL, false);
    Indicador(0);
}
/* Se enciede el led rojo y el buzzer con la nota DO, y luego se apagan. Esto se repite cierta cantidad de veces,
 * como un blinking. Con esto se le indica al jugador que pierde la partida.
 */
void Perder(void){
    bool blink = true;
    int Conteo = 0;

    for(Conteo = 0; Conteo < 4; Conteo++){
        SysCtlDelay((0.5*SysCtlClockGet())/3);
        if(blink){
            Suena_Buzzer(DO, true);
            Indicador(Rojo);
            blink = false;
        }else{
            Suena_Buzzer(DO, false);
            Indicador(0);
            blink = true;
        }
    }
}

/* FUNCIÓN BOOLEANA PARA SABER SI EL BARCO CABE O NO CABE EN EL TABLERO:
 * Se verifica si el barco a colocar en el tablero cabe en las casillas que, según la dirección indicada por el jugador
 * y el tamaño del barco, se llenaran con 1. Para ello se verifica si la última casilla que ocupará el barco al colocarse
 * esta dentro del espacio del tablero o supera los límites del tablero de 8x8.
 * Si el barco no cabe se le avisa al jugador con el indicador rojo y un tono del buzzer.
 */
bool Space_Of_Ships(void){
    if((Limite - SumaOResta)  >= 0 & (Limite - SumaOResta) <= 7){
        return true;
    }else{
        Fallo_Error();
        return false;
    }
}

/* FUNCIÓN PARA SABER SI EL BARCO COMPARTE O NO CASILLAS CON OTRO BARCO:
 * Se verifica si el barco a colocar en el tablero, según la dirección indicada por el jugador y el tamaño del barco, se
 * intersecta con otro barco que ya ha sido colocado. Para esto, se verifica si las casillas a usarse para colocar el barco
 * ya tienen un 1 lo cual indica que coinciden con las casillas de otro barco. Basta que haya intersección en una sola casilla
 * para que el barco ya no se almacene.
 * Si el barco tiene intersección se le avisa al jugador con el indicador rojo y un tono del buzzer.
 */
bool Intersection_Of_Ships(void){
    int Contador = 0, Suma = 0;

    DatoRecorrer = ORIENTACION[DireccionFlecha].RecorrerFilaOColumna;

    if(DatoRecorrer == Fila){
        QueRecorrer = Row;
    }else{
        QueRecorrer = Column;
    }

    ComoRecorrerF = ORIENTACION[DireccionFlecha].ComoRecorrerFila;
    ComoRecorrerC = ORIENTACION[DireccionFlecha].ComoRecorrerColumna;

    for(Contador = QueRecorrer; Contador != Limite; Contador = Contador + SumaOResta){
        Suma = Suma + Board[Row][Column];   //Si Suma llega a ser igual o mayor a 1 es porque hay casillas ocupadas.
        Row = Row + ComoRecorrerF;
        Column = Column + ComoRecorrerC;
    }

    Row = (int)PosicionV;
    Column = 7 - (int)PosicionH;

    if (Suma > 0){
        Suma = 0;
        Fallo_Error();
        return false;
    }else{
        return true;
    }
}

/* COLOCACIÓN DE LOS BARCOS EN EL TABLERO DEL JUEGO:
 * Según la posición y dirección en que el jugador desea colocar el barco en el tablero, se determina entre el dato de Fila y Columna
 * cuál de los 2 se debe recorrer y cuál debe permanecer constante para ir colocando un 1 en las casillas de la matriz Board[][] que
 * ocupa el barco. Si el barco cabe en las casillas en donde se desea colocar y no se intersecta con ningún otro barco el barco se
 * colocará (true), de lo contrario el barco no se colocará (false).
 */
bool Storage_Of_Ships(void){
    int Contador = 0;

    DatoRecorrer = ORIENTACION[DireccionFlecha].RecorrerFilaOColumna;

    if(DatoRecorrer == Fila){
        QueRecorrer = Row;
    }else{
        QueRecorrer = Column;
    }

    SumaOResta = ORIENTACION[DireccionFlecha].ComoRecorrerFila + ORIENTACION[DireccionFlecha].ComoRecorrerColumna; //Se determina la operación de suma(1) o resta(-1).
    ComoRecorrerF = ORIENTACION[DireccionFlecha].ComoRecorrerFila;
    ComoRecorrerC = ORIENTACION[DireccionFlecha].ComoRecorrerColumna;
    Limite = QueRecorrer + ((BARCO_ALMACENAR[BarcoQueSeColoca].TamañoDeBarco)*SumaOResta); //Se calcula la casilla límite para almacenar el barco en el tablero.

    if(Space_Of_Ships() && Intersection_Of_Ships()){
        BARCOS[BarcoQueSeColoca].FilaApuntada = Row;
        BARCOS[BarcoQueSeColoca].ColumnaApuntada = Column;
        BARCOS[BarcoQueSeColoca].SeRecorrioFilaOColumna = QueRecorrer;
        BARCOS[BarcoQueSeColoca].UltimaCasillaOcupada = Limite;
        BARCOS[BarcoQueSeColoca].ComoSeRecorrioFila = ComoRecorrerF;
        BARCOS[BarcoQueSeColoca].ComoSeRecorrioColumna = ComoRecorrerC;
        BARCOS[BarcoQueSeColoca].OperacionSumaOResta = SumaOResta;

        for(Contador = QueRecorrer; Contador != Limite; Contador = Contador + SumaOResta){
            Board[Row][Column] = 1;
            Row = Row + ComoRecorrerF;
            Column = Column + ComoRecorrerC;
        }
        return true;
    }else{
        return false;
    }
}

/* FUNCIÓN PARA OBTENER A CUÁL DE MIS BARCOS LE DISPARÓ EL RIVAL:
 * Se solicita como parámetros el número de barco(NumBarco) a verificar, el número de fila(Fi) y columna(Co) en donde se recibió el disparo.
 * Se toman los datos que se utilizaron cuando se colocó el barco que se esta verificando y utilizando esos datos se verifica si la casilla en
 * donde se recibió el disparo del rival coincide con alguna de las casillas ocupadas por dicho barco. Si coincide el disparo con alguna
 * casilla del barco, se termina de verificar.
 */
bool GetBarco(int NumBarco, int FilaDisparada, int ColumnaDisparada){
    int Contador = 0, GetRow, GetColumn;

    GetRow = BARCOS[NumBarco].FilaApuntada;
    GetColumn = BARCOS[NumBarco].ColumnaApuntada;
    QueRecorrer = BARCOS[NumBarco].SeRecorrioFilaOColumna;
    Limite = BARCOS[NumBarco].UltimaCasillaOcupada;
    ComoRecorrerF = BARCOS[NumBarco].ComoSeRecorrioFila;
    ComoRecorrerC = BARCOS[NumBarco].ComoSeRecorrioColumna;
    SumaOResta = BARCOS[NumBarco].OperacionSumaOResta;

    for(Contador = QueRecorrer; Contador != Limite; Contador = Contador + SumaOResta){
        if(GetRow == FilaDisparada & GetColumn == ColumnaDisparada){
            break;
        }else{
            GetRow = GetRow + ComoRecorrerF;
            GetColumn = GetColumn + ComoRecorrerC;
        }
    }

    if(GetRow == FilaDisparada & GetColumn == ColumnaDisparada){
        return true;
    }else{
        return false;
    }
}

/* ESTADO DEL JUEGO EN QUE SE ENVIAN DATOS ---> SE DISPARA AL RIVAL:
 * Depués de que se realizó el disparo al rival, el juego espera a que el rival le responda. El recibir la respuesta del rival,
 * se toman los datos que se han recibido en la FIFO y se verifica si la respuesta del rival fué: A, F, HB o HTB.
 * Con ello se le indica al jugador por medio de un indicador LED (rojo o azul) y un tono del buzzer si acertó el disparo,
 * si falló el disparo, si hundió un barco o si hundió todos los barcos.
 * Si el jugador hundió todos los barcos se resetea el juego, reseteando todas las variables usadas y limpiando la matriz
 * Board[][] del juego.
 */
void Send_Data(void){
    char CaracterRecibido[4] = "\0"; //Vector en donde se copian los datos que llegaron a la FIFO.
    int fila = 0, columna = 0, Barco = 0;

    if(FIFO[UltimoCaracter] == 13 ){  //13 es el valor en ASCii de ENTER.
        NumCaracterRecibido = 0;
        UltimoCaracter = 0;
        CaracterRecibido[0] = FIFO[0];
        CaracterRecibido[1] = FIFO[1];
        CaracterRecibido[2] = FIFO[2];
        CaracterRecibido[3] = FIFO[3];
        //Se limpia el vector FIFO.
        FIFO[0] = '\0';
        FIFO[1] = '\0';
        FIFO[2] = '\0';
        FIFO[3] = '\0';

        if(CaracterRecibido[0] == 'A' && CaracterRecibido[1] == 13){
            Acierto();
                EstadoDeJuego = BATTLESHIP[EstadoDeJuego].SigEstadoDeJuego;
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, BATTLESHIP[EstadoDeJuego].LedDeEstadoDeJuego);
        }else if(CaracterRecibido[0] == 'F' && CaracterRecibido[1] == 13){
            Fallo_Error();
                EstadoDeJuego = BATTLESHIP[EstadoDeJuego].SigEstadoDeJuego;
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, BATTLESHIP[EstadoDeJuego].LedDeEstadoDeJuego);
        }else if(CaracterRecibido[0] == 'H' && CaracterRecibido[1] == 'B' && CaracterRecibido[2] == 13){
            Barco_Hundido();
                EstadoDeJuego = BATTLESHIP[EstadoDeJuego].SigEstadoDeJuego;
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, BATTLESHIP[EstadoDeJuego].LedDeEstadoDeJuego);
        }else if(CaracterRecibido[0] == 'H' && CaracterRecibido[1] == 'T' && CaracterRecibido[2] == 'B' && CaracterRecibido[3] == 13){
            Ganar();
                EstadoDeJuego = BATTLESHIP[EstadoDeJuego].PosibleEstadoDeJuego;  //Se vuelve al estado de juego "Storage".
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, BATTLESHIP[EstadoDeJuego].LedDeEstadoDeJuego);
            //Se limpia el tablero del juego.
            for(fila = 0; fila < 8; fila++){
                for(columna = 0; columna < 8; columna++){
                    Board[fila][columna] = 0;
                }
            }
            for(Barco = 0; Barco < 4; Barco++){
                BARCOS[Barco].FilaApuntada = 0;
                BARCOS[Barco].ColumnaApuntada = 0;
                BARCOS[Barco].SeRecorrioFilaOColumna = 0;
                BARCOS[Barco].UltimaCasillaOcupada = 0;
                BARCOS[Barco].ComoSeRecorrioFila = 0;
                BARCOS[Barco].ComoSeRecorrioColumna = 0;
                BARCOS[Barco].OperacionSumaOResta = 0;
            }
            BarcoQueSeColoca = BigShip;
            DireccionFlecha = Right;
            GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, ORIENTACION[DireccionFlecha].QueFlechaEncender); //Se enciende la flecha derecha de la cruz de flechas.
        }else{
        }
        UARTCharPut(UART1_BASE, '\n');  //Línea nueva.
        UARTCharPut(UART1_BASE, '\n');  //Línea nueva.
        UARTCharPut(UART1_BASE, '\r');  //Retornar cursor.
    }
}

/* ESTADO DEL JUEGO EN QUE SE RECIBEN DATOS ---> SE RECIBE DISPARO DEL RIVAL:
 * Se espera a que el rival dispare a una de nuestras casillas del tablero. Al recibir el disparo del rival, se toman
 * los datos que se han recibido en la FIFO y se determina a cuál de nuestros barcos le dió el rival. Para ello se busca
 * a cuál de nuestros barcos le pertenece la casilla donde se recibió el disparo.
 * Con ello se le indica al rival si acertó el disparo, si falló el disparo, si hundió un barco o si hundió todos los barcos
 * envíandole los caracteres A, F, HB o HTB.
 * Si el rival hundió todos los barcos se le indica al jugador por medio del indicador LED rojo y un tono del buzzer que perdió,
 * y se resetea el juego, reseteando todas las variables usadas y limpiando la matriz Board[][] del juego.
 */
void Receive_Data(void){
    int NumBarco = 0, FilaDisparada = 0, ColumnaDisparada = 0, fila = 0, columna = 0, Barco = 0, BarcoDisparado = -1;
    bool BarcoHundido = false;

    if(FIFO[2] == 13){          //13 es el valor en ASCii de ENTER.
        NumCaracterRecibido = 0;
        UltimoCaracter = 0;
        ColumnaDisparada = FIFO[0] - 65;
        FilaDisparada = FIFO[1] - 49;
        //Se limpia el vector FIFO.
        FIFO[0] = '\0';
        FIFO[1] = '\0';
        FIFO[2] = '\0';

        if(Board[FilaDisparada][ColumnaDisparada] == 1){
            for(NumBarco = 0; NumBarco < 4; NumBarco++){
                if(GetBarco(NumBarco, FilaDisparada, ColumnaDisparada)){ //Se pide que se busque a qué barco le pegó el rival.
                    BarcoDisparado = NumBarco;
                    break;
                }
            }
            Board[FilaDisparada][ColumnaDisparada] = 0; //Se elimina la casilla donde el barco recibió el disparo.
            //Se cuentan los disparos que recibe cada barco.
            switch(BarcoDisparado){
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
                UARTCharPut(UART1_BASE, '\n');  //Línea nueva.
                UARTCharPut(UART1_BASE, '\n');  //Línea nueva.
                UARTCharPut(UART1_BASE, '\r');  //Retornar cursor.
                //Se limpian los disparos contados por cada barco.
                DisparosBG = 0;
                DisparosBM = 0;
                DisparosBP1 = 0;
                DisparosBP2 = 0;
                Perder();
                BarcoHundido = false;
                EstadoDeJuego = BATTLESHIP[EstadoDeJuego].PosibleEstadoDeJuego;  //Se vuelve al estado de juego "Storage".
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, BATTLESHIP[EstadoDeJuego].LedDeEstadoDeJuego);
                //Se limpia el tablero del juego.
                for(fila = 0; fila < 8; fila++){
                    for(columna = 0; columna < 8; columna++){
                        Board[fila][columna] = 0;
                    }
                }
                for(Barco = 0; Barco < 4; Barco++){
                    BARCOS[Barco].FilaApuntada = 0;
                    BARCOS[Barco].ColumnaApuntada = 0;
                    BARCOS[Barco].SeRecorrioFilaOColumna = 0;
                    BARCOS[Barco].UltimaCasillaOcupada = 0;
                    BARCOS[Barco].ComoSeRecorrioFila = 0;
                    BARCOS[Barco].ComoSeRecorrioColumna = 0;
                    BARCOS[Barco].OperacionSumaOResta = 0;
                }
                BarcoQueSeColoca = BigShip;
                DireccionFlecha = Right;
                GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, ORIENTACION[DireccionFlecha].QueFlechaEncender); //Se enciende la flecha derecha de la cruz de flechas.
            }else if(BarcoHundido){
                UARTCharPut(UART1_BASE, '\n');  //Línea nueva.
                UARTCharPut(UART1_BASE, '\r');  //Retornar cursor.
                UARTCharPut(UART1_BASE, 'H');
                UARTCharPut(UART1_BASE, 'B');
                UARTCharPut(UART1_BASE, '\n');  //Línea nueva.
                UARTCharPut(UART1_BASE, '\n');  //Línea nueva.
                UARTCharPut(UART1_BASE, '\r');  //Retornar cursor.
                BarcoHundido = false;
                EstadoDeJuego = BATTLESHIP[EstadoDeJuego].SigEstadoDeJuego;
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, BATTLESHIP[EstadoDeJuego].LedDeEstadoDeJuego);
            }else{
                UARTCharPut(UART1_BASE, '\n');  //Línea nueva.
                UARTCharPut(UART1_BASE, '\r');  //Retornar cursor.
                UARTCharPut(UART1_BASE, 'A');
                UARTCharPut(UART1_BASE, '\n');  //Línea nueva.
                UARTCharPut(UART1_BASE, '\n');  //Línea nueva.
                UARTCharPut(UART1_BASE, '\r');  //Retornar cursor.
                EstadoDeJuego = BATTLESHIP[EstadoDeJuego].SigEstadoDeJuego;
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, BATTLESHIP[EstadoDeJuego].LedDeEstadoDeJuego);
            }

        }else{
            UARTCharPut(UART1_BASE, '\n');  //Línea nueva.
            UARTCharPut(UART1_BASE, '\r');  //Retornar cursor.
            UARTCharPut(UART1_BASE, 'F');
            UARTCharPut(UART1_BASE, '\n');  //Línea nueva.
            UARTCharPut(UART1_BASE, '\n');  //Línea nueva.
            UARTCharPut(UART1_BASE, '\r');  //Retornar cursor.
            EstadoDeJuego = BATTLESHIP[EstadoDeJuego].SigEstadoDeJuego;
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, BATTLESHIP[EstadoDeJuego].LedDeEstadoDeJuego);
        }
    }
}

/* MOVIMIENTO DE SERVOMOTORES:
 * Se toma los valores que tiene cada potenciómetro con el ADC y con ello se calcula La posición vertical, que determina
 * el número de fila, y la posición horizontal, que determina el número de columna, correspondiente a la casilla a la que
 * se apunta con el laser.
 * Con el número de fila y columna se determina el ángulo al que debe posicionarse cada servomotor, según el movimiento que
 * se haga con los potenciómetros, para así apuntar a la casilla deseada con el laser.
 */
void Servomotores_Move(){
    ADCIntClear(ADC0_BASE, 2);                     //Se limpia interrupción dada por secuenciador 2 del ADC 0.
    ADCProcessorTrigger(ADC0_BASE, 2);             //Se dispara el secuenciador 2 del ADC 0.
        //Se verifica si hay nueva lectura en ADC0. Si no la hay, se permanece en el while.
        while(!ADCIntStatus(ADC0_BASE, 2, false)){
        }

    ADCSequenceDataGet(ADC0_BASE, 2, SamplesADC0);      //Se guardan las 4 muestras tomadas por el ADC0 en vector SamplesADC0.
    PromedioPotH = (SamplesADC0[0] + SamplesADC0[1])/2;
    PromedioPotV = (SamplesADC0[2] + SamplesADC0[3])/2;
    PosicionV = (PromedioPotV*7)/4060;                  //Ecuación que detetermina el número de Fila (0 a 7) según se mueve el Pot de movimiento vertical.
    Row = (int)PosicionV;
    PosicionH = (PromedioPotH*7)/4060;                  //Ecuación que detetermina el número de Columna (0 al 7) según se mueve el Pot de movimiento horizontal.
    Column = 7 - (int)PosicionH;
    //Se calcula el ancho de pulso para cada PWM de cada Servomotor.
    AnchoPulsoH = ((1250*AngulosDeServoH[Row][Column])/180) + 312.5;
    AnchoPulsoV = ((1250*AngulosDeServoV[Row][Column])/180) + 312.5;

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, AnchoPulsoH); //Se establece el ancho de pulso de la señal PWM para el servo horizontal.
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, AnchoPulsoV); //Se establece el ancho de pulso de la señal PWM para el servo vertical.
}

//PARA EL CONTROL DE LA MATRIZ DE LEDS
//Se inicializa la Matriz.
void Start_Matriz(void){
    int i = 0;
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, ClearRegistroDeFilas);

    for(i = 0; i < 8; i++){
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, DataRegistroDeColumnas);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, DataRegistroDeColumnas|ClockRegistroDeColumnas);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
    }
}
//Reloj que controla el reloj del registro que guardan los bits a mostrar en la Matriz
void Clock_Matriz(void){
    if(HabilitarColumna){
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, ClockRegistroDeColumnas);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
        HabilitarColumna = false;
    }else{
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, DataRegistroDeColumnas);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, DataRegistroDeColumnas|ClockRegistroDeColumnas);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
    }
}
//Se pinta la Matriz.
void Print_Matriz(void){
    int f = 7, c = 0;

    for(c = 0; c < 8; c++){
        Clock_Matriz();
        for(f = 7; f > -1; f--){
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, (2*Board[f][c])|ClearRegistroDeFilas);
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, (2*Board[f][c])|ClockRegistroDeFilas|ClearRegistroDeFilas);
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, ClearRegistroDeFilas);
        }
        SysCtlDelay((0.001*SysCtlClockGet())/3);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, ClearRegistroDeFilas);
    }
    HabilitarColumna = true;
}

//************************************** RUTINAS DE INTERRUPCIÓN ******************************************//

/* INTERRUPCIÓN POR PF0 Y PF4 ---> AL PRESIONAR EL BOTÓN "ENTER" o "DIRECCIÓN":
 * Cuando se presiona alguno de los botones, ya sea ENTER o DIRECCIÓN, se lee cuál de los 2 botones se presionó y en base
 * a ese botón se determina que hacer por presionar dicho botón.
 * Si se presiona ENTER y el juego está en el estado "Storage" se coloca un barco en el tabler; si el juego está en el
 * estado "SendData se envía un disparo al rival; si el juego está en el estado "ReceiveData" no se hace nada. Si se
 * presiona DIRECCIÓN y el juego está en el estado "Storage" se cambia la dirección indicada por la cruz de flechas,
 * de lo contrario, en otro estado de juego no se hace nada.
 * >> PIN_0 = ENTER
 * >> PIN_4 = DIRECCIÓN.
 */
void BOTONES(void){
    SysCtlDelay((0.5*SysCtlClockGet())/3); //Retardo de 500ms.
    int ReadButton = 0;

    ReadButton = GPIOIntStatus(GPIO_PORTF_BASE, true); //Se lee si se interrumpio con PF0 o PF4.
    GPIOIntClear(GPIO_PORTF_BASE, ReadButton);         //Se limpia la interrupción por pulsador PF0 o PF4.

    if(ReadButton == GPIO_INT_PIN_0){
        switch(BATTLESHIP[EstadoDeJuego].ActualEstadoDeJuego){
        case Storage:
            if(Storage_Of_Ships()){  //Se verifica si el barco cupo en las casillas y si no se intersectó con otro barco en el tablero.

                if(BarcoQueSeColoca == LittleShip2){
                    EstadoDeJuego = BATTLESHIP[EstadoDeJuego].SigEstadoDeJuego;
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, BATTLESHIP[EstadoDeJuego].LedDeEstadoDeJuego);
                    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 0); //Se apagan LEDs de cruz de flechas.
                }else{
                    BarcoQueSeColoca = BARCO_ALMACENAR[BarcoQueSeColoca].SiguientePaso;
                }

            }else{
                BarcoQueSeColoca = BarcoQueSeColoca;
            }
            break;
        case ReceiveData:
            break;
        case SendData:
            UARTCharPut(UART1_BASE, (65 + Column)); //Envío de número de columna. Se suma 65 para enviar las letras de la A a la H.
            UARTCharPut(UART1_BASE, (49 + Row));    //Envío de número de fila. Se suma 49 para enviar los números del 1 al 8.
            UARTCharPut(UART1_BASE, '\n');          //Se envía la instruccón de "nueva línea".
            UARTCharPut(UART1_BASE, '\r');          //Se envía la instrucción de "retornar cursor".
            break;
        default:
            break;
        }
    }else{
        switch(BATTLESHIP[EstadoDeJuego].ActualEstadoDeJuego){
        case Storage:
            DireccionFlecha = ORIENTACION[DireccionFlecha].SigDireccionDeFlecha;
            GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, ORIENTACION[DireccionFlecha].QueFlechaEncender);
            break;
        case ReceiveData:
            break;
        case SendData:
            break;
        default:
            break;
        }
    }
}

//INTERRUPCIÓN POR UART ---> AL RECIBIR UN DATO POR BLUETOOTH.
void UARTInt(void){
    uint32_t Status;
    Status = UARTIntStatus(UART1_BASE, true);  //Se lee si se interrumpió por RX o RT.
    UARTIntClear(UART1_BASE, Status);          //Se limpia la interrupción por RX o RT.

    while(UARTCharsAvail(UART1_BASE)){
        Dato = UARTCharGet(UART1_BASE);       //Se guarda el dato recibido por UART.
        UARTCharPut(UART1_BASE, Dato);        //Se devuelve el dato recibido por UART.
        FIFO[NumCaracterRecibido] = Dato;
        UltimoCaracter = NumCaracterRecibido; //Se guarda la posición que ocupó el último caracter recibido.
        NumCaracterRecibido++;
    }
}

//SE INICIALIZA EL JUEGO.
void INICIO(void){
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, BATTLESHIP[EstadoDeJuego].LedDeEstadoDeJuego);              //Enciende el Led Rojo --> Estado de juego "Storage".
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, ORIENTACION[DireccionFlecha].QueFlechaEncender); //Enciende la flecha "Derecha" de la cruz de flechas.
    Start_Matriz();
}

//*********************************************** MAIN *****************************************************//
int main(void){
    SysCtlClockSet(SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ|SYSCTL_SYSDIV_5); //Reloj de Tiva C a 40MHz
    GPIO();
    ADC();
    PWM();
    UART();
    NVIC();
    INICIO();

    while(true){
        if(EstadoDeJuego == Storage){

        }else if(EstadoDeJuego == ReceiveData){
            Receive_Data();
        }else{
            Send_Data();
        }
        Servomotores_Move();  //Movimiento de ServoMotores.
        Print_Matriz();       //Se pinta la Matriz.
    }
}
