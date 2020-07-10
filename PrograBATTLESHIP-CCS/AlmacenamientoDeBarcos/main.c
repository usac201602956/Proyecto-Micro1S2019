//**************************************LIBRERIAS DE C***************************************//
#include <stdint.h>
#include <stdbool.h>
//*************************************LIBRERIAS TIVA C**************************************//
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"  //Periféricos de entrada y salida
#include "inc/hw_gpio.h"  //Para desbloquear pines.
#include "inc/hw_memmap.h" //Macros defining the memory map of the device.
#include "inc/hw_types.h"  //Common types and macros.
#include "inc/tm4c123gh6pm.h"  //TM4C123GH6PM Register Definitions.
#include "driverlib/pin_map.h"  //Mapping of peripherals to pins for all parts. Configurar pines de GPIO
#include "driverlib/interrupt.h"  //Prototypes for the NVIC Interrupt Controller Driver. Para interrupciones
#include "driverlib/uart.h"  //Para protocolo UART
//**************************************DEFINICIONES*****************************************//
#define BigShip      0
#define MediumShip   1
#define LittleShip1  2
#define LittleShip2  3

#define BigShipSize     5
#define MediumShipSize  3
#define LittleShipSize  2

#define Fila     'F'
#define Columna  'C'

#define Right  0
#define Up     1
#define Left   2
#define Down   3

#define LEDRight  16
#define LEDUp     32
#define LEDLeft   64
#define LEDDown   128

#define A 0
#define B 1
#define C 2
#define D 3
#define E 4
#define F 5
#define G 6
#define H 7
//***************************************VARIABLES*******************************************//
volatile uint8_t Dato, Bits;  //Variables para enviar datos por UART.
int Row, Column, Button = 0;  //Variables para almacenar los barcos.
int Board[8][8];              //Matriz que almacena(coloca) los barcos en el tablero.
int BarcoQueSeAlmacena;       //Variable para hacer el cambio de estados en SM de Ships1.
int Direccion;                //Variable para hacer el cambio de estados en SM de Data.
int Limit, a, b, i, cte;      //Variables para la orientación del barco. Definen la característica de cada barco.
char DatoRecorrer;            //Variable que almacena qué dato se recorrerá, ya sea Row o Column.
int PosH, PosV;               //Variables que almacenan la casilla que apunta el laser.
//****************************************MÉTODOS********************************************//
void Position(void);
bool Space_Of_Ships(void);
bool Intersection_Of_Ships(void);
void Storage_Of_Ships(void);
//**************************************ESTRUCTURAS******************************************//
struct Ships1{  //Se establece la características de los barcos.
    int Size; //Tamaño del barco.
    int Siguiente;  //El barco siguiente.
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

struct Data{
    char Recorrer;
    int ValorDeCteA;
    int ValorDeCteB;
    int SigOrientacion;
    int LED;
};

typedef const struct Ships1 TipoEstado1;
typedef const struct Data   TipoEstado2;

struct Ships2 BARCOS[4];

TipoEstado1 BARCO_ALMACENAR[4] = {
                                  {BigShipSize,     MediumShip},  //BarcoQueSeAlmacena 0 = Barco grande de 5 casillas.
                                  {MediumShipSize, LittleShip1},  //BarcoQueSeAlmacena 1 = Barco mediano de 3 casillas.
                                  {LittleShipSize, LittleShip2},  //BarcoQueSeAlmacena 2 = Barco pequeño de 2 casillas.
                                  {LittleShipSize,     BigShip}   //BarcoQueSeAlmacena 3 = Barco pequeño de 2 casillas.
                                 };

TipoEstado2 ORIENTACION[4] = {
                              {Columna,  0,  1, Up,   LEDRight},  //Dirección 0 = Almacenar/Leer hacia derecha
                              {Fila,    -1,  0, Left,    LEDUp},  //Dirección 1 = Almacenar/Leer hacia arriba
                              {Columna,  0, -1, Down,  LEDLeft},  //Dirección 2 = Almacenar/Leer hacia izquierda
                              {Fila,     1,  0, Right, LEDDown}   //Dirección 3 = Almacenar/Leer hacia abajo
                             };
//********************************RUTINAS DE INTERRUPCIÓN************************************//
void ENTER(void){
    int r = 0, c = 0, a = 0;
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_0); //Se limpia la interrupción por pulsador PF0.

    Storage_Of_Ships();
    if(Space_Of_Ships() && Intersection_Of_Ships()){  //Se verifica si el barco cupo en las casillas y si no se intersectó con otro barco en el tablero.
        BarcoQueSeAlmacena = BARCO_ALMACENAR[BarcoQueSeAlmacena].Siguiente; //Se pasa a pedir que se almacene el siguiente barco.

        for(r = 0; r < 8; r++){  //Se recorre la Matriz "Board" y se pinta por UART.
            for(c = 0; c < 8; c++){
                Bits = 48 + Board[r][c];
                UARTCharPut(UART1_BASE, Bits); //Pinta el bit correspondiente.
            }
            UARTCharPut(UART1_BASE, '\n'); //Línea nueva.
            UARTCharPut(UART1_BASE, '\r'); //Retornar cursor.
        }
        UARTCharPut(UART1_BASE, '\n'); //Línea nueva.
        UARTCharPut(UART1_BASE, '\r'); //Retornar cursor.

        for(a = 0; a < 4; a++){
            UARTCharPut(UART1_BASE, 48 + BARCOS[a].Dato_Row);
            UARTCharPut(UART1_BASE, ' '); //espacio.
            UARTCharPut(UART1_BASE, 48 + BARCOS[a].Dato_Column);
            UARTCharPut(UART1_BASE, ' '); //espacio.
            UARTCharPut(UART1_BASE, 48 + BARCOS[a].Dato_i);
            UARTCharPut(UART1_BASE, ' '); //espacio.
            UARTCharPut(UART1_BASE, 48 + BARCOS[a].Dato_Limit);
            UARTCharPut(UART1_BASE, ' '); //espacio.
            UARTCharPut(UART1_BASE, 48 + BARCOS[a].Dato_a);
            UARTCharPut(UART1_BASE, ' '); //espacio.
            UARTCharPut(UART1_BASE, 48 + BARCOS[a].Dato_b);
            UARTCharPut(UART1_BASE, ' '); //espacio.
            UARTCharPut(UART1_BASE, 48 + BARCOS[a].Dato_cte);
            UARTCharPut(UART1_BASE, '\n'); //Línea nueva.
            UARTCharPut(UART1_BASE, '\r'); //Retornar cursor.
        }
        UARTCharPut(UART1_BASE, '\n'); //Línea nueva.*/

    }else{
        BarcoQueSeAlmacena = BarcoQueSeAlmacena;  //Se continua pediendo que se almacena el barco actual.
    }
}

void UARTInt(void){
    uint32_t Status;
    Status = UARTIntStatus(UART1_BASE, true);
    UARTIntClear(UART1_BASE, Status);  //Se limpia la interrupción por UART.

    if(UARTCharsAvail(UART1_BASE)){
        Dato = UARTCharGet(UART1_BASE);  //Se almacena el dato que el micro recibe por UART.
        UARTCharPut(UART1_BASE, Dato);  //Se devuelve el dato que micro recibió por UART.
        UARTCharPut(UART1_BASE, '\n'); //Línea nueva.
        UARTCharPut(UART1_BASE, '\r'); //Retornar cursor.
    }

    Position();
}
//*************************************PROCEDIMIENTOS****************************************//
void GPIO(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);  //Habilitar puerto F.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);  //Habilitar puerto C.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);  //Habilitar puerto D.
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7); //Pines 4, 5, 6 y 7 del puerto C como salidas.
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6); //Pin 6 del puerto D como salida.

    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_4);
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;  // Desbloquear PF0.
    GPIO_PORTF_CR_R = 0x0f;
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); //PF0 y PF4 con resistencias PULL-UP.
}

void UART(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);  //Habilitar Módulo 1 del periférico UART.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);  //Habilitar puerto B.
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1);  //Configurar pines PB0 y PB1 como tipo UART.
    GPIOPinConfigure(GPIO_PB0_U1RX);  //Configurar pin PB0 como RX.
    GPIOPinConfigure(GPIO_PB1_U1TX);  //Configurar pin PB1 como TX.
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 9600, UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE);  //Cadena de 8 bits, 1 bit de parada, paridad par.
}

void NVIC(void){
    IntMasterEnable();  //Habilitar interrupciones globales del periférico NVIC.
    IntEnable(INT_GPIOF);  //Habilitar las interrupciones del puerto F.
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_0);  //Habilitar las interrupciones por pin PF0.
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_FALLING_EDGE); //Configuración de las condiciones para que pin PF0 interrumpan.
    IntEnable(INT_UART1);  //Habilitar las interrupciones del Módulo 0.
    UARTIntEnable(UART1_BASE, UART_INT_RX|UART_INT_RT);  //Configuración de las condiciones para que UART0 interrumpa.
    IntPrioritySet(INT_GPIOF, 1);
    IntPrioritySet(INT_UART1, 0);
}

void Position(void){  //Se establece la casilla del laser según la letra recibida por UART.
    if(Dato == 'A'){
        PosV = 0;
        PosH = A;
        Column = PosV;
        Row = PosH;
    }else if(Dato == 'S'){
        PosV = 2;
        PosH = D;
        Column = PosV;
        Row = PosH;
    }else if(Dato == 'D'){
        PosV = 6;
        PosH = E;
        Column = PosV;
        Row = PosH;
    }else if(Dato == 'F'){
        PosV = 2;
        PosH = G;
        Column = PosV;
        Row = PosH;
    }
}

bool Space_Of_Ships(void){  //Se verifica si el barco tiene espacio para poder almacenarse en el tablero.
    if(Limit >= 0 & Limit <= 7){ //Se verifica si la última casilla del barco cabe en el tablero.
        return true;
    }else{
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 64);  //Pin PD6 = 1.
        SysCtlDelay((0.3*SysCtlClockGet())/3);  //Retardo de aprox 0.3 seg.
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0);  //Pin PD6 = 0.
        return false;
    }
}

bool Intersection_Of_Ships(void){  //Se verifica si el barco a almacenarse se intersecta con otro ya almacenado en el tablero.
    int n = 0, sum = 0;

    DatoRecorrer = ORIENTACION[Direccion].Recorrer;  //Se almacena qué dato se recorrerá.
    if(DatoRecorrer == Fila){ //Se determina si la variable a recorrer es la de Fila o Columna.
        i = Row;
    }else{
        i = Column;
    }

    a = ORIENTACION[Direccion].ValorDeCteA;  //Valor que determina si Row se mantiene cte o variara.
    b = ORIENTACION[Direccion].ValorDeCteB;  //Valor que determina si Column se mantien cte o variara.

    for(n = i; n != Limit; n = n + cte){  //Se leen las casillas donde se quiere almacenar el barco.
        sum = sum + Board[Row][Column]; //Sum determina si una casilla ya esta ocupada por otro barco.
        Row = Row + a;
        Column = Column + b;
    }

    Row = PosH;
    Column = PosV;

    if(sum > 0){  //Se verifica si las casillas del tablero que almacenarán el barco estan vacías.
        sum = 0;
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 64);  //Pin PD6 = 1.
        SysCtlDelay((0.2*SysCtlClockGet())/3);  //Retardo de aprox 0.2 seg.
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0);  //Pin PD6 = 0.
        return false;
    }else{
        return true;
    }
}

void Storage_Of_Ships(void){  //Se almacena el barco en el tablero según la orientación escogida.
    int x = 0;

    DatoRecorrer = ORIENTACION[Direccion].Recorrer;  //Se almacena qué dato se recorrerá.
    if(DatoRecorrer == Fila){  //Se determina si la variable a recorrer es la de Fila o Columna.
        i = Row;
    }else{
        i = Column;
    }
    cte = ORIENTACION[Direccion].ValorDeCteA + ORIENTACION[Direccion].ValorDeCteB; //Constante que determina la operación de suma o resta.
    a = ORIENTACION[Direccion].ValorDeCteA;  //Valor que determina si Row se mantiene cte o variara.
    b = ORIENTACION[Direccion].ValorDeCteB;  //Valor que determina si Column se mantien cte o variara.

    Limit = i + ((BARCO_ALMACENAR[BarcoQueSeAlmacena].Size)*cte);  //Se calcula la casilla límite para almacenar el barco en el tablero.
    if(Space_Of_Ships() && Intersection_Of_Ships()){  //Se verifica si el barco cupo en las casillas y si no se intersectó con otro barco en el tablero.
        BARCOS[BarcoQueSeAlmacena].Dato_Row = Row;
        BARCOS[BarcoQueSeAlmacena].Dato_Column = Column;
        BARCOS[BarcoQueSeAlmacena].Dato_i = i;
        BARCOS[BarcoQueSeAlmacena].Dato_Limit = Limit;
        BARCOS[BarcoQueSeAlmacena].Dato_a = a;
        BARCOS[BarcoQueSeAlmacena].Dato_b = b;
        BARCOS[BarcoQueSeAlmacena].Dato_cte = cte;
        for(x = i; x != Limit; x = x + cte){ //Se escribe(coloca) el barco en la Matriz "Board" o tablero del juego.
            Board[Row][Column] = 1;
            Row = Row + a;
            Column = Column + b;
        }
    }
}
//*****************************************MAIN***********************************************//
int main(void){
    SysCtlClockSet(SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ|SYSCTL_SYSDIV_2_5); //Reloj de Tiva C a 80MHz.
    GPIO();
    UART();
    NVIC();
    BarcoQueSeAlmacena = BigShip;
    Direccion = Right;

    while(1){
        Button = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4);  //Se lee el valor que toma el pulsador PF4.
        if (Button == 0){
            SysCtlDelay((0.3*SysCtlClockGet())/3);
            Direccion = ORIENTACION[Direccion].SigOrientacion;  //Se cambia la orientación en que se almacenará el barco.
        }else{
            Direccion = Direccion;  //Se mantiene la orientación en que se almacenará el barco.
        }
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, ORIENTACION[Direccion].LED);  //LEDs de orientación
    }
}
