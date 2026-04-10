//BIBLIOTECAS
#include <stdint.h>
#include <stdbool.h>
#include "driverlib/sysctl.h"
#include "driverlib/rom_map.h"
#include "inc/tm4c1294ncpdt.h"

/*****************************************************************************************************************************************
--------------------------------------------------- NOTAS IMPORTANTES ----------------------------------------------

  Pinout
    *PN4 TRIGGER
    *PN5 ECO
    *PN0 RELÉ para Bomba de LLenado
    *PD4 MOTOR
    *PN3 RELÉ para Bomba de Vaciado

    *PE4 Entrada del ADC PE4
    *PB4 Emergencia


******************************************************************************************************************************************/
// VARIABLES GLOBALES DEFINICIONES Y PROTOTIPOS

// Definiciones para LLENADO
#define RELAY_PIN0 (1U << 0)  // Pin 0 del puerto N (PN0) ->Pulso para Relé Llenado
#define TRIGGER_PIN (1U << 4) // Pin 4 del puerto N (PN4) -> Pulso que activa Sensor Sonico -> SE UTILIZA EN TODOS LOS CICLOS
#define ECHO_PIN (1U << 5)    // Pin 5 del puerto N (PN5) -> Pulso de Rebote para el Sensor Sonico -> SE UTILIZA EN TODOS LOS CICLOS.

// Variables globales LLENADO
uint32_t ui32Distancia;
uint32_t ui32Loop; // Retardo para el Timer 3 (MOTOR)
uint32_t g_ui32SysClock;
uint32_t ui32LoopS;  // Retardo para el Timer 1 (SENSOR)


// Prototipos de funciones
void PORTN_INI(void); // Función para inicializar el puerto N
void TIMER_INI(void); // Función para inicializar el timer 3
void TRIGGER_04(void); // Función para generar el trigger de 10 µs

void TIMER_INI_SENSOR(void);// Timer para el Sensor


void Ciclo_Sensado_Llenado(void); //Funcion para ejecutar el ciclo de LLenado tal cual
void CicloLavado(void); //Funcion PAra ejecutar Lavado con movimiento de Servomotor

void LlenadoManual(void); //Para Realizar llenado de tina al pulsar un boton

// Definiciones para LAVADO
//#define SERVO_PIN (1U << 4)   // Pin 4 del puerto D (PD4) -> Salida Para Servomotor


// Prototipos de funciones LAVADO
void PORTD_INI(void); // Función para inicializar el puerto D (Servo)
void MOVER_SERVO(void); // Función para Mover el servomotor


// Definiciones para VACIADO
#define RELAY_PIN3 (1U << 3)   // Pin 3 del puerto N (PN3) ->Pulso para Relé Vaciado

// Variables globales VACIADO (Mismas que en Llenado)

// Prototipos de funciones VACIADO
void CicloVaciado(void);
void VaciadoManual(void);

//Prototipo de funciones ENJUAGADO
void CicloEnjuagado(void);

//Prototipo Funciones Mostrar Mensajes
void MostrarMensajeCiclo1(void);
void MostrarMensajeCiclo2(void);
void MostrarMensajeCiclo3(void);
void MostrarMensajeCiclo4(void);
void MostrarMensajeCiclo5(void);
void MostrarMensajeError1(void);
void MostrarMensajeExito(void);

//Variables Globales Rutina de Emergencia Limites excedidos del Voltaje
int time;
float result;
float v;
float VOLTAJE;
int delay;
float R1=1000;
float R2=1000;

// Función que mide el Voltaje
void MedicionVoltaje(void);
void IniVoltaje(void);

// INTERRUPCIONES

// Rutina de interrupción Emergencia    SE ACTIVA CUANDO EL RANGO DEL VOLTAJE SE SALE DE LOS LIMITES   V DEBE SER   4.5 < V < 6.0
extern void ADC0Seq3(void) {

    // Leer el valor del ADC (12 bits, máximo de 4095)
   result=(ADC0_SSFIFO3_R&0xFFF);   // Obtener el dato del FIFO

   // Limpiar la bandera de la interrupción para SS3
   //ADC0_ISC_R = 0x08;  // Limpiar la interrupción del Secuenciador 3 (SS3)
   ADC0_ISC_R = 8;                 // Limpiar la interrupción en SS3

    // Convertir el valor del ADC a voltaje
    v=(result * 3.3) / 4096; // Conversión con referencia de 3.3V y 12 bits
    VOLTAJE=(v/(R2/(R1+R2)));

    // Verificar rango de voltaje y controlar LED
    if (VOLTAJE < 4.5 || VOLTAJE > 6.0) {
        GPIO_PORTB_AHB_DATA_R |= 0x10; // Encender LED en PB4
    } else {
        GPIO_PORTB_AHB_DATA_R &= ~0x10; // Apagar LED

    }
}


/*****************************************************************************************************************************************
                                                PARA RETARDOS DE TIEMPO
*****************************************************************************************************************************************/
uint32_t LoopLavado;
uint32_t ui32SysClkFreq=16000000;//si definimos la frecuencia del reloj con SysCtlClockFreqSet cambiar este valor de 16 MHz
void delayUs(uint32_t Us);
void delayMs(uint32_t Ms);

void delayUs(uint32_t Us) { //funcion para lograr un retardo de tantos microsegundos
    SysCtlDelay(Us*(ui32SysClkFreq/3000000.0));//3/1000000;
}
void delayMs(uint32_t Ms) { //funcion para lograr un retardo de tantos mili
    SysCtlDelay(Ms*(ui32SysClkFreq/3000.0));//3/1000;
}

uint8_t   contador;
//unsigned char

int Fila,Columna,val,a, valtec;
char charTecla;

  //                       76543210
 //bitset(portc,2)         ********
//                         *****1**
//  00000001
//  00000010
//  00000100
 //                        76543210
 //bitset(portc,2)         ********
//
// principal, subrutinas, rutinas de interrupcion, tablas

uint8_t Valcon,Temp,Temp2,i;
  //bitset (PORTCON,2)                                PORTCON  0000 0000
#define bitset(Var,bitno) ((Var) |= 1 << (bitno)) // |         0000 0100 <-0
#define bitclr(Var,bitno) ((Var) &= ~(1 << (bitno)))  //&      1111 1011


/////////////////////////////////////////////     Arreglos de Mensajes Especificos      ////////////////////////////////////////////////////

static const uint8_t MensajeCiclo1[] = "Iniciando Ciclo";
static const uint8_t Mensaje3Puntos[] = "...";

static const uint8_t MensajeCiclo1_2[] = "1: Llenado";
static const uint8_t MensajeCiclo2[] = "2: Lavado";
static const uint8_t MensajeCiclo3[] = "3: Enjuagado";
static const uint8_t MensajeCiclo4[] = "4: Vaciado";
static const uint8_t MensajeVaciado1[] = "Ponga Manguera";
static const uint8_t MensajeVaciado2[] = " Presione 5";

static const uint8_t MensajeExito1[] = "Ciclo Completo.";
static const uint8_t MensajeExito2[] = "Retira Tu Ropa.";

static const uint8_t MensajeError1[] = "Ingresa Opcion Correcta";
static const uint8_t MensajeError2[] = "No Hay Agua";
static const uint8_t MensajeError3[] = "No Hay Agua";
static const uint8_t MensajeError3_1[] = "Seleccione Otra Opcion";
/*
static const uint8_t Alfa []= {0x00,0x00,0x09,0x15,0x012,0x12,0x0D,0x00};
static const uint8_t Beta []= {0x00,0x00,0x0E,0x11,0x011,0x0A,0x1B,0x00};
static const uint8_t carfe []= {0x00,0x00,0x0a,0x00,0x00e,0x0A,0x04,0x00};
*/
/////Lectura de datos en teclado matricial////
int filas[] = {0x0E, 0x0D, 0x0B, 0x07};
int columnas[] = {0x0E, 0x0D, 0x0B, 0x07};
char teclas[4][4] ={{'1','2','3','A'},{'4','5','6','B'},{'7','8','9','C'},{'*','0','#','D'}};

#define PORTDIS GPIO_PORTK_DATA_R
#define PORTCON GPIO_PORTM_DATA_R

//bits de control en PORTM

#define RS   0x00
#define R_W  0x01
#define E    0x02

#define BIT5 0X20
#define BIT6 0X40
#define BIT7 0X80

#define DISP_ON 0X0f  //1 1 1 1
#define CLR_DISP 0x01
#define HOME 0X02

void INIPORTS (void);
void INILCD(void);
void BUSY (void);
void _E(void);
void ESCDAT (uint8_t c);
void ESCCON (uint8_t d);
void CLS(void);
void BYTEDIS(uint8_t DByte); // 0x4e  4->ascii e->ascii  4e
//void HOME(void);
void AND491(void);
void LEECON(void);

//teclado
void keyboard_INI(void);
void LeerTeclas(void);


///////////////////////////////////////////////////////////// MAIN //////////////////////////////////////////////////////////

int main (void) {

    INIPORTS();  //Inicializa Los Puertos
    IniVoltaje();     // Inicializa Los Timers y Puertos para Medir el voltaje
    INILCD(); //inicializa el display 16 x 2

    MedicionVoltaje(); // Funcion para Medir el Voltaje

    GPIO_PORTN_DATA_R |= RELAY_PIN0; // APAGA relé
    GPIO_PORTN_DATA_R |= RELAY_PIN3; // APAGA relé

    //1. Muestra Menu de Seleccion

       ESCCON(0x01); //Limpia Pantalla y regresa cursor al Inicio

       ESCDAT('-');
       ESCDAT('S');
       ESCDAT('E');
       ESCDAT('L');
       ESCDAT('E');
       ESCDAT('C');
       ESCDAT('C');
       ESCDAT('I');
       ESCDAT('O');
       ESCDAT('N');
       ESCDAT('E');
       ESCDAT(':');

       ESCCON(0xC0); //Cambia al Segundo Renglon

       ESCDAT('1');
       ESCDAT(':');
       ESCDAT('F'); //Fill / Llenar

       ESCDAT(' ');

       ESCDAT('2');
       ESCDAT(':');
       ESCDAT('W'); //Wash / Lavar + Vaciar

       ESCDAT(' ');

       ESCDAT('3');
       ESCDAT(':');
       ESCDAT('R'); // Rinse / Llenar + Enjuagar

       ESCDAT(' ');

       ESCDAT('4');
       ESCDAT(':');
       ESCDAT('E'); // Empty / Vaciar


       ESCCON(0x02); //Cambia a Posicion Inicial

       ESCCON(0x8c); //Recorrer Cursor 12 espacios

       //El siguienter bloque recorre el cursor 8 espacios.
/*
    for(contador=0;contador<=8;contador++)

        {ESCCON(0B00011000);
            SysCtlDelay(2666666);   //t*3*n  @16MHz-> t=62.5nS *3=187.5nS   .5/187.5e-9

           // ESCCON(0B00011100);
             //           SysCtlDelay(2666666);
        }
*/

// 2. Se ingresa y se lee la opcion

        SysCtlDelay(4000000);
        ESCCON(0x11);  //Recorre Cursor 11 Espacios y Escribir

        while(1){

                 LeerTeclas();

               if (charTecla!=0xff)

               {
                   valtec=charTecla;
                   ESCDAT(valtec);

                   //Seleccion de Ciclo.


                   if (charTecla == 49) { //Al presionar la Tecla 1 en teclado Matricial
                       //Ciclo 1: Llenado Y Lavado
                        MostrarMensajeCiclo1();
                        SysCtlDelay(4000000);
                        Ciclo_Sensado_Llenado(); //Ejecuta codigo Para medir nivel de agua y Llenar Tina
                        MedicionVoltaje(); // Funcion para Medir el Voltaje

                        MostrarMensajeCiclo2();
                        SysCtlDelay(4000000);
                        CicloLavado(); //Ejecuta codigo para Lavar Ropa Con la Tina Llena.
                        MedicionVoltaje(); // Funcion para Medir el Voltaje

                        MostrarMensajeCiclo3();
                        SysCtlDelay(4000000);
                        CicloEnjuagado();
                        MedicionVoltaje(); // Funcion para Medir el Voltaje

                        MostrarMensajeCiclo4();
                        SysCtlDelay(4000000);
                        CicloVaciado();
                        MedicionVoltaje(); // Funcion para Medir el Voltaje

                        MostrarMensajeExito();
                        //Insertar mensaje de Exito
                        SysCtlDelay(36750000);
                        main();

                   } else if (charTecla == 50) { //Al presionar Tecla 2 en Teclado MAtricial

                       MostrarMensajeCiclo2();
                       SysCtlDelay(4000000);
                       CicloLavado(); //Ejecuta codigo para Lavar Ropa Con la Tina Llena.
                       MedicionVoltaje(); // Funcion para Medir el Voltaje

                       MostrarMensajeCiclo3();
                       SysCtlDelay(4000000);
                       CicloEnjuagado();
                       MedicionVoltaje(); // Funcion para Medir el Voltaje

                       MostrarMensajeCiclo4();
                       SysCtlDelay(4000000);
                       CicloVaciado();
                       MedicionVoltaje(); // Funcion para Medir el Voltaje

                       MostrarMensajeExito();
                       //Insertar mensaje de Exito
                       SysCtlDelay(36750000);
                       main();

                   } else if (charTecla == 51) { //Al Presionar tecla 3 en Teclado MAtricial

                       MostrarMensajeCiclo3();
                       SysCtlDelay(4000000);
                       CicloEnjuagado();
                       MedicionVoltaje(); // Funcion para Medir el Voltaje

                       MostrarMensajeCiclo4();
                       SysCtlDelay(4000000);
                       CicloVaciado();
                       MedicionVoltaje(); // Funcion para Medir el Voltaje

                       MostrarMensajeExito();
                       //Insertar mensaje de Exito
                       SysCtlDelay(36750000);
                       main();

                   }else if (charTecla == 52) { //Al Presionar Tecla 4 en Teclado MAtricial

                       MostrarMensajeCiclo4();
                       SysCtlDelay(4000000);
                       CicloVaciado();
                       MedicionVoltaje(); // Funcion para Medir el Voltaje

                       MostrarMensajeExito();
                       //Insertar mensaje de Exito
                       SysCtlDelay(36750000);
                       main();


                   } else {
                       MostrarMensajeError1();
                       SysCtlDelay(4000000);
                       main();

                   }

    }
  }
}

///////////////////////////////////////////////////////////RUTINAS Y FUNCIONES/////////////////////////////////////////////////////////////////////////
void INIPORTS(void){

      //FUNCIONES QUE INICIALIZAN PUERTOS NECESARIOS
      TIMER_INI();
      TIMER_INI_SENSOR();

      PORTN_INI();
      PORTD_INI();

      // Q P N  M L K J  H G F E  D C B A
     //  1 X X  1 X 1 X  1 X X 1  X X X X

      SYSCTL_RCGCGPIO_R |= 0X4E80;  //
       while ((SYSCTL_PRGPIO_R&0x4E80)!=0x4E80);
       //              1 0 0 0 0 0 0 1 0 0 0 0 0 0 0

       //HAbilitando Registro de Datos
       GPIO_PORTK_DATA_R=0X00; //Todos Los Pines en Nivel Bajo (0)
       GPIO_PORTM_DATA_R=0X00;
       GPIO_PORTH_AHB_DATA_R=0X00;
      // GPIO_PORTL_DATA_R=0X00;

       //Habilitando Funcion Digital
       GPIO_PORTK_DEN_R|=0XFF;  //Todos Los Pines
       GPIO_PORTM_DEN_R|=0X07;  //Habilita PM0, PM1 y PM2
       GPIO_PORTH_AHB_DEN_R|=0XFF; //Todos Los Pines
       GPIO_PORTQ_DEN_R|=0XFF; //Todos Los Pines
      // GPIO_PORTL_DEN_R=0XFF; //Todos Los Pines

       //Configurando Direcciones
       GPIO_PORTK_DIR_R|=0Xff;  //Todos en PK Como Salidas
       GPIO_PORTM_DIR_R|=0x07;  // M0 - M2 como Salidas
       GPIO_PORTH_AHB_DIR_R=0X0f; // PH0 - PH3 como Salidas
       GPIO_PORTQ_DIR_R=0X00; // Todos en PQ Como Entradas
      // GPIO_PORTL_DIR_R=0Xff; //Todos en PA Como Salidas

      GPIO_PORTQ_PUR_R=0X0F; // Habilita Resistencias Pull Up en PQ0 - PQ3
}

void INILCD(void){  //INICIALIZA EL DISPLAY A DOS LINEAS, 8 BITS DE INTERFAZ, CURSOR Y DESTELLO.
    //PRIMER METODO DE INICIALIZACION


    SysCtlDelay(1000000);                 //Delay 200ms
    PORTDIS=0X38; // **D7-D4 = 0011
    _E();                               //toggle E
    SysCtlDelay (540000);                   //10ms
    _E();                               //toggle E
    SysCtlDelay (540000);                   //10ms
    PORTDIS=0X38;
    _E();                               //toggle E
    SysCtlDelay (540000);                   //10ms
    ESCCON(0X38);   //8 BITS , 2 LINEAS, 5X7

    ESCCON(DISP_ON);  // ENCIENDE DISPLAY CON CURSOR Y DESTELLO


    ESCCON(CLR_DISP); // BORRAR DISPLAY

    ESCCON(0X06); // AL ESCRIBIR EL CURSOR SE INCREMENTA Y SE DESPLAZA
    ESCCON(0X38); // //8 BITS , 2 LINEAS, 5X7 REQUERIDO POR LOS DISPLAYS CLONES
    }

void _E(void){  //GENERA UN PULSO DE 450 nS EN LA TERMINAL E DEL DISPLAY

    bitset(PORTCON,E);        // E=1
    SysCtlDelay (3);  //retraso de 562.5nS *2
    bitclr(PORTCON,E);  // E=0
}

void ESCDAT (uint8_t c){  //ESCRIBE UN DATO ALFANUMERICO AL DISPLAY

    //ENVIA dato
    PORTDIS = c;
    bitclr(PORTCON,R_W);
    bitset(PORTCON,RS);    //ENVIA A REGISTRO DE DATOS
    _E();//TOGGLE _E()
    bitclr(PORTCON,RS);    //niveles de control a 0
    BUSY(); //PREGUNTA POR LA BANDERA DE BUSY

 }

//       E  R/W  RS
//       0   0    0

void ESCCON (uint8_t d)// ENVIA UN COMANDO AL REGISTRO DE CONTROL DEL LCD
{

    PORTDIS = d;               //envia CMD al LCD
    bitclr(PORTCON,R_W);
    bitclr(PORTCON,RS);               //direcciona registro de controldel  LCD
    _E();                           //toggle E
    BUSY(); //VERIFICA LA BANDERA DE BUSY
    }



void CLS(void)
{ ESCCON(0x01);
    // AND491( );
        }

/*void HOME(void)
 { ESCCON(0x02);
    // AND491( );
   }
*/

void AND491(void)  // GENERA LOS CORRIMIENTOS PARA ALINEAR COMO EN
    {   ESCCON (0X1C);  //EL AND491 PARA LOS DISPLAYS GENERICOS
        ESCCON (0X1C);
        ESCCON (0X1C);
        ESCCON (0X1C);
    }


void BUSY (void)  //PREGUNTA POR EL ESTADO DE LA BANDERA BUSY Y ESPERA HASTA QUE SEA CERO
{ SysCtlDelay(24000); //espera 565.5 nS  1.8 mS

    }


/*

void BUSY (void)  //PREGUNTA POR EL ESTADO DE LA BANDERA BUSY Y ESPERA HASTA QUE SEA CERO
{ //SysCtlDelay(10666); //espera 565.5 nS  1.8 mS
    do LEECON( );               // VALCON        1XXX XXXX
    while ((Valcon & BIT7) != 0);   //BIT7 0X80  1000 0000B
    }


void   LEECON(void)
// LEE EL VALOR DEL REGISTRO DE CONTROL DEL DISPLAY Y REGRESA EL CONTENIDO EN VALCON
    {   PORTDIS=0;
        GPIO_PORTK_DIR_R=0x00; //PORTK como entrada
        bitset(PORTCON,R_W); // LEER PUERTO DE CONTROL
         bitset(PORTCON,E);      //ACTIVA E
         SysCtlDelay(8); //espera 565.5 nS
         Temp=PORTDIS; // LEE PARTE ALTA DEL BUS DE DATOS
         bitclr(PORTCON,E);
         //bitclr(PORTCON,E);
         bitclr(PORTCON,R_W);
         GPIO_PORTK_DIR_R=0XFF; //REGRESA A LA CONDICION ORIGINAL DEL PUERTO K A SALIDA
          Valcon=Temp;
                }
*/
//BYTEDIS(0x2d);

void BYTEDIS(uint8_t DByte) //escribe un byte a pantalla  0X45  0X04 +0x30  0x05 +0x30
                            // 0xad 0x0a +0x37 0x61
{ Temp2=DByte;
    Temp2=Temp2>>4;
    if (Temp2<=0x09)
        Temp2+=0x30;
        else
        Temp2+=0x37;
            ESCDAT(Temp2);
        Temp2=DByte&0x0f;
        if (Temp2<=0x09)
        Temp2+=0x30;
        else
        Temp2+=0x37;
            ESCDAT(Temp2);

}


//funcion de teclado

void LeerTeclas(void){
    charTecla=0xff;
    if((GPIO_PORTQ_DATA_R&0x0f)!=0x0f)
    {delayUs(20000);

//Q entrada H salida
    val =  GPIO_PORTQ_DATA_R;
    for (Fila=0; Fila<4; Fila++){
        GPIO_PORTH_AHB_DATA_R = filas[Fila]; //
        for (Columna=0;Columna<4;Columna++){
            if (GPIO_PORTQ_DATA_R == columnas[Columna])
            {
                charTecla=teclas [Fila][Columna];
              //  LCD_Write_Char(charTecla);



            }
        }
        //delayUs(10);
    }
    GPIO_PORTH_AHB_DATA_R &= 0x00;  //
    while((GPIO_PORTQ_DATA_R&0x0f)!=0x0f);
                   delayUs(20000);
    //menu_key();
    }
}




////////////////////////////////////////////////// FUNCIONES PARA LOS MENSAJES /////////////////////////////////////

void MostrarMensajeCiclo1(void){

    ESCCON(0x01); //Limpia Pantalla y regresa cursor al Inicio
    ESCCON(0xC0); // Posiciona Cursor en Segundo Renglon

                            i=0;
                                while(MensajeCiclo1[i]!= 0)

                                {
                                  ESCDAT(MensajeCiclo1[i++]);

                                }
                           SysCtlDelay(4000000);
                                ESCCON(0x01); //Limpia Pantalla y regresa cursor al Inicio

                                i=0;
                                    while(Mensaje3Puntos[i]!= 0)

                                    {
                                      ESCDAT(Mensaje3Puntos[i++]);

                                    }


                            SysCtlDelay(4000000);
                            i=0;
                                while(MensajeCiclo1_2[i]!= 0)

                                {
                                  ESCDAT(MensajeCiclo1_2[i++]);

                                }
                            SysCtlDelay(4000000);
                            i=0;
                                while(Mensaje3Puntos[i]!= 0)

                                {
                                  ESCDAT(Mensaje3Puntos[i++]);

                                }


}

void MostrarMensajeCiclo2(void){

    ESCCON(0x01); //Limpia Pantalla y regresa cursor al Inicio
    ESCCON(0xC0); // Posiciona Cursor en Segundo Renglon

                            i=0;
                                while(MensajeCiclo1[i]!= 0)

                                {
                                  ESCDAT(MensajeCiclo1[i++]);

                                }
                           SysCtlDelay(4000000);
                                ESCCON(0x01); //Limpia Pantalla y regresa cursor al Inicio

                                i=0;
                                    while(Mensaje3Puntos[i]!= 0)

                                    {
                                      ESCDAT(Mensaje3Puntos[i++]);

                                    }


                            SysCtlDelay(4000000);
                            i=0;
                                while(MensajeCiclo2[i]!= 0)

                                {
                                  ESCDAT(MensajeCiclo2[i++]);

                                }
                            SysCtlDelay(4000000);
                            i=0;
                                while(Mensaje3Puntos[i]!= 0)

                                {
                                  ESCDAT(Mensaje3Puntos[i++]);

                                }


}

void MostrarMensajeCiclo3(void){

    ESCCON(0x01); //Limpia Pantalla y regresa cursor al Inicio
    ESCCON(0xC0); // Posiciona Cursor en Segundo Renglon

                            i=0;
                                while(MensajeCiclo1[i]!= 0)

                                {
                                  ESCDAT(MensajeCiclo1[i++]);

                                }
                           SysCtlDelay(4000000);
                                ESCCON(0x01); //Limpia Pantalla y regresa cursor al Inicio

                                i=0;
                                    while(Mensaje3Puntos[i]!= 0)

                                    {
                                      ESCDAT(Mensaje3Puntos[i++]);

                                    }


                            SysCtlDelay(4000000);
                            i=0;
                                while(MensajeCiclo3[i]!= 0)

                                {
                                  ESCDAT(MensajeCiclo3[i++]);

                                }
                            SysCtlDelay(4000000);
                            i=0;
                                while(Mensaje3Puntos[i]!= 0)

                                {
                                  ESCDAT(Mensaje3Puntos[i++]);

                                }


}

void MostrarMensajeCiclo4(void){

    ESCCON(0x01); //Limpia Pantalla y regresa cursor al Inicio
    ESCCON(0xC0); // Posiciona Cursor en Segundo Renglon

                            i=0;
                                while(MensajeCiclo1[i]!= 0)

                                {
                                  ESCDAT(MensajeCiclo1[i++]);

                                }
                           SysCtlDelay(4000000);
                                ESCCON(0x01); //Limpia Pantalla y regresa cursor al Inicio

                                i=0;
                                    while(Mensaje3Puntos[i]!= 0)

                                    {
                                      ESCDAT(Mensaje3Puntos[i++]);

                                    }


                            SysCtlDelay(4000000);
                            i=0;
                                while(MensajeCiclo4[i]!= 0)

                                {
                                  ESCDAT(MensajeCiclo4[i++]);

                                }
                            SysCtlDelay(4000000);
                            i=0;
                                while(Mensaje3Puntos[i]!= 0)

                                {
                                  ESCDAT(Mensaje3Puntos[i++]);

                                }


}

void MostrarMensajeVaciado(void){
    SysCtlDelay(4000000);

//    static const uint8_t MensajeVaciado1[] = "Ponga Manguera";
//    static const uint8_t MensajeVaciado2[] = " Presione 5";

    ESCCON(0x01); //Limpia Pantalla y regresa cursor al Inicio

                            i=0;
                                while(MensajeVaciado1[i]!= 0)

                                {
                                  ESCDAT(MensajeVaciado1[i++]);

                                }
                            SysCtlDelay(4000000);

                            ESCCON(0xC0); // Posiciona Cursor en Segundo Renglon
                            i=0;
                                while(MensajeVaciado2[i]!= 0)

                                {
                                  ESCDAT(MensajeVaciado2[i++]);

                                }


}

void MostrarMensajeError1(void){

    ESCCON(0x01); //Limpia Pantalla y regresa cursor al Inicio
    ESCCON(0xC0); // Posiciona Cursor en Segundo Renglon
    ESCDAT('-');

                            i=0;
                                while(MensajeError1[i]!= 0)

                                {
                                  ESCDAT(MensajeError1[i++]);

                                }

                                for(contador=0;contador<=16;contador++) //Recorrer Display 8 Espacios

                                    {ESCCON(0B00011000);
                                        SysCtlDelay(2666666);   //t*3*n  @16MHz-> t=62.5nS *3=187.5nS   .5/187.5e-9
                                    }

}


void MostrarMensajeError2(void){

    ESCCON(0x01); //Limpia Pantalla y regresa cursor al Inicio
    ESCCON(0xC0); // Posiciona Cursor en Segundo Renglon
    ESCDAT('-');

                            i=0;
                                while(MensajeError2[i]!= 0)

                                {
                                  ESCDAT(MensajeError2[i++]);

                                }

                                for(contador=0;contador<=8;contador++) //Recorrer Display 8 Espacios

                                    {//ESCCON(0B00011000);
                                        SysCtlDelay(2666666);   //t*3*n  @16MHz-> t=62.5nS *3=187.5nS   .5/187.5e-9
                                    }

}
void MostrarMensajeError3(void){

ESCCON(0x01); //Limpia Pantalla y regresa cursor al Inicio
ESCCON(0xC0); // Posiciona Cursor en Segundo Renglon
ESCDAT('-');

                            i=0;
                                while(MensajeError3[i]!= 0)

                                {
                                  ESCDAT(MensajeError3[i++]);

                                }

                                for(contador=0;contador<=8;contador++) //Recorrer Display 8 Espacios

                                    {//ESCCON(0B00011000);
                                        SysCtlDelay(2666666);   //t*3*n  @16MHz-> t=62.5nS *3=187.5nS   .5/187.5e-9
                                    }

ESCCON(0x01); //Limpia Pantalla y regresa cursor al Inicio
ESCCON(0xC0); // Posiciona Cursor en Segundo Renglon

                            i=0;
                               while(MensajeError3_1[i]!= 0)

                               {
                                 ESCDAT(MensajeError3_1[i++]);

                               }

                               for(contador=0;contador<=8;contador++) //Recorrer Display 8 Espacios

                                   {ESCCON(0B00011000);
                                       SysCtlDelay(2666666);   //t*3*n  @16MHz-> t=62.5nS *3=187.5nS   .5/187.5e-9
                                   }



}
void MostrarMensajeExito(void){


                            i=0;
                                  while(MensajeExito1[i]!= 0)

                                  {
                                    ESCDAT(MensajeExito1[i++]);

                                  }
                             SysCtlDelay(4000000);
                                  ESCCON(0x01); //Limpia Pantalla y regresa cursor al Inicio
                                  ESCDAT('-');  //AQUI

                              i=0;
                                  while(MensajeExito2[i]!= 0)

                                  {
                                    //ESCDAT('-');  //AQUI
                                    ESCDAT(MensajeExito2[i++]);

                                  }
                              SysCtlDelay(4000000);

  }


////////////////////////////////////////////////// FUNCIONES PARA EL LAVADO /////////////////////////////////////////////////

// Función para inicializar el puerto D (Servo)
void PORTD_INI(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3; // Habilitar reloj para puerto D

    // CONFIGURACION DE PUERTO D4 PARA PWM
    GPIO_PORTD_AHB_DEN_R = 0x10;            // Habilitar el pin PD4 como digital
    GPIO_PORTD_AHB_DIR_R = 0x10;            // Configurar PD4 como salida
    GPIO_PORTD_AHB_AFSEL_R = 0x10;          // Habilitar función alternativa en PD4
    GPIO_PORTD_AHB_PCTL_R = (GPIO_PORTD_AHB_PCTL_R & 0xFFF0FFFF) | 0x00030000; // Seleccionar función alternativa de TIMER3 en PD4
}

// Función para inicializar el timer 3 (SERVOMOTOR)
void TIMER_INI(void) {

    SYSCTL_RCGCTIMER_R |= 0x08;              // Habilitar TIMER3
    ui32Loop = SYSCTL_RCGCTIMER_R;           // Pequeño retardo para estabilización
    TIMER3_CTL_R = 0x00000000;               // Deshabilitar TIMER3 para configuración

    TIMER3_CTL_R = 0x00000000;              // Desactivar el TIMER3 durante la configuración
    TIMER3_CFG_R = 0x00000004;              // Configurar como Timer de 16 bits
    TIMER3_TAMR_R = 0x0000000A;             // Configurar en modo periódico y PWM invertido

    TIMER3_TAPR_R = 0x00;                   // No se necesita prescaler
    TIMER3_TAILR_R = 0x4E200;               // Valor de carga para un periodo de 20 ms (frecuencia de 50 Hz)

    TIMER3_CTL_R |= 0x00000001;             // Limpiar bandera de interrupción
}


// Función para inicializar el timer 1 (Medición de tiempo para ECHO)
void TIMER_INI_SENSOR(void){
    SYSCTL_RCGCTIMER_R |= 0x02; // HABILITA TIMER 1 (0x02 = Bit 1 de la máscara para Timer 1)
    ui32LoopS = SYSCTL_RCGCTIMER_R;  // Retardo para que el reloj alcance el PORTN Y TIMER 1
    TIMER1_CTL_R = 0x00000000; // DESHABILITA TIMER EN LA CONFIGURACION

    TIMER1_CFG_R = 0x00000000; // CONFIGURAR PARA 32 BITS
    TIMER1_TAMR_R = 0x00000012; // CONFIGURAR PARA MODO PERIODICO CUENTA HACIA ARRIBA
    TIMER1_TAILR_R = 0x00FFFFFF; // VALOR DE RECARGA (Por si llegase a desbordarse)
    TIMER1_TAPR_R = 0x00; // SOLO PARA MODOS DE 16 BITS
    TIMER1_ICR_R = 0x00000001; // LIMPIA PISOBLE BANDERA PENDIENTE DE TIMER1
}


// Funcion para Mover el servomotor
void MOVER_SERVO(void) {

    TIMER3_CTL_R |= 0x00000001;             // Se Asegura de que el TIMER 3 está encendido

    uint32_t loopM;
    uint32_t elapsed_time = 0;

    // Ciclo de operación del servo durante 60 segundos
    while (elapsed_time < 1240) { // 55 segundos = 2500 aproximadamente

        // Generar ciclo de trabajo mínimo (~5%) para movimiento continuo en una dirección
       // TIMER3_TAMATCHR_R = 0x3A980; // Ajuste para aproximadamente 2 ms (giro en sentido antihorario)

        //TIMER3_TAMATCHR_R = 0xBB80;  // Ajuste para aproximadamente 3 ms (giro en sentido antihorario)

        TIMER3_TAMATCHR_R =0xC800; // Ajuste para aproximadamente 3.2 ms (giro en sentido antihorario) IDEAL

        //TIMER3_TAMATCHR_R =0xCE00;  // Ajuste para aproximadamente 3.3 ms (giro en sentido antihorario)(ValMAX)

        for (loopM = 0; loopM < 1000000; loopM++); // Pequeña pausa
        elapsed_time += 20; // Incrementar el tiempo en 20 ms (aproximación por iteración)

       // Generar ciclo de trabajo máximo (~10%) para movimiento continuo en la otra dirección
       //TIMER3_TAMATCHR_R = 0x47A00; // Ajuste para aproximadamente 1 ms (giro en sentido horario)

       TIMER3_TAMATCHR_R = 0x3B80; // Ajuste para aproximadamente 0.95 ms (giro en sentido horario) //IDEAL

        //TIMER3_TAMATCHR_R = 0x3C00; // Ajuste para aproximadamente 0.96 ms (giro en sentido horario)

        //TIMER3_TAMATCHR_R = 0x3840; // Ajuste para aproximadamente 0.9 ms (giro en sentido horario)

         //TIMER3_TAMATCHR_R =0x3580; // Ajuste para aproximadamente 0.85 ms (giro en sentido horario)

        //TIMER3_TAMATCHR_R =0x3480; // Ajuste para aproximadamente 0.84 ms (giro en sentido horario) (ValMIN)

        for (loopM = 0; loopM < 1000000; loopM++); // Pausa
        elapsed_time += 20; // Incrementar el tiempo en 20 ms

    }

    // Desactivar el TIMER3 una vez terminado el tiempo
    TIMER3_CTL_R = 0x00000000; // Desactivar TIMER3
    TIMER3_TAMATCHR_R = 0x0;   // Desactivar señal PWM en PD4
}

void CicloLavado(void){

            MostrarMensajeCiclo2();
            int garantia = 0;

            while (garantia ==0) {

                TRIGGER_04(); // Generar trigger

                TIMER1_TAV_R = 0x00;
                while((GPIO_PORTN_DATA_R & 0x20) == 0);  // 0010 0000 ESPERAR A QUE PN5 SEA 1

                TIMER1_CTL_R |= 0x00000001; // HABILITA TIMER EN LA CONFIGURACION CUANDO PN5 SEA 1
                while((GPIO_PORTN_DATA_R & 0x20) == 0x20);
                TIMER1_CTL_R &= ~(0x00000001);  // Detener timer

                // ui32Distancia = TIMER1_TAV_R / (58 * 48); // Se corrigió de 50 a 48, ya que estamos trabajando a 48 MHz; Entonces, el factor por el que se multiplica es la Frecuencia del Reloj

                 ui32Distancia = TIMER1_TAV_R / (58 * 15);
                 SysCtlDelay(1050000);   // Retardo de 63 mS

                // Si la distancia es menor o igual a 6 cm, mover el servo durante un minuto
                if (ui32Distancia <= 9) {
                        MOVER_SERVO(); // Mover servo
                        garantia = 1;
                    }

                else {
                      MostrarMensajeError2(); //Mensaje para indicar que no hay suficiente agua.
                      SysCtlDelay(400000);
                      ESCCON(0x01);
                      MostrarMensajeCiclo1();
                      SysCtlDelay(4000000);
                      Ciclo_Sensado_Llenado();
                   }
              // Retardo para evitar mediciones rápidas consecutivas
              SysCtlDelay(1050000); // Aprox. 63 ms
          }
}


////////////////////////////////////////////////// FUNCIONES PARA EL LLENADO /////////////////////////////////////////////////

// Función para inicializar el puerto N
void PORTN_INI(void) {
    SYSCTL_RCGCGPIO_R = SYSCTL_RCGCGPIO_R12; // Habilitar reloj para puerto N
    ui32Loop = SYSCTL_RCGCGPIO_R;

    GPIO_PORTN_DIR_R = RELAY_PIN3 | RELAY_PIN0 | TRIGGER_PIN; // Configurar PN0 y PN4 como salidas
    GPIO_PORTN_DEN_R = RELAY_PIN3 | RELAY_PIN0 | TRIGGER_PIN | ECHO_PIN; // Configurar PN0, PN4, PN5 como digitales
    GPIO_PORTN_DATA_R = 0x00; // Apagar relé y trigger inicialmente
}

void TRIGGER_04 (void) // Función para generar disparo
{
     GPIO_PORTN_DATA_R |= 0B00010000;  // PONER A 1 PN4
     SysCtlDelay(168);  // 50 MHz T=20 ns  20 x 3 = 60 nS  cuanto vale x para 10 uS con 60 nS RB //Modificamos Fecuencia de Trabajo y Disparos por Tiempo
     // retardo de 10 uS    x=10uS / 60 nS
     GPIO_PORTN_DATA_R &= ~(0B00010000); // PONER A 0 PN4
}


//Funcion del Llenado

void Ciclo_Sensado_Llenado(void){

   int garantia = 0;

       while (garantia == 0) {
           MostrarMensajeCiclo1();

           TRIGGER_04();

           TIMER1_TAV_R = 0x00;
           while((GPIO_PORTN_DATA_R & 0x20) == 0);  // 0010 0000 ESPERAR A QUE PN5 SEA 1

           TIMER1_CTL_R |= 0x00000001; // HABILITA TIMER EN LA CONFIGURACION CUANDO PN5 SEA 1
           while((GPIO_PORTN_DATA_R & 0x20) == 0x20);
           TIMER1_CTL_R &= ~(0x00000001);  // Detener timer

          // ui32Distancia = TIMER1_TAV_R / (58 * 48); // Se corrigió de 50 a 48, ya que estamos trabajando a 48 MHz; Entonces, el factor por el que se multiplica es la Frecuencia del Reloj

           ui32Distancia = TIMER1_TAV_R / (58 * 15);
           SysCtlDelay(1050000);   // Retardo de 63 mS


      // Control del relé según distancia  para (BOMBA DE LLENADO)

           if (ui32Distancia >= 10) { // Distancia a medir apra apagr relevador y por ende bomba. //Puede ser 7
               LlenadoManual();
           }
           else if (ui32Distancia <= 9) {
                 GPIO_PORTN_DATA_R |= RELAY_PIN0; // Apaga relé  //APAGADO
              // GPIO_PORTN_DATA_R &= ~RELAY_PIN0;  // Enciende relé
               garantia = 1;
           }

           SysCtlDelay(1050000); //Retardo para evitar mediciones rápidas consecutivas (Aprox. 63 ms)
       }
       SysCtlDelay(1050000); // Retardo para evitar mediciones rápidas consecutivas (Aprox. 63 ms)
}

////////////////////////////////////////////////// FUNCIONES PARA EL VACIADO /////////////////////////////////////////////////

void CicloVaciado(void) {

    int garantia = 0;

        while (garantia == 0) {
            MostrarMensajeCiclo4();

            TRIGGER_04();

            TIMER1_TAV_R = 0x00;
            while((GPIO_PORTN_DATA_R & 0x20) == 0);  // 0010 0000 ESPERAR A QUE PN5 SEA 1

            TIMER1_CTL_R |= 0x00000001; // HABILITA TIMER EN LA CONFIGURACION CUANDO PN5 SEA 1
            while((GPIO_PORTN_DATA_R & 0x20) == 0x20);
            TIMER1_CTL_R &= ~(0x00000001);  // Detener timer

            // ui32Distancia = TIMER1_TAV_R / (58 * 48); // Se corrigió de 50 a 48, ya que estamos trabajando a 48 MHz; Entonces, el factor por el que se multiplica es la Frecuencia del Reloj

             ui32Distancia = TIMER1_TAV_R / (58 * 15);
             SysCtlDelay(1050000);   // Retardo de 63 mS

            // Control del relé según tiempo
            if (ui32Distancia <= 9) {

                VaciadoManual();
                garantia=1;
            }
            else if(ui32Distancia >= 10)  {
                GPIO_PORTN_DATA_R |= RELAY_PIN3; // Mantener relé apagado
                MostrarMensajeError3();
                SysCtlDelay(40000);

                MostrarMensajeCiclo1();
                SysCtlDelay(400000);
                Ciclo_Sensado_Llenado();
                SysCtlDelay(400000);
            }

            SysCtlDelay(1050000); // Retardo para evitar mediciones rápidas consecutivas (Aprox. 63 ms)
        }

}
//////////////////////////////////////////// FUNCIONES PARA EL ENJUAGADO //////////////////////////////////////////////////////


void CicloEnjuagado(void){

    MostrarMensajeCiclo3();
    SysCtlDelay(400000);
    CicloVaciado();
    SysCtlDelay(400000);

    MostrarMensajeCiclo3();
    SysCtlDelay(400000);
    Ciclo_Sensado_Llenado();
    SysCtlDelay(400000);

    MostrarMensajeCiclo3();
    SysCtlDelay(400000);
    CicloLavado();
    SysCtlDelay(400000);

}

//////////////////// MEDICION VOLTAJE Y FUNCIONES MANUALES EXTRA /////////////////////////////////////////////

void MedicionVoltaje(void){

        ADC0_PSSI_R=0x0008;              // Iniciar muestreo en SS3
               result=(ADC0_SSFIFO3_R&0xFFF);
               v=(result * 3.3) / 4096;
               VOLTAJE=(v/(R2/(R1+R2)));
               ADC0_ISC_R = 8;                 // Limpiar la interrupción en SS3

                for ( delay = 0; delay < 1000; delay++);
}

void IniVoltaje(void){
    //HABILITACIÓN DE PUERTOS
    //SYSCTL_RCGCGPIO_R=SYSCTL_RCGCGPIO_R4;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4 | SYSCTL_RCGCGPIO_R1; // GPIOE y GPIOB
    SYSCTL_RCGCADC_R |= 1;                                       // ADC0
    while ((SYSCTL_PRGPIO_R & (SYSCTL_PRGPIO_R4 | SYSCTL_PRGPIO_R1)) == 0); // Esperar habilitación de GPIO

    // Configuración del pin PE4 como entrada del ADC
    GPIO_PORTE_AHB_DIR_R&=~0x10;
    GPIO_PORTE_AHB_AFSEL_R|=0x10;         // Habilitar función alternativa en PE4
    GPIO_PORTE_AHB_DEN_R=~0x10;           // Deshabilitar función digital en PE4
    GPIO_PORTE_AHB_AMSEL_R|=0x10;         // Habilitar función analógica en PE4

    // Habilitar función alternativa en PE4
    GPIO_PORTE_AHB_DEN_R &= ~0x10;
    GPIO_PORTE_AHB_AMSEL_R |= 0x10;

    // Configuración del pin PB4 como salida digital (LED)
    GPIO_PORTB_AHB_DIR_R |= 0x10;        // Configurar PB4 como salida
    GPIO_PORTB_AHB_DEN_R |= 0x10;        // Habilitar función digital en PB4
    GPIO_PORTB_AHB_DATA_R &= ~0x10;      // Asegurarse de que el LED esté apagado

    //CONFIGURACIÓN ADC0
    ADC0_PC_R=0x01;
    ADC0_SSPRI_R=0x0123;
    ADC0_ACTSS_R&=~0x0008;               // Deshabilitar el secuenciador 3 durante la configuración
    ADC0_EMUX_R&=~0xF000;                // Configuración para disparo por procesador
    ADC0_SSMUX3_R=(ADC0_SSMUX3_R & 0xFFFFFFF)+0x09;   // Seleccionar canal AIN9 (PE4)
    ADC0_SSCTL3_R=0x0006;                // Configurar para una muestra con interrupción al final
    ADC0_IM_R |= 0x08;
    ADC0_ACTSS_R|=0x0008;                // Habilitar el secuenciador SS3
    //NVIC_EN0_R = 1 << 17;              // Habilitar interrupción en NVIC para ADC0 SS3
    // NVIC_EN1_R=0x80000;
    // NVIC_EN0_R |= 1 << 17;
    NVIC_PRI4_R = (NVIC_PRI4_R & ~0xE000) | (2 << 13); // Configurar prioridad 2
    NVIC_EN0_R |= (1 << 17);                           // Habilitar interrupción 17 (ADC0 SS3)

    //CONFIGURACIÓN DE PLL (PHASE LOCKED LOOP)
    SYSCTL_PLLFREQ0_R|=SYSCTL_PLLFREQ0_PLLPWR;  //ACTIVACIÓN DE PLL
    while((SYSCTL_PLLSTAT_R&0x01)==0);          //DESACTIVACIÓN DE PLL
    SYSCTL_PLLFREQ0_R&=~SYSCTL_PLLFREQ0_PLLPWR;
}

void Ciclo1Completo(void){
    SysCtlDelay(4000000);
    Ciclo_Sensado_Llenado(); //Ejecuta codigo Para medir nivel de agua y Llenar Tina
    CicloLavado(); //Ejecuta codigo para Lavar Ropa Con la Tina Llena.
}

void VaciadoManual(void){

  int flagVaciado=0;
  ESCCON(0x01);
  MostrarMensajeVaciado();

  while(flagVaciado==0){
    LeerTeclas();

   if (charTecla!=0xff){
    valtec=charTecla;
    ESCDAT(valtec);

    if (charTecla == 53) {
        GPIO_PORTN_DATA_R &= ~RELAY_PIN3; // Encender relé
        for (ui32Loop = 0; ui32Loop < 30000000; ui32Loop++); // Retardo de 39 segundos (~50 MHz)
        GPIO_PORTN_DATA_R |= RELAY_PIN3; // Apagar relé // VACIADO CICLO 4
        flagVaciado=1;
    }
    else{
        MostrarMensajeError1();
        VaciadoManual();

    }
   }
 }
}

void LlenadoManual(void){

    int flagLlenado=0;
    ESCCON(0x01);
    MostrarMensajeVaciado(); //Tambien funciona como llenado

  while(flagLlenado==0){
    LeerTeclas();

   if (charTecla!=0xff){
      valtec=charTecla;
      ESCDAT(valtec);

      if (charTecla == 53) {
         GPIO_PORTN_DATA_R &= ~RELAY_PIN0; // Encender relé
         for (ui32Loop = 0; ui32Loop < 30000000 ; ui32Loop++); // Retardo de 39 segundos (~50 MHz)
         GPIO_PORTN_DATA_R |= RELAY_PIN0; // APAGA RELE
       //  GPIO_PORTN_DATA_R |= RELAY_PIN3;
        // Ciclo_Sensado_Llenado();
          flagLlenado=1;
      }
      else{
         MostrarMensajeError1();
         LlenadoManual();
      }

   }
  }
}
