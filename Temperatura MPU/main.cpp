/* Conexiones:
		PB11: SDA 
		PB10: SCL
		
*/

// ************* LIBRERIAS ****************
		#include <stdio.h>
		#include "STM32F7xx.h"
		#include "math.h"
// ****************************************

// ************* VELOCIDADES **************
		#define standar_mode 0x30420F13
		#define fast_mode 0x10320309
		#define fast_plus_mode 0x00200204 
// ****************************************

// ************* VARIABLES GLOBALES *******
		// Variables IMU 1
			int16_t aceleracion_X=0; // Variable donde almaceno la aceleracion X
			int16_t aceleracion_Y=0;
			int16_t aceleracion_Z=0;
			int16_t temperatura=0;
			
			float aceleracion_X_convertida=0; // Variable donde almacenare la aceleracion en m/s^2
			float aceleracion_Y_convertida=0;
			float aceleracion_Z_convertida=0;
			float temperatura_convertida=0;
			short temp=0;
			
		// Variables UART
			short dato_recibido=0;
			short caracter1=0;
			short caracter2=0;  // Variables donde guardare la fragmentacion del valor a enviar
			short caracter3=0;
			short caracter4=0;
// ****************************************

// ************* FUNCIONES ****************
		// Funciones I2C
		void escribir(char direccion_esclavo, char direccion_escribir, char dato){
			I2C2->CR2 |=I2C_CR2_AUTOEND; // Habilito la finalizacion automatica de la comuniaccion 
			I2C2->CR2 &=~I2C_CR2_RD_WRN; // Modo escritura 
			I2C2->CR2 |=(direccion_esclavo<<1); // Envio la direccion del esclavo 
			I2C2->CR2 |=(2<<16); // Envio la cantidad de bytes de la comunicacion 
			I2C2->CR2 |=I2C_CR2_START; // Condicion de inicio 
			while(!(I2C2->ISR & I2C_ISR_TXIS));
			I2C2->TXDR =direccion_escribir; // Envio la direccion del registro que quiero acceder 
			while(!(I2C2->ISR & I2C_ISR_TXIS));
			I2C2->TXDR =dato; // Envio el dato a la direccion especificada anteriormente
			while(!(I2C2->ISR & I2C_ISR_TXE)); // Verifico que envie la informacion correctamente
			while(!(I2C2->ISR & I2C_ISR_STOPF)); // Tiempo de espera mientras el proceso se acaba
			I2C2->CR2 =0x02000000; // CR2 lo vuelvo al inicio 
	} 
		char leer_datos(char direccion_esclavo,char direccion_leer){ // Me retorna un tipo de dato entero de 8 bits
			
			I2C2->CR2 &=~I2C_CR2_RD_WRN; // Modo Escritura para enviar la direccion del esclavo 
			I2C2->CR2 |=(direccion_esclavo<<1); // Envio la direccion del esclavo 
			I2C2->CR2 |=(1<<16); // Envio la cantidad de bytes que se transmitiran 
			I2C2->CR2 |=I2C_CR2_AUTOEND; // Activo auto end
			I2C2->CR2 |=I2C_CR2_START; // Condicion de inicio 
			while(!(I2C2->ISR & I2C_ISR_TXIS));
			I2C2->TXDR = direccion_leer; // Envio la direccion del registro a leer 
			while (!(I2C2->ISR & I2C_ISR_TXE)); // Verifico que todo se halla enviado correctamente 
			while (!(I2C2->ISR & I2C_ISR_STOPF)); // Espero mientras se detiene la comnucacion 
		
			// Reset de la comunicacion 
			I2C2->CR1 &= ~I2C_CR1_PE;
			while (I2C2->CR1 & I2C_CR1_PE);
			I2C2->CR1 |=I2C_CR1_PE; // Activo el periferico I2C 
		
			I2C2->CR2 |=(direccion_esclavo<<1);
			I2C2->CR2 |=I2C_CR2_RD_WRN; // Modo lectura
			I2C2->CR2 |=(1<<16);
			I2C2->CR2 |=I2C_CR2_START;
			while (!(I2C2->ISR & I2C_ISR_RXNE));  // Verifico si se lleno el buffer de datos recibidos 
			uint8_t informacion = I2C2->RXDR; // Obtengo la informacion del sensor  
			while (!(I2C2->ISR & I2C_ISR_STOPF)); // Espero hasta que se acabe la comunicacion 
			I2C2->CR2 |=I2C_CR2_STOP;
			
			// Reset de la comunicacion
			I2C2->CR1 &= ~I2C_CR1_PE;
			while (I2C2->CR1 & I2C_CR1_PE);
			I2C2->CR1 |=I2C_CR1_PE; // Activo el periferico I2C 
			return informacion;
		}
		void lectura(void){			
					// Obtener medidas del sensor
					aceleracion_X = ((leer_datos(0x68,0x3B)<<8) | leer_datos(0x68,0x3C));
					temperatura = ((leer_datos(0x68,0x41)<<8) | leer_datos(0x68,0x42));
					
					// Conversion a unidades de medida
					aceleracion_X_convertida = (((aceleracion_X*2.0)/32767.0)*9.80665); // Calculo la aceleracion en 1G
					temperatura_convertida= (temperatura/340.0)+36.53; 
		}
		// Funciones del UART
		void enviar_caracter(char caracter){ // Inicio de enviar uncaracter 
					UART4->TDR =caracter;
					while((UART4->ISR &=0x80)==0){}  
	} // Fin funcion de enviar un caracter 
		void enviar_frase(char frase[], char parada){ // Inicio funcion enviar frase 
			char caracter_frase='1';
		for(int i=0;caracter_frase!=parada;i++){ // Inicio for para enviar
			caracter_frase=frase[i]; // Lo que hago es que a una variable externa le mando el caracter i de la frase 
			UART4->TDR = caracter_frase; // El contenido de la variable externa la mando al registro de transmision para mandar por el serial 
			while((UART4->ISR &=0x80)==0){} // While de tiempo para una correcta transmision de datos  
		} // Fin for para enviar 
		caracter_frase='1';
	} // Fin funcion enviar frase 
		void dividir_datos(short resultado){ // Inicio de funcion de dividir datos 
				caracter1 = resultado/1000;
				caracter2 = (resultado/100)%10;
				caracter3 = (resultado%100)/10;
				caracter4 = (resultado%10);
		} // Fin funcion de dividir numeros
// ****************************************************

// ********************** INTERRUPCIONES **************
		extern "C"{
			void UART4_IRQHandler(void){
					dato_recibido = UART4->RDR; // Guardo el dato 
					}
			}
// ****************************************

// ************* MAIN *********************
int main(void){
		// ********* PUERTOS ******************
				RCC->AHB1ENR |=0x06; // Puerto B y C
		// ************************************
	
		// ********* PINES ********************
				GPIOB->MODER |=0XA00000; // ALTERNANTE pin 10 y 11
				GPIOB->AFR[1]=0X4400; // PB10 y PB11 -> AF4
				GPIOB->OTYPER |=0xC00; // OPEN DRAIN pines 10 y 11
				GPIOB->OSPEEDR |=0xA00000; //HIGH-SPEED pines 10 y 11
				GPIOA->PUPDR |=0x500000; // PULL-UP pines 10 y 11
	
				GPIOC->MODER |=0xA00000; //ALTERNATIVO pin 10 y 11
				GPIOC->AFR[1]=0X8800; // PB10 y PB11 -> AF8
		// ************************************
	
		// ****************** UART ************************
				RCC->APB1ENR |=0x80000; // Activo el reloj del UART 4 
				UART4->BRR =0x683; // 9600 Baudios
				UART4->CR1 |=0x2C; // Activo Rx, Tx y la interrupcion por Rx
				UART4->CR1 |=0x01; // Habilito el modulo UART
				NVIC_EnableIRQ(UART4_IRQn); 
		// ************************************************
	
		// ********* I2C **********************
				RCC->APB1ENR |=RCC_APB1ENR_I2C2EN; // Habilito el reloj del I2C2						
				I2C2->TIMINGR |=fast_plus_mode;  
				I2C2->CR1 |=0x01; // Activo el periferico I2C
				
				escribir(0x68,0x6B,0x00); // Despertar IMU 
		// ************************************
		
		// ********* BUCLE ********************
				while(true){
					// Leer las aceleraciones 
					aceleracion_X=(leer_datos(0x68,0x3B)<<8); // Leo la parte alta de la aceleracion 
					aceleracion_X=aceleracion_X | leer_datos(0x68,0x3C); // Leo la aceleracion completa en X 
					aceleracion_Y=(leer_datos(0x68,0x3D)<<8); // Leo la parte alta de la aceleracion 
					aceleracion_Y=aceleracion_Y | leer_datos(0x68,0x3E); // Leo la aceleracion completa en Y
					aceleracion_Z=(leer_datos(0x68,0x3F)<<8); // Leo la parte alta de la aceleracion 
					aceleracion_Z=aceleracion_Z | leer_datos(0x68,0x40); // Leo la aceleracion completa en Z
					
					aceleracion_X_convertida=((aceleracion_X*2.0)/32767.0)*9.80665; // Convierto la aceleracion a m/s^2
					aceleracion_Y_convertida=((aceleracion_Y*2.0)/32767.0)*9.80665; // Convierto la aceleracion a m/s^2
					aceleracion_Z_convertida=((aceleracion_Z*2.0)/32767.0)*9.80665; // Convierto la aceleracion a m/s^2

					temperatura = ((leer_datos(0x68,0x41)<<8) | leer_datos(0x68,0x42));
					temperatura_convertida= (temperatura/340.0)+36.53; 
					temp=temperatura_convertida*10;
				}
		// ************************************
}
// ****************************************