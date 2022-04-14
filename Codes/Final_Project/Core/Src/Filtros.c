#include "Filtros.h"


double numerador[]={-0.0212206590789194 ,1.94908591625969e-17 ,0.0244853758602916 ,-1.94908591625969e-17 ,-0.0289372623803446 ,1.94908591625969e-17 ,0.0353677651315323 ,-1.94908591625969e-17 ,-0.0454728408833987 ,1.94908591625969e-17 ,0.0636619772367581 ,-1.94908591625969e-17 ,-0.106103295394597 ,1.94908591625969e-17 ,0.318309886183791 ,0.500000000000000 ,0.318309886183791 ,1.94908591625969e-17 ,-0.106103295394597 ,-1.94908591625969e-17 ,0.0636619772367581 ,1.94908591625969e-17 ,-0.0454728408833987 ,-1.94908591625969e-17 ,0.0353677651315323 ,1.94908591625969e-17 ,-0.0289372623803446 ,-1.94908591625969e-17 ,0.0244853758602916 ,1.94908591625969e-17 ,-0.0212206590789194};
double denominador[4]={1,-1.76004188034317,1.18289326203783,-0.278059917634547};

short mediciones[31]={0};
float salida[4]={0};

double FIR(short medicion_actual){
	double resultado_suma=0; // Variable donde almacenare el resultado de la suma y la multiplicacion

	// Almacenar el dato actual
	mediciones[30]=medicion_actual; // Almaceno la imformacion en la ultima posicion del vector

	for(short i=0; i<30; i++){ // For para recorrer todo el vector
		resultado_suma=resultado_suma+numerador[i]*mediciones[30-i]; // Calculo el Filtro
		mediciones[i]=mediciones[i+1];
	}
	return resultado_suma;
}
float IIR(short medicion_actual){
	double resultado_FIR=0;
	double resultado_IIR=0;
	float resultado=0;

	// Almacenar la muestra actual
	mediciones[0]=medicion_actual;

	for(short i=0; i<4; i++){
		resultado_FIR=resultado_FIR+(numerador[i]*mediciones[i]);
	}

	for(short i=1; i<4; i++){
		resultado_IIR=resultado_IIR+(numerador[i]*salida[i]);
	}

	resultado=resultado_FIR-resultado_IIR;

	// Actualizar variables

	mediciones[3]=mediciones[2];
	mediciones[2]=mediciones[1];
	mediciones[1]=mediciones[0];

	salida[0]=resultado;

	salida[3]=salida[2];
	salida[2]=salida[1];
	salida[1]=salida[0];

	return resultado;
}
