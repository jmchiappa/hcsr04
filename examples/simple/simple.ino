#include "HCSR04.h"

#define PIN_TRIGGER1	D3
#define PIN_ECHO1			D2

#define PIN_TRIGGER2	D5
#define PIN_ECHO2			D4

#define PRINT(a,b)	{Serial.print(a);Serial.print(b);}

#define PRESCALER	25
// if you want to force a timer peripheral :
// HCSR04 sensor(PIN_TRIGGER,PIN_ECHO,TIM1);

// if you don't care about timer peripheral to use :
HCSR04 sensor1(PIN_TRIGGER1,PIN_ECHO1);
HCSR04 sensor2(PIN_TRIGGER2,PIN_ECHO2);


void setup() {
	Serial.begin(115200);
	if( !sensor1.begin(PRESCALER) ) {
		Serial.println("can't intialize sensor1");
		while(1);
	}
	if( !sensor2.begin(PRESCALER) ) {
		Serial.println("can't intialize sensor2");
		while(1);
	}
}
float  dist1,dist2;
void loop() {
	if( sensor1.DistanceUpdated() ) {
		if( sensor1.IsObjectDetected() ) {
			// to use with serial plotter
			// PRINT("sensor1: ",sensor1.DistanceinCm());
			dist1 = sensor1.DistanceinCm();
			// to use with serial monitor
			//Serial.print("distance : ");Serial.println( sensor.DistanceinCm() );
		} else {
			// Serial.print("sensor1:no obstacle");
		}
}
	if( sensor2.DistanceUpdated() ) {
		if( sensor2.IsObjectDetected() ) {
			// to use with serial plotter
			// PRINT(" - sensor2: ",sensor2.DistanceinCm());
			dist2 = sensor2.DistanceinCm();
			// to use with serial monitor
			//Serial.print("distance : ");Serial.println( sensor.DistanceinCm() );
		} else {
			// Serial.print(" - sensor2: no obstacle");
		}
	}
	PRINT("sensor1: ",dist1);
	PRINT(",sensor2: ",dist2);
	Serial.println();
	// delay(20);
}