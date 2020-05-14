#include "HCSR04.h"

#define PIN_TRIGGER	D3
#define PIN_ECHO	D2
#define PRESCALER	25
// if you want to force a timer peripheral :
// HCSR04 sensor(PIN_TRIGGER,PIN_ECHO,TIM1);

// if you don't care about timer peripheral to use :
HCSR04 sensor(PIN_TRIGGER,PIN_ECHO);


void setup() {
	Serial.begin(115200);
	if( !sensor.begin(PRESCALER) ) {
		Serial.println("can't intialize timer hardware");
		while(1);
	}
}

void loop() {
	if( sensor.DistanceUpdated() ) {
		if( sensor.IsObjectDetected() ) {
			// to use with serial plotter
			Serial.println(sensor.DistanceinCm());
			// to use with serial monitor
			//Serial.print("distance : ");Serial.println( sensor.DistanceinCm() );
		} else {
			Serial.println("no obstacle");
		}
	}
}