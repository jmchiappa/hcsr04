/*********************************************************************
	HC-SR04library

	Author : Jean-Marc Chiappa
	Date   : 13.05.2020
	Rev : 1
	
	*******************************************************************/

#ifndef hcsr04_h
#define hcsr04_h

#include "Arduino.h"
#include "PeripheralPins.h"

//#define DEBUG

#ifdef DEBUG
# define DEBUG1LN(a)	{Serial.println(a);}
# define DEBUG(a,b)	{Serial.print(a);Serial.print(b);}
# define DEBUGLN(a,b)	{Serial.print(a);Serial.println(b);}
#else
# define DEBUG(a,b)
# define DEBUGLN(a,b)
# define DEBUG1LN(a)
#endif

#if defined(STM32L476xx)
# include "L476.h"
#elif defined(STM32F401xE)
# include "F401.h"
#elif defined(STM32L432xx)
# include "L432.h"
#else
# error "cette carte n'est pas encore supportée"
#endif

class HCSR04
{
	public:
		HCSR04(uint8_t pinTrigger, uint8_t pinEcho, TIM_TypeDef *tim=NULL);
		uint8_t begin(uint8_t PrescalerFactor=1);
		bool DistanceUpdated(void);
		bool IsObjectDetected(void);
		float DistanceinCm(void);
	private:
		uint32_t find( PinName pin, TIM_TypeDef *tim);
		uint64_t startTime;
		uint8_t _pinTrigger;
		uint8_t _pinEcho;
		TIM_TypeDef *_Instance;
};

#endif // hcsr04_h