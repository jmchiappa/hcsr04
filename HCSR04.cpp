#include "HCSR04.h"

#include "pinmap.h"
#include "pinconfig.h"

HardwareTimer *MyTim;
float Distance;
uint32_t channelRising, channelFalling;
volatile uint32_t StartHighLevel, HighStateMeasured;
bool CaptureInProgress, ObjectDetected,newValue;
uint32_t input_freq = 0;

/***** global timer callback ******/
/**
    @brief  Input capture interrupt callback : Compute frequency and dutycycle of input signal
*/
void TIMINPUT_Capture_Rising_IT_callback() {
  if(CaptureInProgress) {
    MyTim->setCount(0);
    // StartHighLevel = MyTim->getCaptureCompare(channelRising);
  }  
}

/* In case of timer rollover, frequency is to low to be measured set values to 0
   To reduce minimum frequency, it is possible to increase prescaler. But this is at a cost of precision. */
void Rollover_IT_callback() {
  if(CaptureInProgress) {
    ObjectDetected = false;
    CaptureInProgress = false;
    newValue=true;
  }
}

/**
    @brief  Input capture interrupt callback : Compute frequency and dutycycle of input signal
*/
void TIMINPUT_Capture_Falling_IT_callback() {
  
  if( CaptureInProgress ) {
    ObjectDetected = true;
    CaptureInProgress = false;
  /* prepare DutyCycle computation */
    uint32_t CurrentCapture = MyTim->getCaptureCompare(channelFalling);
    // HighStateMeasured = CurrentCapture - StartHighLevel;
    Distance = (float)CurrentCapture / (float)input_freq;
    Distance = Distance / 58.8f;
    newValue=true;
  }
}

HCSR04::HCSR04(uint8_t pinTrigger, uint8_t pinEcho, TIM_TypeDef *tim) {
	this->_pinTrigger = pinTrigger;
	this->_pinEcho = pinEcho;
	this->_Instance = tim;
}

uint8_t HCSR04::begin(uint8_t PrescalerFactor)
{
	PinName pin = digitalPinToPinName(_pinEcho);
	DEBUG1LN("entree begin()");
  // Automatically retrieve TIM instance and channelRising associated to pin
  // This is used to be compatible with all STM32 series automatically.
  if(_Instance==NULL) {
  	DEBUG1LN("_Instance est NULL, on cherche le timer adapté");
	  _Instance = (TIM_TypeDef *)pinmap_peripheral(pin, PinMap_TMR);
	  channelRising = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_TMR));
	} else {
  	DEBUG1LN("_Instance n'est pas nul, on cherche le couple qui fonctionne");
		channelRising = find(pin,_Instance);
    DEBUGLN("channel :", channelRising);
		if(channelRising == NP)	{
			return 0;
		}
	}
  // channelRisings come by pair for TIMER_INPUT_FREQ_DUTY_MEASUREMENT mode:
  // channelRising1 is associated to channelFalling and channelRising3 is associated with channelRising4
  switch (channelRising) {
    case 1:
      channelFalling = 2;
      break;
    case 2:
      channelFalling = 1;
      break;
    case 3:
      channelFalling = 4;
      break;
    case 4:
      channelFalling = 3;
      break;
  }

  // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup() function is finished.
  MyTim = new HardwareTimer(_Instance);

  // Configure rising edge detection to measure frequency
  MyTim->setMode(channelRising, TIMER_INPUT_FREQ_DUTY_MEASUREMENT, _pinEcho);

  // With a PrescalerFactor = 1, the minimum frequency value to measure is : TIM counter clock / CCR MAX
  //  = (SystemCoreClock) / 65535
  // Example on Nucleo_L476RG with systemClock at 80MHz, the minimum frequency is around 1,2 khz
  // To reduce minimum frequency, it is possible to increase prescaler. But this is at a cost of precision.
  // The maximum frequency depends on processing of both interruptions and thus depend on board used
  // Example on Nucleo_L476RG with systemClock at 80MHz the interruptions processing is around 10 microseconds and thus Max frequency is around 100kHz
  MyTim->setPrescaleFactor(PrescalerFactor);
  MyTim->setOverflow(0x10000); // Max Period value to have the largest possible time to detect rising edge and avoid timer rollover
  MyTim->attachInterrupt(channelRising, TIMINPUT_Capture_Rising_IT_callback);
  MyTim->attachInterrupt(channelFalling, TIMINPUT_Capture_Falling_IT_callback);
  MyTim->attachInterrupt(Rollover_IT_callback);

  MyTim->resume();

  // Compute this scale factor only once
  input_freq = MyTim->getTimerClkFreq() / (1e6*MyTim->getPrescaleFactor() );
  ObjectDetected = false;
  CaptureInProgress = false;
  pinMode(this->_pinTrigger, OUTPUT);
  digitalWrite(this->_pinTrigger,LOW);
	DEBUG1LN("fin begin()");
  return !0;
}

bool HCSR04::DistanceUpdated(void) {
	bool ret= newValue;
	if(!CaptureInProgress) {
    DEBUG("début:",StartHighLevel);
    DEBUG("\tdurée :",HighStateMeasured);
    DEBUG("\tobjet :",ObjectDetected);
    DEBUGLN("\tdistance :",Distance);
		if(startTime==0){
			startTime = millis();
			DEBUG1LN("début chrono");
			// donot chnge status for 100 ms
		}else {
	  	if(startTime+50<millis()) {
				DEBUG1LN("début acqui");
	  		startTime=0;
	  		newValue=false;
		    ObjectDetected = false;
		    // _Instance->CNT=0;
        MyTim->setCount(0);
		    CaptureInProgress = true;
		    digitalWrite(_pinTrigger,HIGH);
		    delayMicroseconds(10);
		    digitalWrite(_pinTrigger,LOW);
		  }
		}
	}
	return ret;
}

bool HCSR04::IsObjectDetected(void) {
	newValue=false;
	return ObjectDetected;
}

float HCSR04::DistanceinCm(){
	if(!CaptureInProgress) {
    newValue = false; //clear "new value flag to prevent from reading again DistanceUpdated() = true
		return Distance;
	}
	else {
		return -1;
	}
}

uint32_t HCSR04::find( PinName pin, TIM_TypeDef *tim) {
	uint32_t i=0;
  while (PinMap_TMR[i].pin != NC) {
  	DEBUG("\tpin en cours   : ",PinMap_TMR[i].pin == pin);
  	// DEBUG("\tpériph a chercher : ",(uint32_t)tim);
  	DEBUGLN("\tpin en cours   : ",(uint32_t)PinMap_TMR[i].peripheral == (uint32_t)tim);
    if ((PinMap_TMR[i].pin == pin) && ((uint32_t)PinMap_TMR[i].peripheral == (uint32_t)tim) ) {
      return STM_PIN_CHANNEL(PinMap_TMR[i].function);
    }
    i++;
  }
  return NP;
}