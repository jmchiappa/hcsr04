#include "HCSR04.h"

#include "pinmap.h"
#include "pinconfig.h"

#include <HardwareTimer.h>

#define HAL_TIM_MODULE_ENABLED

#if defined(HAL_TIM_MODULE_ENABLED) && !defined(HAL_TIM_MODULE_ONLY)

static hcsr04_t HCSR04sensors[MAX_HCSR04_SENSORS];  // static array of servo structures

uint8_t HCSR04SensorCount = 0;    // the total number of attached sensors
uint8_t HCSR04SensorIndex = 0;    // the current index of attached sensors
bool HCSR04EchoCanceller = false; // true when a delay of 10 ms is in progress
// moved to sensor structure
// HardwareTimer *MyTim;
// float Distance;
// uint32_t channelRising, channelFalling;
volatile uint32_t StartHighLevel, HighStateMeasured;
// bool CaptureInProgress, ObjectDetected,newValue;
// uint32_t input_freq = 0;

#define HCSR04CurrentSensor (HCSR04sensors[HCSR04SensorIndex])

/**
 *  global definitions
 */
void HCSR04TimerReinit(uint8_t index);
void StartEchoCancelling(void);

/***** global timer callback ******/
/**
    @brief  Input capture interrupt callback : Compute frequency and dutycycle of input signal
*/
void TIMINPUT_Capture_Rising_IT_callback() {
  if(HCSR04CurrentSensor.CaptureInProgress) {
    StartHighLevel = HCSR04CurrentSensor.MyTim->getCaptureCompare(HCSR04CurrentSensor.Pin.channelRising);
  }  
}

/* In case of timer rollover, frequency is to low to be measured set values to 0
   To reduce minimum frequency, it is possible to increase prescaler. But this is at a cost of precision. */
void Rollover_IT_callback() {
  if(!HCSR04EchoCanceller) {
    if(HCSR04CurrentSensor.CaptureInProgress) {
      LEDoff();
      HCSR04CurrentSensor.ObjectDetected = false;
      HCSR04CurrentSensor.CaptureInProgress = false;
      HCSR04CurrentSensor.newValue=true;
      HCSR04SensorIndex++;
      HCSR04SensorIndex %=HCSR04SensorCount;
      StartEchoCancelling();
      // HCSR04TimerReinit(HCSR04SensorIndex);
    }
  }else{
    // DEBUG1LN("fin du canceller");
    // this is the 10 ms overflow, so :
    // 1. stop the current timer to prevent next IT
    HCSR04CurrentSensor.MyTim->pause();
    // 2. clear the echo canceller flag
    HCSR04EchoCanceller = false;
    // 3. start the next sensor
    HCSR04SensorIndex++;
    HCSR04SensorIndex %=HCSR04SensorCount;
    HCSR04TimerReinit(HCSR04SensorIndex);
  }
}

/**
    @brief  Input capture interrupt callback : Compute frequency and dutycycle of input signal
*/
void TIMINPUT_Capture_Falling_IT_callback() {
  float distance_;
  if(!HCSR04EchoCanceller) {
    if( HCSR04CurrentSensor.CaptureInProgress ) {
      HCSR04CurrentSensor.ObjectDetected = true;
      HCSR04CurrentSensor.CaptureInProgress = false;
    /* prepare DutyCycle computation */
      uint32_t CurrentCapture = HCSR04CurrentSensor.MyTim->getCaptureCompare(HCSR04CurrentSensor.Pin.channelFalling);
      HighStateMeasured = CurrentCapture - StartHighLevel;
      distance_ = (float)HighStateMeasured / (float)HCSR04CurrentSensor.input_freq;
      HCSR04CurrentSensor.Distance = distance_ / 58.8f;
      HCSR04CurrentSensor.newValue = true;
      StartEchoCancelling();
    }
  }
}
/**
 * static declarations
 */
void HCSR04TimerReinit(uint8_t index) {
  hcsr04_t *sensor = &HCSR04sensors[index];
  HardwareTimer *MyTim = sensor->MyTim;
  MyTim->pause();
  MyTim->setMode(sensor->Pin.channelRising, TIMER_INPUT_FREQ_DUTY_MEASUREMENT, sensor->Pin.echo);
  // With a PrescalerFactor = 1, the minimum frequency value to measure is : TIM counter clock / CCR MAX
  //  = (SystemCoreClock) / 65535
  // Example on Nucleo_L476RG with systemClock at 80MHz, the minimum frequency is around 1,2 khz
  // To reduce minimum frequency, it is possible to increase prescaler. But this is at a cost of precision.
  // The maximum frequency depends on processing of both interruptions and thus depend on board used
  // Example on Nucleo_L476RG with systemClock at 80MHz the interruptions processing is around 10 microseconds and thus Max frequency is around 100kHz
  MyTim->setPrescaleFactor(sensor->Prescalar);
  MyTim->setOverflow(0x10000); // Max Period value to have the largest possible time to detect rising edge and avoid timer rollover
  MyTim->setCount(0);
  pinMode(sensor->Pin.trigger, OUTPUT);
  digitalWrite(sensor->Pin.trigger,HIGH);
  delayMicroseconds(10);
  digitalWrite(sensor->Pin.trigger,LOW);
  sensor->CaptureInProgress = true;
  MyTim->refresh();
  MyTim->resume();
}

void StartEchoCancelling() {
  // Start the echo canceller :
  // 1. set the current timer to 1 ms of period
  // 2. set the overflow to 10
  // 3. start the timer
  HCSR04EchoCanceller = true;
  // HCSR04CurrentSensor.MyTim->detachInterrupt(HCSR04CurrentSensor.Pin.channelFalling);
  // HCSR04CurrentSensor.MyTim->detachInterrupt(HCSR04CurrentSensor.Pin.channelRising);
  HCSR04CurrentSensor.MyTim->pause();
  HCSR04CurrentSensor.MyTim->setMode(1, TIMER_OUTPUT_COMPARE);
  HCSR04CurrentSensor.MyTim->setOverflow(50,HERTZ_FORMAT); // next overflow set to 1s
  // HCSR04CurrentSensor.MyTim->attachInterrupt(Rollover_IT_callback);
  HCSR04CurrentSensor.MyTim->setCount(0);
  HCSR04CurrentSensor.MyTim->refresh();
  HCSR04CurrentSensor.MyTim->resume();
}

HCSR04::HCSR04(uint8_t pinTrigger, uint8_t pinEcho, TIM_TypeDef *tim) {
  if(HCSR04SensorCount < MAX_HCSR04_SENSORS) {
    this->_pinTrigger = pinTrigger;
    this->_pinEcho = pinEcho;
    this->_Instance = tim;
    this->Index = HCSR04SensorCount++;
    HCSR04SensorIndex = this->Index;
    // Initialize the current structure
    HCSR04CurrentSensor.Instance = tim;
    HCSR04CurrentSensor.Pin.trigger = pinTrigger;
    HCSR04CurrentSensor.Pin.echo = pinEcho;
    HCSR04CurrentSensor.CaptureInProgress = false;
    HCSR04CurrentSensor.ObjectDetected = false;
    HCSR04CurrentSensor.newValue = false;
    HCSR04CurrentSensor.tickValue = 0;
    HCSR04CurrentSensor.input_freq = 0;
    HCSR04CurrentSensor.Distance = -1;
  }
  else{
    this->Index = INVALID_HANDLER;
  }
}

uint8_t HCSR04::begin(uint8_t PrescalerFactor)
{

  uint32_t channelRising, channelFalling;

  hcsr04_t *sensor = &HCSR04sensors[this->Index];
	PinName pin = digitalPinToPinName(_pinEcho);
	DEBUG1LN("entree begin()");
  // Automatically retrieve TIM instance and channelRising associated to pin
  // This is used to be compatible with all STM32 series automatically.
  if(this->_Instance==NULL) {
  	DEBUG1LN("_Instance est NULL, on cherche le timer adapté");
	  _Instance = (TIM_TypeDef *)pinmap_peripheral(pin, PinMap_TMR);
	  channelRising = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_TMR));
    sensor->Instance = this->_Instance;
	} else {
  	DEBUG1LN("_Instance n'est pas nul, on cherche le couple qui fonctionne");
		channelRising = find(pin,_Instance);
    DEBUGLN("channel :", channelRising);
		if(channelRising == NP)	{
			return 0;
		}
	}
  // channelRisings come by pair for TIMER_INPUT_FREQ_DUTY_MEASUREMENT mode:
  // channelRising1 is associated to channelFalling2 and channelRising3 is associated with channelRising4
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
  sensor->Pin.channelRising = channelRising;
  sensor->Pin.channelFalling = channelFalling;

  // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup() function is finished.
  MyTim = new HardwareTimer(_Instance);
  sensor->MyTim = MyTim;
  sensor->Prescalar = PrescalerFactor;
  // Compute this scale factor only once
  // Configure rising edge detection to measure frequency
  MyTim->attachInterrupt(sensor->Pin.channelRising, TIMINPUT_Capture_Rising_IT_callback);
  MyTim->attachInterrupt(sensor->Pin.channelFalling, TIMINPUT_Capture_Falling_IT_callback);
  MyTim->attachInterrupt(Rollover_IT_callback);
  HCSR04TimerReinit(this->Index);
  sensor->input_freq = MyTim->getTimerClkFreq() / (1e6*MyTim->getPrescaleFactor() );
  return !0;
}

bool HCSR04::DistanceUpdated(void) {
  hcsr04_t *sensor = &HCSR04sensors[this->Index];
	bool ret=sensor->newValue;
	if(sensor->newValue) {
    DEBUG("index:",this->Index);
    DEBUG("début:",StartHighLevel);
    DEBUG("\tdurée :",HighStateMeasured);
    DEBUG("\tobjet :",sensor->ObjectDetected);
    DEBUGLN("\tdistance :",sensor->Distance);
    // sensor->newValue=false;
    // sensor->ObjectDetected = false;
	}
	return ret;
}

bool HCSR04::IsObjectDetected(void) {
  bool ret = HCSR04sensors[this->Index].ObjectDetected;
  HCSR04sensors[this->Index].newValue = false;
  HCSR04sensors[this->Index].ObjectDetected = false;
	return ret;
}

float HCSR04::DistanceinCm(){
	// if(!HCSR04sensors[this->Index].CaptureInProgress) {
		return HCSR04sensors[this->Index].Distance;
	// }
	// else {
	// 	return -1;
	// }
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

#endif
