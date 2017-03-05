/*Data acqusition station script
Armada Aeronautics Inc copyrighted
*/
//   I/O PINS
// Arduino Mega 2560
#define TRIMMER_PIN      A2

// LOAD CELLS
#define CELL_THRUST_DATA_PIN  27
#define CELL_THRUST_CLK_PIN   47
#define CELL_TORQUE_DATA_PIN  45
#define CELL_TORQUE_CLK_PIN   43
#define CELL_AUX_DATA_PIN     41
#define CELL_AUX_CLK_PIN      39

// RANGE SENSORS
#define TORQUE_RANGE_PIN      37
#define THRUST_RANGE_LO_PIN   35
#define THRUST_RANGE_HI_PIN   33

// MSGEQ
#define MSGEQ7_ANALOG_PIN     A15
#define MSGEQ7_RESET_PIN      31
#define MSGEQ7_STROBE_PIN     29

//#define DHT_PIN       6
#define RPM_PIN         49
// unusable for analogWrite: 6, 7, 8

#define THERMO_CS_PIN   10

#define SERVO0_PIN      11
#define DUBUG_LED_PIN   13
#define SPEAKER_PIN     12

#define ADXL345_CS_PIN  53
#define ADXL345_INT_PIN 2

// I2S PINS: used by ADS1115, BME280, PCA9685, ISL28022
// SDA A4
// SCL A5

// SPI PINS: used by ADXL345(through level shifter), Thermo
//      UNO   MEGA2560
// SS   10    53*
// MOSI 11    51
// MISO 12    50
// SCK  13    52

// ADS1115 PINS
#define VOLTAGE_PIN 0
#define CURRENT_PIN 1

// PORTS:
// I2S
//#define SERVO_ADR    0x40
#define MONITOR1_ADR 0x41
#define MONITOR2_ADR 0x42
#define ADS1115_ADR  0x48
//#define BME280_ADR   0x77

// SERVO
#define MOTOR0_CHANNEL 0
#define SERVO_DEFAULT_FREQUENCY 53.7F

#include "crc16.h"
#include "Arduinotypes.h"
#include "eulerdata.h"
#include "serialutils.h"
#include "eeprom_utils.h"
#include "modules.h"
#include "diagnostics.h"                    

static DeviceInfo host;
static EepromData eepromData;
static Modules    modules;


//===============================  EXPERIMENT CLASS

class Experiment
{
public:
  Experiment()
  {
    reset();
    mStarted = false;
  }

  ~Experiment()
  {
    stopExperiment();
  }

#include "static.h"
#include "dynamic.h"

  bool  startExperiment()
  { // returns false to stop experiment
    mStarted = true;
    tone(SPEAKER_PIN, 1500, 300);      // an experiment start beep
    delay(300);
    modules.startExperiment();
    mDynamicOutput.reset(true/*resetDebugAndFlags*/); // reset flags and debug data
    uint16_t stepNumber = 0;
    if(mInput.quantTime == 0) return false;
    uint16_t zeroQuants = mInput.getZeroMeasureQuants();

    if(mInput.isInCalibrationMode())
    {
      // CALIBRATION
      for(int i = 0; i < mInput.stepMeasureQuants; ++i) // 1 hour of cycles: static raw, static calib, dynamic raw, dynamic calib
      {
          // static
        if(!measureStatic(mInput.manualServo)) return false; // measure and get average raw values
        if(!exCmdStaticOutputData(false/*async*/)) return false; // send raw static data
        applyCalibration(mStaticOutput);
        if(!exCmdStaticOutputData(false/*async*/)) return false; // send calibrated static data

        // dynamic
        if(!servoStep(mInput.manualServo, stepNumber++, 0/*settleQuants*/, 1/*measureQuants*/, 1/*measureTimes*/)) return false;
      }
    }
    else 
    {
      // STATIC
      if(!measureStatic(0)) return false;
      applyCalibration(mStaticOutput);
      if(!exCmdStaticOutputData(false/*async*/)) return false;
  
      // DELAYED START
      mZeroLoadCellData.reset();
      int delayedStartQuants = mInput.delayedStart * 1000 / mInput.quantTime;
      if(delayedStartQuants > 0)
      {
        mDynamicOutput.flags |= dofDelayedStart; // mark that we are delaying start
        if(!servoStep(0, stepNumber++, delayedStartQuants/*settleQuants*/, 0/*measureQuants*/, 0/*measureTimes*/)) return false;    // delayed start
        mDynamicOutput.flags &= ~dofDelayedStart;
      }

      // ZERO STEP
      mExperimentStartTime = millis(); // reset timeStamp timer
      if(!servoStep(0, stepNumber++, 1/*settleQuants*/, zeroQuants/*measureQuants*/, 1/*measureTimes*/)) return false;    // initial zero servo
      storeZeroValues();
  
      // DYNAMIC
      if(mInput.isManualServo())
      {
        if(!servoStep(mInput.manualServo, stepNumber++, 0/*settleQuants*/, mInput.stepMeasureQuants/*measureQuants*/, 1/*measureTimes*/))
            return false;
      }
      else
      {
        bool up = mInput.servoEnd >= mInput.servoStart;
        if(up)
        {
          for(int32_t servo = mInput.servoStart; servo <= mInput.servoEnd; servo += mInput.servoStep)
          {
            uint16_t times = ((servo == mInput.servoEnd) ? mInput.endMeasurements : mInput.startMeasurements);
            if(!servoStep((uint16_t)servo, stepNumber++, mInput.stepSettleQuants, mInput.stepMeasureQuants, times))
                break;
          }
        }
        else  // down
        {
         for(int32_t servo = mInput.servoStart; servo >= mInput.servoEnd ; servo -= mInput.servoStep)
          {
            uint16_t times = ((servo == mInput.servoEnd) ? mInput.endMeasurements : mInput.startMeasurements);
            if(!servoStep((uint16_t)servo, stepNumber++, mInput.stepSettleQuants, mInput.stepMeasureQuants, times))
                break;
          }
        }
      }
    }

    uint16_t stopSettleQuants = mInput.getStopSettleQuants();
    if(!servoStep(0, stepNumber++, stopSettleQuants/*settleQuants*/, 1/*measureQuants*/, 1/*measureTimes*/)) return false;   // final zero servo
    return true;
  }

  void stopExperiment()
  {
    if(!mStarted) return;
    noTone(SPEAKER_PIN);                   // stopping experiment start beep
#ifdef PCA9685_MODULE    
    modules.servo.setPulseWidthUs(MOTOR0_CHANNEL, 0);
#endif    
    mStarted = false;
    Serial.flush();
    modules.stopExperiment();

    if(eepromData.writeMarked())
      tone(SPEAKER_PIN, 2000, 400);

    if(mDynamicOutput.flags & dofLimitExceeded)
      beeps(5, 100, SPEAKER_PIN);
      
    //Serial.end();
    delay(1000);
    software_Reset();
  }

  bool isStarted() const {return mStarted;}

  void software_Reset() // Restarts program from beginning but does not reset the peripherals and registers
  {
     asm volatile ("  jmp 0"); 
  }  
 
#include "excmd.h"

//////////

  bool waitAndDispatchCommand()
  {
      while(Serial.available()==0)
        delay(1);
      //blinkLED(2);

      Command cmd;
      if(!readCommand(cmd)) return false;
      bool async = mInput.isAsyncOutput();
      return dispatchCommand(cmd, async);
  }

  bool dispatchCommand(Command cmd, bool async)
  {
      switch(cmd)
      {
        case cmdReset                 : return exCmdReset                 (async); break;
        case cmdStop                  : return exCmdStop                  (async); break;
        case cmdDataInfo              : return exCmdDataInfo              (async); break;
        case cmdInputData             : return exCmdInputData             (async); break;
        case cmdStaticOutputData      : return exCmdStaticOutputData      (async); break;
        case cmdDynamicOutputInterData: return exCmdDynamicOutputInterData(async); break;
        case cmdDynamicOutputFinalData: return exCmdDynamicOutputFinalData(async); break;
        case cmdFinished              : return exCmdFinished              (async); break;
        case cmdCalibration           : return exCmdCalibration           (async); break;
        case cmdInputFlags            : return exCmdInputFlags            (async); break;
        case cmdOutputFlags           : return exCmdOutputFlags           (async); break;
        case cmdServo                 : return exCmdServo                 (async); break;
        case cmdError                 : return exCmdError                 (async); break;
        case cmdInfo                  : return exCmdInfo                  (async); break;
        case cmdLoopStart             : return exCmdLoopStart             (async); break;
        case cmdInit                  : return exCmdInit                  (async); break;
        default                       : return false;
      };
  }

  void stopOnError()
  {
    bool async = mInput.isAsyncOutput();
    exCmdStop(async);
  }

  bool calibrate(uint8_t parameter, float32_t newValue, uint8_t flags)
  {
    if(parameter >= cpCount)
        eepromData.reset();
    else
    {
        if(isnan(newValue))
            return false;
        if(flags & Calibration::cfShift)
            eepromData.setShift((CalibratedParameter)parameter, newValue);
        else
            eepromData.setScale((CalibratedParameter)parameter, newValue);
    }

    eepromData.markToWrite();
    return true;
  }

void  reset()
  {
    mInput           .reset();
    mStaticOutput    .reset();
    mDynamicOutput   .reset();
    mLoadCellData    .reset();
    mZeroLoadCellData.reset();
    mOutputInterSamples  = 0;
    mOutputFinalSamples  = 0;
    mLastError           = 0;
    mLastErrorData       = 0;
    mFinishErrorMask     = 0;
    mExperimentStartTime = 0;
  }

private:
  bool                 mStarted;
  InputData            mInput;
  StaticOutputData     mStaticOutput;
  DynamicOutputData    mDynamicOutput;
  LoadCellData         mLoadCellData;
  LoadCellData         mZeroLoadCellData;
  ZeroOutputData       mZeroOutputData;
  StaticAccumulator    mStaticAccumulator;
  DynamicAccumulator   mDynamicAccumulator;
  uint32_t             mOutputInterSamples; // ?
  uint32_t             mOutputFinalSamples;
  uint32_t             mFinishErrorMask;
  uint8_t              mLastError;
  uint32_t             mLastErrorData;
  uint32_t             mExperimentStartTime;
}; // class Experiment

static Experiment experiment;

bool waitForInit()
{
  while(true)
  {
    while(Serial.available()==0)
      delay(100);
    if(Serial.read() == cmdInit)
    {
      //blinkLED(3);
      return (Serial.write(cmdInit) == 1);
    }
  }
}

 
//////////// HARDWARE WRITES

void configureHardware()
{
  //pinMode(NOISE_PIN       , INPUT);
  pinMode(TRIMMER_PIN     , INPUT);
  pinMode(SPEAKER_PIN     , OUTPUT);
  pinMode(DUBUG_LED_PIN   , OUTPUT);

  pinMode(TORQUE_RANGE_PIN   , INPUT);
  pinMode(THRUST_RANGE_LO_PIN, INPUT);
  pinMode(THRUST_RANGE_HI_PIN, INPUT);
  digitalWrite(TORQUE_RANGE_PIN   , HIGH);
  digitalWrite(THRUST_RANGE_LO_PIN, HIGH);
  digitalWrite(THRUST_RANGE_HI_PIN, HIGH);
  
  eepromData.readData();
  modules.setPinModes();
  modules.setup();
  experiment.writeServo(0);
}

void setup()
{
  Serial.setTimeout(STATION_SERIAL_TIMEOUT);
  Serial.begin(SERIAL_BAUD_RATE);
  configureHardware();

  while(!Serial)
  {
    ; // wait for connection to establish
  }
  //serialRepeaterBlinker();
  //blinkLED(3);
  //serialRepeaterBlinkerBytes();
  
  delay(500);
  waitForInit();
}

void loop()
{
  experiment.waitAndDispatchCommand();
}
