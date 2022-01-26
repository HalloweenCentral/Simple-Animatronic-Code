#include <VarSpeedServo.h> 
 
VarSpeedServo servo;
# define PinThing1       12
# define PinThing2       13
enum states
{ 
  WAIT_FOR_TRIGGER,
  MOVE_TO_160_DEGREES,
  WAIT1,
  MOVE_TO_60_DEGREES,
  WAIT2,
  WAIT3,
  MOVE_TO_90_DEGREES,
};

const unsigned long wait1Period = 5000;
const unsigned long wait2Period = 5000;
const unsigned long wait3Period = 5000;
const unsigned long movementPeriod = 30000;
unsigned long currentTime;
unsigned long shortPeriodStartTime;
unsigned long movementPeriodStartTime;

byte currentState = WAIT_FOR_TRIGGER;
const byte triggerPin = 9;
const int servoPin = 8;
int pos = 90; 
void setup()
{
  Serial.begin(115200);
  pinMode(triggerPin, INPUT_PULLUP);
  Serial.println("waiting for trigger");
  servo.attach(8);
    digitalWrite (PinThing1,    LOW);
    pinMode(PinThing2, OUTPUT);
    pinMode(PinThing1, OUTPUT);
}
// -----------------------------------------------------------------------------
struct Timer {
    byte           pin;
    unsigned long  duration;
    bool           on;
};

Timer timers [] = {
    { PinThing1, 27000 },
    { PinThing2,    27000 }
};

#define N_TIMERS    (sizeof(timers)/sizeof(Timer))

// -----------------------------------------------------------------------------
void loop()
{static unsigned long msecLst;
           unsigned long msec = millis ();

    // enable timed outputs
    if (digitalRead (triggerPin) == LOW)  {
        msecLst = msec;
        for (unsigned n = 0; n < N_TIMERS; n++)  {
            timers [n].on = true;
            digitalWrite (timers [n].pin, HIGH);
        }
    }

    // turn off timed outputs when duration expired
    Timer *t = timers;
    for (unsigned n = 0; n < N_TIMERS; n++, t++)  {
        if (t->on && (msec - msecLst) > t->duration)  {
            digitalWrite (t->pin, LOW);
            t->on = false;
        }
    }

   
  currentTime = millis();
  switch (currentState)
  {
    case WAIT_FOR_TRIGGER:
      if (digitalRead(triggerPin) == LOW)
      {
        currentState = MOVE_TO_160_DEGREES;
        movementPeriodStartTime = currentTime;
      }
      break;
    case MOVE_TO_160_DEGREES:
      Serial.println("move to 160 degrees");  
    servo.write(160,10,true); 
      shortPeriodStartTime = currentTime;
      currentState = WAIT1;
    
      break;
    case WAIT1:
      if (currentTime - shortPeriodStartTime >= wait1Period)
      {
        currentState = MOVE_TO_60_DEGREES;
      }
      checkMovementPeriod();
      break;
    case MOVE_TO_60_DEGREES:
      Serial.println("move to 60 degrees"); 
      servo.write(60,10,true);
      shortPeriodStartTime = currentTime;
      currentState = WAIT3;
      break;
        case WAIT3:
      if (currentTime - shortPeriodStartTime >= wait3Period)
      {
        currentState = MOVE_TO_90_DEGREES;
      }
      checkMovementPeriod();
      break;
    case MOVE_TO_90_DEGREES:
      Serial.println("move to 90 degrees"); 
      servo.write(90,10,true);
      shortPeriodStartTime = currentTime;
      currentState = WAIT2;
      break;   
    case WAIT2:
      if (currentTime - shortPeriodStartTime >= wait1Period)
      {
        currentState = MOVE_TO_160_DEGREES;
      }
      checkMovementPeriod();
      
      break;
      
  };
 
}

void checkMovementPeriod()
{
  if (currentTime - movementPeriodStartTime >= movementPeriod)
  {
    Serial.println("movement  period ended");
    Serial.println("waiting for trigger");
    currentState = WAIT_FOR_TRIGGER;
  }
}
