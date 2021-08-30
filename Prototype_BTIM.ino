#include <SoftwareSerial.h>

//------------------------------------------------------------------------------------

//Tach defines
const int TACH_PIN = 8;             //Tach input pin (typically resistor pull-up to 3.3V)
float currentTachTime = 0;          //Tracking time between tach pings
float lastTachTime = 0;             //Tracking time between tach pings
int tachCount = 0;                  //Tracking tach pings per revolution (typically 2 per rev)
int tachNoCount = 0;                //Tracking when no tach pings
float notrpm = 0;                   //Time between pings
float rpm = 0;                      //Time between pings converted to rpm
static int currentTachState = LOW;  //Tracking tack input state
static int lastTachState = LOW;     //Tracking tack input state
int rpmAvgTotal = 0;
int rpmAvgCounter = 0;
float rpmAvg = 0;

//Thermistor defines
#define THERM1 A0
#define THERM2 A1
float thermValOne = 0;
float thermValTwo = 0;

//IO Define
//Compressor
//Purple wire
//Relay 4 pin 7
//Control character
#define COMPRESSORCHAR 'c'
int compressorState = 0;

//Water Pump
//Yellow wire
//Relay 3 pin 6
//Control character
#define WATERPUMPCHAR 'p'
int waterpumpState = 0;

//Hot Gas Valve
//Lightblue wire
//Relay 2 pin 5
//Control character
#define HOTGASCHAR 'h'
int hotgasState = 0;

//Water Valve
//Darkblue wire
//Relay 1 pin 4
//Control character
#define WATERVALVECHAR 'w'
int watervalveState = 0;

//Fan
//Control character
#define FANCHAR 'f'
int fanDutyCycle = 90;
int customDutyCycle = 0;

//Freeze
//Control character
#define FREEZECHAR 'e'

//Harvest
//Control character
#define HARVESTCHAR 't'

//Fill
//Control character
#define FILLCHAR 'l'

//Stop
//Control character
#define STOPCHAR 's'

//Go
//Control character
#define GOCHAR 'g'

//Skip (allows you to skip a state or rather immediately max out state timer)
//Control character
#define SKIPCHAR 'k'

//Main state
String mainStateString = "STOP";
//Main states
#define STOP 0
const String stopString = "STOP";
#define GO 1
const String goString = "GO";

//Secondary state
String secStateString = "BINFULL/IDLE/OFF";
//Seconadry States
#define BINFULL 0
const String binfullString = "BINFULL/IDLE/OFF";
float maxBinfullTimeMin = 1;
#define FREEZE 1
const String freezeString = "FREEZE";
float maxFreezeTimeMin = 35;
#define HARVEST 2
const String harvestString = "HARVEST";
float maxHarvestTimeMin = 5;
#define FILL 3
const String flushString = "FILLFLUSH";
float maxFillTimeMin = 2;

//Make ice state machine
char mainState = 0;
char secState = 0;

//State machine timer
float mainStateTimerMin = 0;      //Min in main state
float secStateTimerMin = 0;       //Min in secondary state
float timeInStateMin = 0;   //Trackercounter for minutes in state
float timeInStateSec = 0;   //Trackercounter for seconds in state

//Relay defines
#define RELAY1PIN 4 //Water valve
#define RELAY2PIN 5 //Hot gas valve
#define RELAY3PIN 6 //Water pump
#define RELAY4PIN 7 //Compressor

//BT defines
const int BT_RX_PIN = 10;
const int BT_TX_PIN = 11;
const int BT_BAUD_RATE = 9600;
SoftwareSerial bluetooth(BT_RX_PIN, BT_TX_PIN);

//PWM defines
//PWM Output PIN
const byte OC1A_PIN = 9;
//PWM Output Setup
const word PWM_FREQ_HZ = 25000; //Adjust this value to adjust the frequency
const word TCNT1_TOP = 16000000 / (2 * PWM_FREQ_HZ);

//Bin Ice level defines
int iceLevelAnalogPin = A0;
int digitalIceLevel = 0;
float digiToVolts = 0;
float voltsToInches = 0;

//Time
float timems = 0;
float timesec = 0;
float timemin = 0;

//Timed updates
float timeCheckA = 0;
#define UPDATETIMER 5000 //Time between auto updates 

//------------------------------------------------------------------------------------
void setup() {
  //Serial setup
  Serial.begin(9600);

  //BT serial setup
  bluetooth.begin(BT_BAUD_RATE);

  //Relay setup
  pinMode(RELAY1PIN, OUTPUT);
  pinMode(RELAY2PIN, OUTPUT);
  pinMode(RELAY3PIN, OUTPUT);
  pinMode(RELAY4PIN, OUTPUT);
  //Immediately set all to off
  digitalWrite(RELAY1PIN, LOW);
  digitalWrite(RELAY2PIN, LOW);
  digitalWrite(RELAY3PIN, LOW);
  digitalWrite(RELAY4PIN, LOW);

  //PWM setup
  //Set PWM pin as output
  pinMode(OC1A_PIN, OUTPUT);
  //Clear Timer1 control and count registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  TCCR1A |= (1 << COM1A1) | (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << CS10);
  ICR1 = TCNT1_TOP;

  //Set tach pin as input
  pinMode(TACH_PIN, INPUT);

  //Duty cycle setup
  setPwmDuty(fanDutyCycle);
}

//------------------------------------------------------------------------------------
void loop() {
  timems = millis();
  timesec = timems / 1000;
  timemin = timesec / 60;

  //Set ice level
  digitalIceLevel = analogRead(iceLevelAnalogPin);
  digiToVolts = float(digitalIceLevel);
  digiToVolts = ((digiToVolts / 1024) * 5);
  voltsToInches = (20.87 - (digiToVolts * 3.04));

  //Do therm related logic
  thermLogic();

  //Do tach related logic
  tachLogic();
  //
  //  rpmAvgTotal += rpm;
  //  rpmAvgCounter++;
  //  if(rpmAvgCounter >= 100) {
  //    rpmAvg = rpmAvgTotal/100;
  //    rpmAvgCounter = 0;
  //    rpmAvgTotal = 0;
  //  }

  //Timer for updating info over BT/Serial
  if (timems >= (timeCheckA + UPDATETIMER)) {
    //Reset check timer
    timeCheckA = timems;

    printInfo();

  }

  if (Serial.available()) {
    String serString = Serial.readStringUntil('\n');
    Serial.print("String is: ");
    Serial.println(serString);
    char serChar = serString[0];
    int serInt = (int)serChar - 48;
    if (serString.length() == 1) {
      if (serInt > 10) {
        inputStateChange(serChar);
      }
    }
    else if (serString.length() >= 3) {
      serString.remove(0, 2);
      serInt = serString.toInt();
      inputOther(serChar, serInt);
    }
    else {
      Serial.println("Bad input.");
      Serial.println("If attempting only state change simply send letter for component to-be-changed.");
      Serial.println("Input beyond state change should be in form of \"x yyy\" ");
      Serial.println("where x = command and yyy = number value between 0 and 100");
      bluetooth.println("Bad input.");
      bluetooth.println("If attempting only state change simply send letter for component to-be-changed.");
      bluetooth.println("Input beyond state change should be in form of \"x yyy\" ");
      bluetooth.println("where x = command and yyy = number value between 0 and 100");
    }
  }

  //Bluetooth cannot handle fan speed specific set currently due to readStringUntil function
  if (bluetooth.available()) {
    String btString = bluetooth.readStringUntil('\n');
    bluetooth.print("String is: ");
    bluetooth.println(btString);
    char btChar = btString[0];
    int btInt = (int)btChar - 48;
    if (btString.length() <= 2) {
      if (btInt > 10) {
        inputStateChange(btChar);
      }
    }
    else if (btString.length() >= 3) {
      btString.remove(0, 2);
      btInt = btString.toInt();
      inputOther(btChar, btInt);
    }
    else {
      Serial.println("Bad input.");
      Serial.println("If attempting only state change simply send letter for component to-be-changed.");
      Serial.println("Input beyond state change should be in form of \"x yyy\" ");
      Serial.println("where x = command and yyy = number value between 0 and 100");
      bluetooth.println("Bad input.");
      bluetooth.println("If attempting only state change simply send letter for component to-be-changed.");
      bluetooth.println("Input beyond state change should be in form of \"x yyy\" ");
      bluetooth.println("where x = command and yyy = number value between 0 and 100");
    }
  }

  stateMachLogic();

}

//------------------------------------------------------------------------------------
//State machine logic
void stateMachLogic() {

  if (mainState == GO) {
    mainStateString = goString;
    mainStateTimerMin += timemin - timeInStateMin;
    secStateTimerMin += timemin - timeInStateMin;

    //Freeze state handling
    if (secState == FREEZE) {
      secStateString = freezeString;

      //Special case suspend water for ~30sec at start of every freeze
      //TODO CLEANUP
      if (secStateTimerMin <= 0.5) {
        if (watervalveState == 0) {
          Serial.print("Topoff/suspend water for 30sec at start of freeze...");
          bluetooth.print("Topoff/suspend water for 30sec at start of freeze...");
          digitalWrite(RELAY1PIN, HIGH);
          watervalveState = 1;
        }
      }
      else {
        if (watervalveState == 1) {
          Serial.print("Topoff/suspend water complete.");
          bluetooth.print("Topoff/suspend water complete.");
          digitalWrite(RELAY1PIN, LOW);
          watervalveState = 0;
        }
      }

      if (secStateTimerMin >= maxFreezeTimeMin) {
        secState = HARVEST;
        inputStateChange(HARVESTCHAR);
        secStateTimerMin = 0;
      }
    }
    //Harvest state handling
    else if (secState == HARVEST) {
      secStateString = harvestString;
      if (secStateTimerMin >= maxHarvestTimeMin) {
        secState = FREEZE;
        inputStateChange(FREEZECHAR);
        secStateTimerMin = 0;

      }
    }
    //Flush (fill) handling state
    else if (secState == FILL) {
      secStateString = flushString;
      if (secStateTimerMin >= maxFillTimeMin) {
        secState = FREEZE;
        inputStateChange(FREEZECHAR);
        secStateTimerMin = 0;
      }
    }
  }
  else {
    mainStateString = stopString;
    mainStateTimerMin = 0;
    secStateTimerMin = 0;
  }
  timeInStateMin = timemin;
  timeInStateSec = timesec;
}

//Set PWM duty cycle
void setPwmDuty(byte duty) {
  OCR1A = (word) (duty * TCNT1_TOP) / 100;
}

//Thermistor logic
void thermLogic() {
  thermValOne = analogRead(THERM1);
  thermValTwo = 0;//analogRead(THERM2);
}

//Tach logic
void tachLogic() {
  currentTachState = digitalRead(TACH_PIN);
  currentTachTime = millis();
  //If previous state was low and now we're high (we've revolutioned)
  if (currentTachState == LOW && lastTachState == HIGH) {
    tachCount++;
    //Two pulses per revolution
    if (tachCount >= 2) {
      notrpm = currentTachTime - lastTachTime;
      rpm = 1 / ((notrpm / 1000) / 60);
      tachCount = 0;
      lastTachTime = currentTachTime;
    }
  }
  else {
    //If no tach state change then report 0 rpm and reset things
    tachNoCount++;
    if (tachNoCount > 1000)
    {
      tachNoCount = 0;
      notrpm = 0;
      rpm = 0;
    }
  }
  //Update tach state
  lastTachState = currentTachState;
}

void inputOther(char controlChar, int otherValue) {
  switch (controlChar) {


    case FANCHAR:
      if (otherValue <= 100 && otherValue >= 0) {
        fanDutyCycle = otherValue;
        if (mainState == GO) {
          customDutyCycle = otherValue;
        }
        setPwmDuty(fanDutyCycle);
        Serial.print("Fan duty set to: ");
        Serial.println(fanDutyCycle);
        bluetooth.print("Fan duty set to: ");
        bluetooth.println(fanDutyCycle);
      }
      else {
        Serial.println("Bad input.");
        Serial.println("If attempting only state change simply send letter for component to-be-changed.");
        Serial.println("Input beyond state change should be in form of \"x yyy\" ");
        Serial.println("where x = command and yyy = number value between 0 and 100");
        bluetooth.println("Bad input.");
        bluetooth.println("If attempting only state change simply send letter for component to-be-changed.");
        bluetooth.println("Input beyond state change should be in form of \"x yyy\" ");
        bluetooth.println("where x = command and yyy = number value between 0 and 100");
      }
      break;

    case FILLCHAR:
      if (otherValue <= 100 && otherValue >= 0) {
        maxFillTimeMin = otherValue;
        Serial.print("Fill state minute duration set to: ");
        Serial.println(maxFillTimeMin);
        bluetooth.print("Fill state minute duration set to: ");
        bluetooth.println(maxFillTimeMin);
      }
      else {
        Serial.println("Bad input.");
        Serial.println("If attempting only state change simply send letter for component to-be-changed.");
        Serial.println("Input beyond state change should be in form of \"x yyy\" ");
        Serial.println("where x = command and yyy = number value between 0 and 100");
        bluetooth.println("Bad input.");
        bluetooth.println("If attempting only state change simply send letter for component to-be-changed.");
        bluetooth.println("Input beyond state change should be in form of \"x yyy\" ");
        bluetooth.println("where x = command and yyy = number value between 0 and 100");
      }
      break;

    case FREEZECHAR:
      if (otherValue <= 100 && otherValue >= 0) {
        maxFreezeTimeMin = otherValue;
        Serial.print("Freeze state minute duration set to: ");
        Serial.println(maxFreezeTimeMin);
        bluetooth.print("Freeze state minute duration set to: ");
        bluetooth.println(maxFreezeTimeMin);
      }
      else {
        Serial.println("Bad input.");
        Serial.println("If attempting only state change simply send letter for component to-be-changed.");
        Serial.println("Input beyond state change should be in form of \"x yyy\" ");
        Serial.println("where x = command and yyy = number value between 0 and 100");
        bluetooth.println("Bad input.");
        bluetooth.println("If attempting only state change simply send letter for component to-be-changed.");
        bluetooth.println("Input beyond state change should be in form of \"x yyy\" ");
        bluetooth.println("where x = command and yyy = number value between 0 and 100");
      }
      break;

    case HARVESTCHAR:
      if (otherValue <= 100 && otherValue >= 0) {
        maxHarvestTimeMin = otherValue;
        Serial.print("harvest state minute duration set to: ");
        Serial.println(maxHarvestTimeMin);
        bluetooth.print("Harvest state minute duration set to: ");
        bluetooth.println(maxHarvestTimeMin);
      }
      else {
        Serial.println("Bad input.");
        Serial.println("If attempting only state change simply send letter for component to-be-changed.");
        Serial.println("Input beyond state change should be in form of \"x yyy\" ");
        Serial.println("where x = command and yyy = number value between 0 and 100");
        bluetooth.println("Bad input.");
        bluetooth.println("If attempting only state change simply send letter for component to-be-changed.");
        bluetooth.println("Input beyond state change should be in form of \"x yyy\" ");
        bluetooth.println("where x = command and yyy = number value between 0 and 100");
      }
      break;

    default:
      Serial.println("Command character does not support serial/bluetooth value updates, sorry.");
      bluetooth.println("Command character does not support serial/bluetooth value updates, sorry.");
      break;
  }
}

//Printout current state info on a timer
void printInfo() {
  Serial.println("-------------------------------------------");
  bluetooth.println("-----------------------------");

  //Output bin level state
  Serial.print("   Ice level volts is approx: ");
  Serial.println(digiToVolts);
  bluetooth.print("   Ice level volts is approx: ");
  bluetooth.println(digiToVolts);
  Serial.print("   Ice inches from sensor is approx: ");
  Serial.println(voltsToInches);
  bluetooth.print("   Ice inches from sensor is approx: ");
  bluetooth.println(voltsToInches);

  //Output thermistors
  Serial.print("   Therm1 approx temp F: ");
  Serial.println(thermValOne);
  bluetooth.print("   Therm1 approx temp F: ");
  bluetooth.println(thermValOne);
  Serial.print("   Therm2 approx temp F: ");
  Serial.println(thermValTwo);
  bluetooth.print("   Therm2 approx temp F: ");
  bluetooth.println(thermValTwo);

  //Output IO states
  Serial.print("   Compressor state: ");
  Serial.println(compressorState);
  bluetooth.print("   Compressor state: ");
  bluetooth.println(compressorState);

  Serial.print("   Water pump state: ");
  Serial.println(waterpumpState);
  bluetooth.print("   Water pump state: ");
  bluetooth.println(waterpumpState);

  Serial.print("   Hot gas valve state: ");
  Serial.println(hotgasState);
  bluetooth.print("   Hot gas valve state: ");
  bluetooth.println(hotgasState);

  Serial.print("   Water valve state: ");
  Serial.println(watervalveState);
  bluetooth.print("   Water valve state: ");
  bluetooth.println(watervalveState);

  Serial.print("   Fan duty cycle: ");
  Serial.println(fanDutyCycle);
  bluetooth.print("   Fan duty cycle: ");
  bluetooth.println(fanDutyCycle);

  Serial.print("   Instant RPM: ");
  Serial.println(rpm);
  bluetooth.print("   Instant RPM: ");
  bluetooth.println(rpm);

  Serial.print("   Avg RPM over last 100 readings: ");
  Serial.println(rpmAvg);
  bluetooth.print("   Avg RPM over last 100 readings: ");
  bluetooth.println(rpmAvg);

  Serial.print("   Main state: ");
  Serial.println(mainStateString);
  bluetooth.print("   Main state: ");
  bluetooth.println(mainStateString);

  Serial.print("   Main state minuntes: ");
  Serial.println(mainStateTimerMin);
  bluetooth.print("   Main state minutes: ");
  bluetooth.println(mainStateTimerMin);

  Serial.print("   Secondary state: ");
  Serial.println(secStateString);
  bluetooth.print("   Secondary state: ");
  bluetooth.println(secStateString);

  Serial.print("   Secondary state minutes: ");
  Serial.println(secStateTimerMin);
  bluetooth.print("   Secondary state minutes: ");
  bluetooth.println(secStateTimerMin);



  Serial.println("-------------------------------------------");
  bluetooth.println("-----------------------------");
}

//Set input states based on BT character input
void inputStateChange(char inputChar) {
  switch (inputChar) {

    case COMPRESSORCHAR:
      if (digitalRead(RELAY4PIN) == HIGH) {
        digitalWrite(RELAY4PIN, LOW);
        compressorState = 0;
        Serial.println("Compressor OFF");
        bluetooth.println("Compressor OFF");
      }
      else if (digitalRead(RELAY4PIN) == LOW) {
        digitalWrite(RELAY4PIN, HIGH);
        compressorState = 1;
        Serial.println("Compressor ON");
        bluetooth.println("Compressor ON");
      }
      break;

    case WATERPUMPCHAR:
      if (digitalRead(RELAY3PIN) == HIGH) {
        digitalWrite(RELAY3PIN, LOW);
        waterpumpState = 0;
        Serial.println("Water pump OFF");
        bluetooth.println("Water pump OFF");
      }
      else if (digitalRead(RELAY3PIN) == LOW) {
        digitalWrite(RELAY3PIN, HIGH);
        waterpumpState = 1;
        Serial.println("Water pump ON");
        bluetooth.println("Water pump ON");
      }
      break;

    case HOTGASCHAR:
      if (digitalRead(RELAY2PIN) == HIGH) {
        digitalWrite(RELAY2PIN, LOW);
        hotgasState = 0;
        Serial.println("Hot gas valve CLOSED");
        bluetooth.println("Hot gas valve CLOSED");
      }
      else if (digitalRead(RELAY2PIN) == LOW) {
        digitalWrite(RELAY2PIN, HIGH);
        hotgasState = 1;
        Serial.println("Hot gas valve OPEN");
        bluetooth.println("Hot gas valve OPEN");
      }
      break;

    case WATERVALVECHAR:
      if (digitalRead(RELAY1PIN) == HIGH) {
        digitalWrite(RELAY1PIN, LOW);
        watervalveState = 0;
        Serial.println("Water valve CLOSED");
        bluetooth.println("Water valve CLOSED");
      }
      else if (digitalRead(RELAY1PIN) == LOW) {
        digitalWrite(RELAY1PIN, HIGH);
        watervalveState = 1;
        Serial.println("Water valve OPEN");
        bluetooth.println("Water valve OPEN");
      }
      break;

    case FANCHAR:
      if (fanDutyCycle > 0 && fanDutyCycle <= 50) {
        fanDutyCycle = 100;
        setPwmDuty(fanDutyCycle);
        Serial.println("Fan duty set to 100%");
        bluetooth.println("Fan duty set to 100%");
      }
      else if (fanDutyCycle > 50 && fanDutyCycle <= 100) {
        fanDutyCycle = 0;
        setPwmDuty(fanDutyCycle);
        Serial.println("Fan duty set to 0% (OFF)");
        bluetooth.println("Fan duty set to 0% (OFF)");
      }
      else if (fanDutyCycle == 0) {
        fanDutyCycle = 50;
        setPwmDuty(fanDutyCycle);
        Serial.println("Fan duty set to 50%");
        bluetooth.println("Fan duty set to 50%");
      }
      break;

    case FREEZECHAR:
      Serial.println("Freezing...");
      bluetooth.println("Freezing...");
      secStateString = freezeString;

      //Compressor ON
      digitalWrite(RELAY4PIN, HIGH);
      compressorState = 1;
      //Water pump ON
      digitalWrite(RELAY3PIN, HIGH);
      waterpumpState = 1;
      //Hot gas CLOSED
      digitalWrite(RELAY2PIN, LOW);
      hotgasState = 0;
      //Water valve CLOSED
      digitalWrite(RELAY1PIN, LOW);
      watervalveState = 0;
      //Fan 100%
      fanDutyCycle = 100;
      if (customDutyCycle != 0) {
        fanDutyCycle = customDutyCycle;
      }
      setPwmDuty(fanDutyCycle);
      break;

    case HARVESTCHAR:
      Serial.println("Harvesting...");
      bluetooth.println("Harvesting...");
      secStateString = harvestString;

      //Compressor ON
      digitalWrite(RELAY4PIN, HIGH);
      compressorState = 1;
      //Water pump OFF
      digitalWrite(RELAY3PIN, LOW);
      waterpumpState = 0;
      //Hot gas OPEN
      digitalWrite(RELAY2PIN, HIGH);
      hotgasState = 1;
      //Water valve OPEN
      digitalWrite(RELAY1PIN, HIGH);
      watervalveState = 1;
      //Fan 0%
      fanDutyCycle = 0;
      setPwmDuty(fanDutyCycle);
      break;

    case FILLCHAR:
      //Special case flush
      Serial.println("Filling/flushing...");
      bluetooth.println("Filling/flushing...");
      secStateString = flushString;

      //Compressor ON
      digitalWrite(RELAY4PIN, HIGH);
      compressorState = 1;
      //Water pump OFF
      digitalWrite(RELAY3PIN, LOW);
      waterpumpState = 0;
      //Hot gas OPEN
      digitalWrite(RELAY2PIN, LOW);
      hotgasState = 0;
      //Water valve OPEN
      digitalWrite(RELAY1PIN, HIGH);
      watervalveState = 1;
      //Fan 50%
      fanDutyCycle = 50;
      setPwmDuty(fanDutyCycle);
      break;

    case STOPCHAR:
      Serial.println("Stop! All components OFF/CLOSED");
      bluetooth.println("Stop! All components OFF/CLOSED");
      secStateString = binfullString;

      //Compressor OFF
      digitalWrite(RELAY4PIN, LOW);
      compressorState = 0;
      //Water pump OFF
      digitalWrite(RELAY3PIN, LOW);
      waterpumpState = 0;
      //Hot gas OPEN
      digitalWrite(RELAY2PIN, LOW);
      hotgasState = 0;
      //Water valve OPEN
      digitalWrite(RELAY1PIN, LOW);
      watervalveState = 0;
      //Fan 0%
      fanDutyCycle = 0;
      setPwmDuty(fanDutyCycle);

      //Set main state to stop/off
      mainState = STOP;
      mainStateString = stopString;
      secState = BINFULL;
      secStateString = binfullString;

      mainStateTimerMin = 0;
      secStateTimerMin = 0;

      break;

    case GOCHAR:
      Serial.println("Go! Making ice...");
      bluetooth.println("Go! Making ice...");

      mainState = GO;
      secState = FILL;

      inputStateChange(FILLCHAR);

      mainStateTimerMin = 0;
      secStateTimerMin = 0;
      break;

    case SKIPCHAR:
      Serial.println("Skipping state!");
      bluetooth.println("Skipping state!");

      if (mainState == GO) {
        //Set timer to high value to end current state
        secStateTimerMin = 100;
      }
      break;
  }
}
