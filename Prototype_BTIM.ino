#include <SoftwareSerial.h>

//------------------------------------------------------------------------------------

//Tach defines
const int TACH_PIN = 5;             //Tach input pin (typically resistor pull-up to 3.3V)
float currentTachTime = 0;          //Tracking time between tach pings
float lastTachTime = 0;             //Tracking time between tach pings
int tachCount = 0;                  //Tracking tach pings per revolution (typically 2 per rev)
int tachNoCount = 0;                //Tracking when no tach pings
float notrpm = 0;                   //Time between pings
float rpm = 0;                      //Time between pings converted to rpm
static int currentTachState = LOW;  //Tracking tack input state
static int lastTachState = LOW;     //Tracking tack input state

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
int fanDutyCycle = 0;

//Freeze
//Control character
#define FREEZECHAR 'e'

//Harvest
//Control character
#define HARVESTCHAR 't'

//Stop
//Control character
#define STOPCHAR 's'

//Go
//Control character
#define GOCHAR 'g'

//Main states
#define STOP 0
#define GO 1
//Secondary states
#define BINFULL 0
#define FREEZE 1
#define HARVEST 2

//Make ice state machine
char mainState = 0;
char secState = 0;

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

  //Timer for updating info over BT/Serial
  if (timems >= (timeCheckA + UPDATETIMER)) {
    //Reset check timer
    timeCheckA = timems;

    printInfo();

  }

  if (Serial.available()) {
    String serString = Serial.readStringUntil('\n');
    char serChar = serString[0];
    int serInt = (int)serChar - 48;
    if(serString.length() == 1) {
      if (serInt > 10) {      
        inputStateChange(serChar);
      }
    }
    else if(serString.length() >= 3) {
      serString.remove(0, 2);
      serInt = serString.toInt();
      inputOther(serInt);
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

  if (bluetooth.available()) {
    String btString = bluetooth.readStringUntil('\n');
    char btChar = btString[0];
    int btInt = (int)btChar - 48;
    if(btString.length() == 1) {
      if (btInt > 10) {      
        inputStateChange(btChar);
      }
    }
    else if(btString.length() >= 3) {
      btString.remove(0, 2);
      btInt = btString.toInt();
      inputOther(btInt);
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
}

//------------------------------------------------------------------------------------
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
  if(currentTachState == LOW && lastTachState == HIGH) {
    tachCount++;
    //Two pulses per revolution
    if(tachCount >= 2) {
      notrpm = currentTachTime - lastTachTime;
      rpm = 1/((notrpm/1000)/60);
      tachCount = 0;
      lastTachTime = currentTachTime;
    }
  }
  else {
    //If no tach state change then report 0 rpm and reset things
    tachNoCount++;
    if(tachNoCount > 1000)
    {
      tachNoCount = 0;
      notrpm = 0;
      rpm = 0;
    }
  }
  //Update tach state
  lastTachState = currentTachState;
}

void inputOther(int otherValue) {
  if(otherValue <= 100 && otherValue >= 0) {
    fanDutyCycle = otherValue;
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

    Serial.print("   Approx RPM: ");
    Serial.println(rpm);
    bluetooth.print("   Approx RPM: ");
    bluetooth.println(rpm);

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
        Serial.println("hot gas valve OPEN");
        bluetooth.println("hot gas valve OPEN");
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
      else if (fanDutyCycle == 100) {
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
      //Fan 50%
      fanDutyCycle = 50;
      setPwmDuty(fanDutyCycle);
      break;

    case HARVESTCHAR:
      Serial.println("Harvesting...");
      bluetooth.println("Harvesting...");
      
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

    case STOPCHAR:
      Serial.println("All components OFF/CLOSED");
      bluetooth.println("All components OFF/CLOSED");
        
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
      break;
      
    case GOCHAR:
      mainState = 1;
      break;
  }
}
