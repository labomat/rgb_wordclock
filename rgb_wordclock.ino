/* 

(c) 2014 - Markus Backes - https://backes-markus.de/blog/

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Dieses Programm ist Freie Software: Sie k�nnen es unter den Bedingungen
der GNU General Public License, wie von der Free Software Foundation,
Version 3 der Lizenz oder (nach Ihrer Wahl) jeder neueren
ver�ffentlichten Version, weiterverbreiten und/oder modifizieren.

Dieses Programm wird in der Hoffnung, dass es n�tzlich sein wird, aber
OHNE JEDE GEW�HRLEISTUNG, bereitgestellt; sogar ohne die implizite
Gew�hrleistung der MARKTF�HIGKEIT oder EIGNUNG F�R EINEN BESTIMMTEN ZWECK.
Siehe die GNU General Public License f�r weitere Details.

Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>. 
*/

//Library includes
#include <FastLED.h>            // library for ws2812 led arry
#include <Wire.h>               // library for serial communication
#include <TimeLib.h>            // library for time handling / needed for RTC and DCF77
#include <Timezone.h>
#include <DS3232RTC.h>          // library for DS3231/2 RTC
#include <DCF77.h>              // library for dcf77 radio time signal receiver
#include <IRremote.h>           // library for infrared remote


//#include "default_layout.h"
//#include "alt_layout1.h"
#include "alt_layout2.h"       // alt. layout horizontal alternating row

// IR defines
#define ONOFF 0xFF02FD
#define AUTO 0xFFF00F
#define BLUE_DOWN 0xFF48B7
#define BLUE_UP 0xFF6897
#define BRIGHTER 0xFF3AC5
#define DIM 0xFFBA45
#define DIY1 0xFF30CF
#define DIY2 0xFFB04F
#define DIY3 0xFF708F
#define DIY4 0xFF10EF
#define DIY5 0xFF906F
#define DIY6 0xFF50AF
#define FLASH 0xFFD02F
#define QUICK 0xFFE817
#define SLOW 0xFFC837

// Touch threshold
#define TTS 50

// Sync timeout

#define SYNCTIMEOUT 1*1000 // 1min

// LED defines
#define NUM_LEDS 114

// PIN defines
#define STRIP_DATA_PIN 17 // LED strip
#define IR_RECV_PIN 11   // IR Receiver
#define ARDUINO_LED 13   //Default Arduino LED
#define DCF_PIN 2           // Connection pin to DCF 77 device
#define DCF_INTERRUPT 0    // Interrupt number associated with pin
#define LDR_PIN 15        // LDR for light level sensing

#define TOUCH_PIN 22  // touch sensor input

// RTC PINs (müssen nicht definiert werden, nur zur Info)
// SCL 19
// SDA 18

// dcf variables
//time_t time;
//DCF77 DCF = DCF77(DCF_PIN,DCF_INTERRUPT);
bool timeInSync = false;

// timeout for dcf77 sync
int timestart = millis();
int timenow = 0;
boolean timeout = 0;
boolean forcedcf = 0; // 1 = kein Timeout


// led array
uint8_t strip[NUM_LEDS];
uint8_t stackptr = 0;

CRGB leds[NUM_LEDS];

// ir receiver
IRrecv irrecv = IRrecv(IR_RECV_PIN);
decode_results irDecodeResults;

// touch control
int touchNull_R = 0;
int touchNull = 0;

uint8_t selectedLanguageMode = 0;
const uint8_t RHEIN_RUHR_MODE = 0; //Define?
const uint8_t WESSI_MODE = 1;

boolean autoBrightnessEnabled = true;

// automatic brightness control via ldr
int newBrightness = 128; 
int oldBrightness = 128;


int displayMode = DIY1;

CRGB defaultColor = CRGB::White;
uint8_t colorIndex = 0;

int testHours = 0;
int testMinutes = 0;

const unsigned long DEFAULT_TIME = 1506816000; // Oct 1 2017 
  
// multitasking helper

const long oneSecondDelay = 1000;
const long halfSecondDelay = 500;

long waitUntilRtc = 0;
long waitUntilParty = 0;
long waitUntilOff = 0;
long waitUntilFastTest = 0;
long waitUntilHeart = 0;
long waitUntilTree = 0;
long waitUntilDCF = 0;
long waitUntilLDR = 0;

// switch 
int modeSwitch = 1;
int set = 0;
int set2 = 0;
int dark = 0;

// forward declaration
void fastTest();
void doTouchLogic();
void clockLogic();
void doIRLogic();
void doLDRLogic();
void makeParty();
void off();
void showHeart();
void pushToStrip(int ledId);
void resetAndBlack();
void resetStrip();
void displayStripRandomColor();
void displayStrip();
void displayStrip(CRGB colorCode);
void timeToStrip(uint8_t hours,uint8_t minutes);
void doDCFLogic();

int baudRate = 115200;
char unixString[11];
long unixTime;
boolean dataSync = false;


#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(str)  Serial.println (str)
#else
  #define DEBUG_PRINT(str)
#endif

void setup() {
  
  #ifdef DEBUG
    Serial.begin(9600);
  #endif
  
  pinMode(ARDUINO_LED, OUTPUT);

  pinMode(LDR_PIN, INPUT);
  
  //setup leds incl. fastled
  for(int i = 0; i<NUM_LEDS; i++) {
    strip[i] = 0;
  }
  FastLED.addLeds<WS2812B, STRIP_DATA_PIN, GRB>(leds, NUM_LEDS);
  resetAndBlack();
  displayStrip();

  touchNull = touchRead(TOUCH_PIN);

  Serial1.begin(baudRate);

  // noch kein DCF modul verfügbar :-(

  //setup dcf

//  DCF.Start();
  DEBUG_PRINT("Waiting for NNTP time ... ");
  DEBUG_PRINT("It will take at least 2 minutes until a first update can be processed.");
  
   setSyncInterval(3600); //every hour
  // setSyncProvider(getDCFTime);
  // get time from RTC
  setSyncProvider(RTC.get);   // the function to get the time from the RTC
  // ende RTC
  
  //setup ir
  irrecv.enableIRIn();
}

void loop() {
  
  // nur nötig, so lange kein DCF modul vorhanden
  // read time from Serial
  if (Serial.available()) {
    time_t t = processSyncMessage();
    if (t != 0) {
      RTC.set(t);   // set the RTC and the system time to the received value
      setTime(t);          
    }
  }
  // end RTC read from Serial

   char buffer[40];
  int i = 0;

  // while the ESP output available, push it
  // into the buffer and set the flag true
  while (Serial1.available()) {
    buffer[i++] = Serial1.read();
    dataSync = true;
    //DEBUG_PRINT("NNTP message received");
  }

  // if data is available, parse it
  if (dataSync == true) {
    if ((buffer[0] == 'U') && (buffer[1] == 'N') && (buffer[2] == 'X')) {
      // if data sent is the UNX token, take it
      unixString[0] = buffer[3];
      unixString[1] = buffer[4];
      unixString[2] = buffer[5];
      unixString[3] = buffer[6];
      unixString[4] = buffer[7];
      unixString[5] = buffer[8];
      unixString[6] = buffer[9];
      unixString[7] = buffer[10];
      unixString[8] = buffer[11];
      unixString[9] = buffer[12];
      unixString[10] = '\0';

      // print the UNX time on the UNO serial
      DEBUG_PRINT();
      DEBUG_PRINT("TIME FROM ESP: ");
      DEBUG_PRINT(unixString[0]);
      DEBUG_PRINT(unixString[1]);
      DEBUG_PRINT(unixString[2]);
      DEBUG_PRINT(unixString[3]);
      DEBUG_PRINT(unixString[4]);
      DEBUG_PRINT(unixString[5]);
      DEBUG_PRINT(unixString[6]);
      DEBUG_PRINT(unixString[7]);
      DEBUG_PRINT(unixString[8]);
      DEBUG_PRINT(unixString[9]);
      DEBUG_PRINT();
     
      unixTime = atol(unixString);
      
      // Synchronize the time with the internal clock and RTC external clock module
      if (unixTime > DEFAULT_TIME) { // simple check if received time is valid (bigger than default time)

        time_t loctime, utc;
        TimeChangeRule mesz = {"MESZ", Last, Mon, Mar, 2, +120};
        TimeChangeRule mez = {"MEZ", Last, Mon, Oct, 3, +60};
        Timezone ger(mesz, mez);

        loctime = ger.toLocal(unixTime);

        DEBUG_PRINT(unixTime);
        DEBUG_PRINT(loctime);
        
        setTime(loctime);
        RTC.set(loctime);
        DEBUG_PRINT("NNTP time set");
        dataSync = false;
      }
    }
  } 

  doLDRLogic();
  doTouchLogic();           
  doIRLogic();
              
  switch(displayMode) {
    case ONOFF:
      off();
      break;
    case DIY1:
      clockLogic();
      break;
    case DIY2:
      makeParty();
      break;
    case DIY3:
      showHeart();
      break;
    case DIY4:
      showTree();
      break;
    case DIY5:
      fastTest();
      break;
    default:
      clockLogic();
      break;
  }
}


void digitalClockDisplay(){
  // digital clock display of the time on serial port
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year()); 
  Serial.println(); 
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
    Serial.print(digits);
}

/*  code to process time sync messages from the serial port   */
#define TIME_HEADER  "T"   // Header tag for serial time sync message

unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  unsigned long loctime;
 
  TimeChangeRule mesz = {"MESZ", Last, Mon, Mar, 2, +2};
  TimeChangeRule mez = {"MEZ", Last, Mon, Oct, 3, +1};
  Timezone ger(mesz, mez);
 
  if(Serial.find(TIME_HEADER)) {
     pctime = Serial.parseInt();
        RTC.set(pctime);
        Serial.print("Set time to ");
        Serial.println(pctime);
        loctime = ger.toLocal(pctime);
        Serial.print("Set loctime to ");
        Serial.println(loctime);
        return loctime;
         if( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
           pctime = 0L; // return 0 to indicate that the time is not valid
         }
  }
  return pctime;
}
// end serial time setting


// Abfrage der kapazitiven Schalter
// Jeder Druck schaltet durch die vier Displaymodi

void doTouchLogic() {

    //DEBUG_PRINT("doing Touch logic");

   if (touchRead(TOUCH_PIN) > (touchNull + TTS) && set2 == 0) {  

        if ( (modeSwitch == 1) && (dark == 0) && set2 == 0 ) {
          dark = 1;
          DEBUG_PRINT("Dark"); 
          displayMode = ONOFF;
          off();
          set2 = 1;
        }
        if ( (modeSwitch == 1) && (dark == 1) && set2 == 0 ) {
          dark = 0;
          DEBUG_PRINT("Light"); 
          displayMode = DIY1;
          autoBrightnessEnabled = true;
        //to force display update
        testMinutes = -1;
        testHours = -1;
          set2 = 1;
        }

        if (dark == 0) {
            modeSwitch --;
            set2 = 1;
            DEBUG_PRINT("-"); 
            DEBUG_PRINT(modeSwitch);
        }
   }
    if (touchRead(TOUCH_PIN) < (touchNull + TTS) && set2 == 1) {
      delay(500);
      set2 = 0;
    }
    
    if (modeSwitch > 6) {
      modeSwitch = 1; 
    }
    if (modeSwitch < 1) {
      modeSwitch = 6; 
    }
    
    switch(modeSwitch) {
        case 1:
      // Clock  
      displayMode = DIY1;
      break;
        case 2:
      // Party
      displayMode = DIY2;
      break;
        case 3:
      // Heart
      displayMode = DIY3;
      break;
        case 4:
      // Tree
      displayMode = DIY4;
      break;
        case 5:
      // Test
      displayMode = DIY5;
      break;

        default:
      displayMode = DIY1;
      break;
      }
}

void doLDRLogic() {
  if(millis() >= waitUntilLDR && autoBrightnessEnabled) {
    DEBUG_PRINT("doing LDR logic");
    waitUntilLDR = millis();
    
    
    int ldrVal = map(analogRead(LDR_PIN), 0, 1000, 0, 100);
    //int ldrVal = analogRead(LDR_PIN);
    const int threshold = 20;

    DEBUG_PRINT("Read:");
    DEBUG_PRINT(analogRead(LDR_PIN));
    
    int newBrightness = (255-(ldrVal));
    
    if ((newBrightness > (oldBrightness+threshold)) xor (newBrightness < (oldBrightness-threshold))) {
      FastLED.setBrightness(newBrightness);
      FastLED.show();
      oldBrightness = newBrightness;
    }
    DEBUG_PRINT("Value:");
    DEBUG_PRINT(ldrVal);
    DEBUG_PRINT("Brightness:");
    DEBUG_PRINT(newBrightness);
    waitUntilLDR += oneSecondDelay;
  }
}

void doIRLogic() {
  uint8_t brightness = 0;
  if (irrecv.decode(&irDecodeResults)) {
    DEBUG_PRINT("Received IR code");
    delay(50);
    switch(irDecodeResults.value) {
      case ONOFF:
        displayMode = ONOFF;
        break;
      case AUTO:
        autoBrightnessEnabled = !autoBrightnessEnabled;
        break;
      case BLUE_DOWN:
        //TODO
        break;
      case BLUE_UP:
        //TODO
        break;
      case BRIGHTER:
        autoBrightnessEnabled = false;
        brightness = FastLED.getBrightness();
        if(brightness <= 255 - 50) {
          FastLED.setBrightness(brightness + 50);
        } else {
          FastLED.setBrightness(255);
        }
        FastLED.show();
        break;
      case DIM:
        autoBrightnessEnabled = false;
        brightness = FastLED.getBrightness();
        if(brightness >= 50) {
          FastLED.setBrightness(brightness - 50);
        } else {
          FastLED.setBrightness(0);
        }
        FastLED.show();
        break;
      case DIY1:
        displayMode = DIY1;
        autoBrightnessEnabled = true;
        //to force display update
        testMinutes = -1;
        testHours = -1;
        break;
      case DIY2:
        displayMode = DIY2;
        break;
      case DIY3:
        displayMode = DIY3;
        break;
      case DIY4:
        displayMode = DIY4;
        break;
      case DIY5:
        displayMode = DIY5;
        break;
      case DIY6:
        displayMode = DIY6;
        break;
      case FLASH:
        displayMode = FLASH;
        break;
      case QUICK:
        defaultColor = nextColor();
        displayStrip();
        break;
      case SLOW:
        defaultColor = prevColor();
        displayStrip();
        break;
      default:
        DEBUG_PRINT("IR DEFAULT");
        break;
    }
    irrecv.resume();
  }
}

///////////////////////
//DISPLAY MODES
///////////////////////
void clockLogic() {
  if(millis() >= waitUntilRtc) {
    DEBUG_PRINT("doing clock logic");
                digitalClockDisplay();  
    waitUntilRtc = millis();
    if(testMinutes != minute() || testHours != hour()) {
      testMinutes = minute();
      testHours = hour();
      resetAndBlack();
      timeToStrip(testHours, testMinutes);
      displayStrip(defaultColor);
    }
    waitUntilRtc += oneSecondDelay;
  }
}

void off() {
  if(millis() >= waitUntilOff) {
    DEBUG_PRINT("switching off");
    waitUntilOff = millis();
    resetAndBlack();
    displayStrip(CRGB::Black);
    waitUntilOff += halfSecondDelay;
  }
}

void makeParty() {
  if(millis() >= waitUntilParty) {
    autoBrightnessEnabled = false;
    DEBUG_PRINT("YEAH party party");
    waitUntilParty = millis();
    resetAndBlack();
    for(int i = 0; i<NUM_LEDS;i++) {
      leds[i] = CHSV(random(0, 255), 255, 255);
    }
    FastLED.show();
    waitUntilParty += halfSecondDelay;
  }
}

void showHeart() {
  if(millis() >= waitUntilHeart) {
    autoBrightnessEnabled = false;
    DEBUG_PRINT("showing heart");
    waitUntilHeart = millis();
    resetAndBlack();
    pushToStrip(L29); pushToStrip(L30); pushToStrip(L70); pushToStrip(L89);
    pushToStrip(L11); pushToStrip(L48); pushToStrip(L68); pushToStrip(L91);
    pushToStrip(L7); pushToStrip(L52); pushToStrip(L107);
    pushToStrip(L6); pushToStrip(L106);
    pushToStrip(L5); pushToStrip(L105);
    pushToStrip(L15); pushToStrip(L95);
    pushToStrip(L23); pushToStrip(L83);
    pushToStrip(L37); pushToStrip(L77);
    pushToStrip(L41); pushToStrip(L61);
    pushToStrip(L59);
    displayStrip(CRGB::Red);
    waitUntilHeart += oneSecondDelay;
  }
}


void showTree() {
  if(millis() >= waitUntilTree) {
    autoBrightnessEnabled = false;
    DEBUG_PRINT("showing x-mas tree");
    waitUntilTree = millis();
    resetAndBlack();
    pushToStrip(L50); pushToStrip(L48); pushToStrip(L68);
    pushToStrip(L32); pushToStrip(L72); pushToStrip(L33); pushToStrip(L73);
    pushToStrip(L25); pushToStrip(L85);
    pushToStrip(L24); pushToStrip(L84);
    pushToStrip(L16); pushToStrip(L96);
    pushToStrip(L17); pushToStrip(L97);
    pushToStrip(L1); pushToStrip(L18); pushToStrip(L98); pushToStrip(L101);
    pushToStrip(L21); pushToStrip(L38); pushToStrip(L41); pushToStrip(L58); pushToStrip(L61); pushToStrip(L78); pushToStrip(L81);
    pushToStrip(L59);
    displayStrip(CRGB::Green);
    waitUntilTree += oneSecondDelay;
  }
}


void fastTest() {
  if(millis() >= waitUntilFastTest) {
    autoBrightnessEnabled = false;
    DEBUG_PRINT("test test test");
    waitUntilFastTest = millis();
    if(testMinutes >= 60) {
      testMinutes = 0;
      testHours++;
    }
    if(testHours >= 24) {
      testHours = 0;
    }
    
    //Array leeren
    resetAndBlack();
    timeToStrip(testHours, testMinutes);
    displayStripRandomColor();
    testMinutes++;
    waitUntilFastTest += oneSecondDelay;
  }
}
///////////////////////

CRGB prevColor() {
  if(colorIndex > 0) {
    colorIndex--;
  }
  return getColorForIndex();
}
CRGB nextColor() {
  if(colorIndex < 9) {
    colorIndex++;
  }
  return getColorForIndex();
}

CRGB getColorForIndex() {
  switch(colorIndex) {
    case 0:
      return CRGB::White;
    case 1:
      return CRGB::Blue;
    case 2:
      return CRGB::Aqua;
    case 3:
      return CRGB::Green;
    case 4:
      return CRGB::Lime;
    case 5:
      return CRGB::Red;
    case 6:
      return CRGB::Magenta;
    case 7:
      return CRGB::Olive;
    case 8:
      return CRGB::Yellow;
    case 9:
      return CRGB::Silver;
    default:
      colorIndex = 0;
      return CRGB::White;
  }
}

void pushToStrip(int ledId) {
  strip[stackptr] = ledId;
  stackptr++;
}

void resetAndBlack() {
  resetStrip();
  for(int i = 0; i<NUM_LEDS; i++) {
    leds[i] = CRGB::Black;
  }
}

void resetStrip() {
  stackptr = 0;
  for(int i = 0; i<NUM_LEDS; i++) {
    strip[i] = 0;
  }
}

void displayStripRandomColor() {
  for(int i = 0; i<stackptr; i++) {
    leds[strip[i]] = CHSV(random(0, 255), 255, 255);
  }
  FastLED.show();
}

void displayStrip() {
  displayStrip(defaultColor);
}

void displayStrip(CRGB colorCode) {
  for(int i = 0; i<stackptr; i++) {
    leds[strip[i]] = colorCode;
  }
  FastLED.show();
}

void timeToStrip(uint8_t hours,uint8_t minutes)
{
  pushES_IST();

  //show minutes
  if(minutes >= 5 && minutes < 10) {
    pushFUENF1();
    pushNACH();
  } else if(minutes >= 10 && minutes < 15) {
    pushZEHN1();
    pushNACH();
  } else if(minutes >= 15 && minutes < 20) {
    pushVIERTEL();
    pushNACH();
  } else if(minutes >= 20 && minutes < 25) {
    if(selectedLanguageMode == RHEIN_RUHR_MODE) {
      pushZWANZIG();
      pushNACH();
    } else if(selectedLanguageMode == WESSI_MODE) {
      pushZEHN1();
      pushVOR();
      pushHALB();
    }
  } else if(minutes >= 25 && minutes < 30) {
    pushFUENF1();
    pushVOR();
    pushHALB();
  } else if(minutes >= 30 && minutes < 35) {
    pushHALB();
  } else if(minutes >= 35 && minutes < 40) {
    pushFUENF1();
    pushNACH();
    pushHALB();
  } else if(minutes >= 40 && minutes < 45) {
    if(selectedLanguageMode == RHEIN_RUHR_MODE) {
      pushZWANZIG();
      pushVOR();
    } else if(selectedLanguageMode == WESSI_MODE) {
      pushZEHN1();
      pushNACH();
      pushHALB();
    }
  } else if(minutes >= 45 && minutes < 50) {
    pushVIERTEL();
    pushVOR();
  } else if(minutes >= 50 && minutes < 55) {
    pushZEHN1();
    pushVOR();
  } else if(minutes >= 55 && minutes < 60) {
    pushFUENF1();
    pushVOR();
  }
  
  int singleMinutes = minutes % 5;
  switch(singleMinutes) {
    case 1:
      pushONE();
      break;
    case 2:
      pushONE();
      pushTWO();
      break;
    case 3:
      pushONE();
      pushTWO();
      pushTHREE();
      break;
    case 4:
      pushONE();
      pushTWO();
      pushTHREE();
      pushFOUR();
    break;
  }

  if(hours >= 12) {
    hours -= 12;
  }

  if(selectedLanguageMode == RHEIN_RUHR_MODE) {
    if(minutes >= 25) {
      hours++;
    }
  } else if(selectedLanguageMode == WESSI_MODE) {
    if(minutes >= 20) {
      hours++;
    }
  }

  if(hours == 12) {
    hours = 0;
  }

  //show hours
  switch(hours) {
    case 0:
      pushZWOELF();
      break;
    case 1:
      if(minutes > 4) {
        pushEINS(true);
      } else {
        pushEINS(false);
      }
      break;
    case 2:
      pushZWEI();
      break;
    case 3:
      pushDREI();
      break;
    case 4:
      pushVIER();
      break;
    case 5:
      pushFUENF2();
      break;
    case 6:
      pushSECHS();
      break;
    case 7:
      pushSIEBEN();
      break;
    case 8:
      pushACHT();
      break;
    case 9:
      pushNEUN();
      break;
    case 10:
      pushZEHN();
      break;
    case 11:
      pushELF();
      break;
  }
  
  //show uhr
  if(minutes < 5) {
    pushUHR();
  }
}

///////////////////////
//PUSH WORD HELPER
///////////////////////
void pushES_IST()  {
  pushToStrip(L9);
  pushToStrip(L10);
  pushToStrip(L30);
  pushToStrip(L49);
  pushToStrip(L50);
}

void pushFUENF1() {
  pushToStrip(L70);
  pushToStrip(L89);
  pushToStrip(L90);
  pushToStrip(L109);
}

void pushFUENF2() {
  pushToStrip(L74);
  pushToStrip(L85);
  pushToStrip(L94);
  pushToStrip(L105);
}

void pushNACH() {
  pushToStrip(L73);
  pushToStrip(L86);
  pushToStrip(L93);
  pushToStrip(L106);
}

void pushZEHN1() {
  pushToStrip(L8);
  pushToStrip(L11);
  pushToStrip(L28);
  pushToStrip(L31);
}

void pushVIERTEL() {
  pushToStrip(L47);
  pushToStrip(L52);
  pushToStrip(L67);
  pushToStrip(L72);
  pushToStrip(L87);
  pushToStrip(L92);
  pushToStrip(L107);
}

void pushVOR() {
  pushToStrip(L6);
  pushToStrip(L13);
  pushToStrip(L26);
}

void pushHALB() {
  pushToStrip(L5);
  pushToStrip(L14);
  pushToStrip(L25);
  pushToStrip(L34);
}

void pushFUNK() {
  pushToStrip(L33);
  pushToStrip(L46);
  pushToStrip(L53);
  pushToStrip(L66);
}


void pushONE() {
  pushToStrip(L113);
}

void pushTWO() {
  pushToStrip(L110);
}

void pushTHREE() {
  pushToStrip(L111);
}

void pushFOUR() {
  pushToStrip(L112);
}

void pushZWANZIG() {
  pushToStrip(L48);
  pushToStrip(L51);
  pushToStrip(L68);
  pushToStrip(L71);
  pushToStrip(L88);
  pushToStrip(L91);
  pushToStrip(L108);
}

void pushZWOELF() {
  pushToStrip(L61);
  pushToStrip(L78);
  pushToStrip(L81);
  pushToStrip(L98);
  pushToStrip(L101);
}

void pushEINS(bool s) {
  pushToStrip(L4);
  pushToStrip(L15);
  pushToStrip(L24);
  if(s) {
    pushToStrip(L35);
  }
}

void pushZWEI() {
  pushToStrip(L75);
  pushToStrip(L84);
  pushToStrip(L95);
  pushToStrip(L104);
}

void pushDREI() {
  pushToStrip(L3);
  pushToStrip(L16);
  pushToStrip(L23);
  pushToStrip(L36);
}

void pushVIER() {
  pushToStrip(L76);
  pushToStrip(L83);
  pushToStrip(L96);
  pushToStrip(L103);
}

void pushSECHS() {
  pushToStrip(L2);
  pushToStrip(L17);
  pushToStrip(L22);
  pushToStrip(L37);
  pushToStrip(L42);
}

void pushSIEBEN() {
  pushToStrip(L1);
  pushToStrip(L18);
  pushToStrip(L21);
  pushToStrip(L38);
  pushToStrip(L41);
  pushToStrip(L58);
}

void pushACHT() {
  pushToStrip(L77);
  pushToStrip(L82);
  pushToStrip(L97);
  pushToStrip(L102);
}

void pushNEUN() {
  pushToStrip(L39);
  pushToStrip(L40);
  pushToStrip(L59);
  pushToStrip(L60);
}

void pushZEHN() {
  pushToStrip(L0);
  pushToStrip(L19);
  pushToStrip(L20);
  pushToStrip(L39);
}

void pushELF() {
  pushToStrip(L54);
  pushToStrip(L65);
  pushToStrip(L74);
}

void pushUHR() {
  pushToStrip(L80);
  pushToStrip(L99);
  pushToStrip(L100);
}
///////////////////////
