#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include "BatteryTest.h"

//object definitions
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_PIN_CS, TFT_PIN_DC, TFT_PIN_RS);
File logFile;

//Interupt Flags / Values
volatile int encDir;

//flag to indicate new values
volatile bool newEnc;
volatile bool newBtn;

//enables for inputs
volatile bool enEnc;
volatile bool enBtn;

//time of last update
volatile unsigned long lastBtn;
volatile unsigned long lastEnc;

//State Variables
byte state;
byte cursorPos;
byte controlValue;

bool modeIsCP;
bool viewModeIsCP; // mode being viewed on run
bool displayedModeIsCP; //last mode displayed on screen
bool senReadIsValid;  //needed to indicate when the sensor value buffers are full 
bool showCursor; //whether or not the cursor should be shown
bool initEncState;//the first reading of the non-interupt pin after set period
bool SDIsValid = false; //Fully functioning SD card 


/*ARRAYS FOR VALUES*/
char modeStr[][3] = {"CC", "CP"};

int cursorX[] = { -20, 0, 0, 0, 0, TEXT_W * 5};
int cursorY[] = { -20, MENU_Y, MENU_Y + TEXT_H, MENU_Y + TEXT_H * 2, MENU_Y + TEXT_H * 3, MENU_Y + TEXT_H * 3};


//rolling average array for smoothing sensor values
int curReading[NUM_READINGS];
int vltReading[NUM_READINGS];
int refReading[NUM_READINGS];

int curTotal;
int vltTotal;
int refTotal;

int readIndex;

//Values
double value[NUM_VALUES]; //storage for values
char valuePostfix[][3] = {"A", "V", "W", "A", "V", "W", "Ah", "Wh"}; //units for respective values
byte valueField[] = {1, 2, 1, 3, 4, 3, 0, 0}; //field that each value belogns to

double currentSenRef; //Reference internal use only

//field - positions on the screen where values are written
char fieldLastWr[NUM_FIELDS][FLD_STR_L]; //The last string printed to a given field - used to quickly clear that field for the next value
int fieldX[] = {TEXT_W, TEXT_W, TEXT_W, TEXT_W * 8, TEXT_W * 8};
int fieldY[] = { MENU_Y, MENU_Y + TEXT_H, MENU_Y + TEXT_H * 2, MENU_Y + TEXT_H, MENU_Y + TEXT_H * 2};

//non-volatile timing variables
unsigned long lastCursorFlash;
unsigned long lastSensorPrint;
unsigned long lastSensorCheck;
bool printSen;

float ox, oy;
bool redraw;

void setup() {
  if(VERBOSE)
  Serial.begin(9600);
  
  TCCR1B = TCCR1B & B11111000 | B00000001; // PWM frequency of pin 9 and 10 to 31372.55 Hz

  tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab

  tft.setRotation(3);
  tft.setTextWrap(false);

  tft.setTextColor(WHITE);
  tft.setTextSize(TEXT_SIZE);
  tft.fillScreen(BLACK);

  printFlash();
  
  //initiallize timers
  lastCursorFlash = 0;
  lastBtn = 0;
  lastEnc = 0;
  
  pinMode(ENC_PIN_A, INPUT_PULLUP);
  pinMode(ENC_PIN_B, INPUT_PULLUP);
  pinMode(ENC_PIN_C, INPUT_PULLUP);

  modeIsCP = 0;
  
  
  //Initialize interupts
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_A), doEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_C), doButton, FALLING);
  
  if(VERBOSE)
    Serial.println("Initializing SD card...");
  
  if(SD.begin(SDC_PIN_CS)){
    if(VERBOSE)
    Serial.println("SD card found");
    logFile = SD.open(LOG_PATH, FILE_WRITE);
    if(logFile){
      SDIsValid = true;
      if(VERBOSE)
        Serial.println("log opened");
    } else {
      if(VERBOSE)
        Serial.println("log not availible");
    }
    logFile.close();
  } else {
    if(VERBOSE)
      Serial.println("SD card NOT found");
  }
  
  delay(1000);
  //init interupt flags
  newEnc = 0;
  newBtn = 0;

  enBtn = 0;

  float x, y;
  redraw = true;
  for (x = 0; x <= 6.3; x += .1) {

    y = sin(x);
    Graph(tft, x, y, GRAPH_X, GRAPH_Y, GRAPH_W, GRAPH_H, 0, 6.5, 2, -1, 1, 0.5, "", "t", "V", DKBLUE, RED, YELLOW, WHITE, BLACK, redraw);

  }

  readIndex = 0;

  delay(1000);

  initSetupDisplay();
}
/*
   Main Loop

   General programtic structure is as follows

   Accept interupts which asyncronously set flags for main loop

   Given the current state check if a flag has been set make state/value changes accordingly
   if menu is changed update initialise the new screen\
   if the cursor is moved repirint it in the new location

   when the main loop is completed check the time to
*/
void loop() {
  double checkVal;
  switch (state) {
    case ST_SETUP: //while in setup
      if (newBtn) { //on button press
        switch (cursorPos) {
          case CR_SET_RATE:
          case CR_SET_COV:
            state = ST_SET; // otherwise move to set state
            break;
          case CR_MODE:
            modeIsCP = !modeIsCP;
            if (modeIsCP == 0) {
              updateValue(I_S_CUR, value[I_S_CUR], true);
            } else {
              updateValue(I_S_PWR, value[I_S_PWR], true);
            }

            printMode();
            break;
          case CR_START: //if cursor is on start change to that state and reset cursor
            state = ST_VERIFY;
            initVerifyDisplay();
            break;
        }
        newBtn = false;
      }

      if (newEnc) {
        switch (cursorPos) {
          case CR_MODE:
            if (encDir < 0) {
              updateCursor(CR_SET_RATE);
            }
            break;
          case CR_SET_RATE:
            if (encDir > 0) {
              updateCursor(CR_MODE);
            } else if (encDir < 0) {
              updateCursor(CR_SET_COV);
            }
            break;
          case CR_SET_COV:
            if (encDir > 0) {
              updateCursor(CR_SET_RATE);
            } else if (encDir < 0) {
              updateCursor(CR_START);
            }
            break;
          case CR_START:
            if (encDir > 0) {
              updateCursor(CR_SET_COV);
            }
            break;
        }
        newEnc = false;
      }
      break;
    case ST_SET:
      if (newBtn) {
        state = ST_SETUP;
        updateCursor(cursorPos);
        newBtn = false;
      }

      if (newEnc) {
        switch (cursorPos) {
          case CR_SET_RATE:
            if (modeIsCP == 0) {
              checkVal = value[I_S_CUR] + SET_STEP * encDir; //precalculate new setvalue
              if ((encDir > 0 || checkVal > 0) && (encDir < 0 || checkVal < MAX_CURRENT)) { //if the new set value is outside of the bounds don't set it
                updateValue(I_S_CUR, checkVal, true);
              }

            } else {
              checkVal = value[I_S_PWR] + SET_STEP * encDir;

              if (checkVal > 0) {
                updateValue(I_S_PWR, checkVal, true);
              }

            }
            break;
          case CR_SET_COV:
            checkVal = value[I_S_COV] + SET_STEP * encDir;
            if ((encDir > 0 || checkVal > 0) && (encDir < 0 || checkVal < value[I_C_VLT]))
              updateValue(I_S_COV, checkVal, true);

            break;
        }
        newEnc = false;
      }
      break;
    case ST_VERIFY:
      if (newBtn) {
        if (cursorPos == CR_YES) {
          state = ST_RUN;
          initRunDisplay();
        } else if (cursorPos == CR_NO) {
          state = ST_SETUP;
          initSetupDisplay();
        }
        newBtn = false;
      }

      if (newEnc) {
        if (cursorPos == CR_NO) {
          updateCursor(CR_YES);
        } else {
          updateCursor(CR_NO);
        }
        newEnc = false;
      }
      break;
    case ST_RUN:
      if (newBtn) {
        newBtn = false;
        state = ST_QUIT;

        initQuitDisplay();
      }

      if (newEnc) {
        viewModeIsCP = !viewModeIsCP;

        if (viewModeIsCP == 0) {
          updateValue(I_C_CHG, value[I_C_CHG], true);
          updateValue(I_S_CUR, value[I_S_CUR], true);
          updateValue(I_C_CUR, value[I_C_CUR], true);
        } else if (viewModeIsCP == 1) {
          updateValue(I_C_NRG, value[I_C_NRG], true);
          updateValue(I_S_PWR, value[I_S_PWR], true);
          updateValue(I_C_PWR, value[I_C_PWR], true);
        }

        newEnc = false;
      }
      break;
    case ST_QUIT:
      if (newBtn) {
        if (cursorPos == CR_YES) {
          state = ST_SETUP;
          initSetupDisplay();
        } else if (cursorPos == CR_NO) {
          state = ST_RUN;
          initRunDisplay();
        }
        newBtn = false;
      }

      if (newEnc) {
        if (cursorPos == CR_NO) {
          updateCursor(CR_YES);
        } else {
          updateCursor(CR_NO);
        }

        newEnc = false;
      }
      break;
  }

  checkTime();
  checkSensors();
  updateControl();

  if(SENSOR_READOUT)
    Serial.print('\n');
}


/*
   encoder interupt handler

   waits for an interupt on pinA
   then looks for and edge on pinB by
   accepting all of the pinA interupts that are seen for a single encoder tick
   and looking for a change in value on pinB

   when a change of value is detected stop looking for a change
   until the next interupt that takes longer than the set DEBOUNCE_ENC_T
*/
void doEncoder() {
  cli();
  bool pinState = digitalRead(ENC_PIN_B);
  
  if(VERBOSE)
    Serial.print("ENC UPDATE");
    
  newEnc = true;
  encDir = pinState ? -1 : 1;

  sei();
}

/*
   Button interupt handeler
*/
void doButton() {
  cli();
  newBtn = true;
  sei();
}

/*
   time management function

   Updates time sensetive flags not otherwise handled in interupts
*/
void checkTime() {
  unsigned long curTime = millis();

  if (curTime > lastCursorFlash + CR_FLASH_T) {
    lastCursorFlash = curTime;
    showCursor = !showCursor;
    if (state == ST_SET) {
      if (!showCursor) {
        updateCursor(CR_NONE);
      } else {
        updateCursor(cursorPos);
      }
    }
  }

  if (curTime > lastSensorPrint + SENSOR_PRINT_T) {
    lastSensorPrint = curTime;
    printSen = true;
  }
}

/*
   updates values dependant on sensor readings

   Reads analog pins that pretain to current and voltage of the discharge circuit
   as well as the reference voltage for the current sensor

   decides whether or not to print the values to the screen dependant on time and current state

   if the discharge circuit is active calculate how much charge and energy was passed through
   the discharge circuit by multiplying the instantanious current / power by the time elapsed
   and a time conversion factor

   resets flags and updates time
*/
void checkSensors() {
  unsigned long curTime = micros();
  unsigned long timeDif = curTime - lastSensorCheck;
  double readVoltage;
  double readCurrent;

  bool doPrintVoltage = printSen && state < NUM_ST && state != ST_QUIT;
  bool doPrintCurrent = printSen && state == ST_RUN && viewModeIsCP == 0;
  bool doPrintPower = printSen && state == ST_RUN && viewModeIsCP == 1;

  curTotal -= curReading[readIndex];
  vltTotal -= vltReading[readIndex];
  refTotal -= refReading[readIndex];


  delay(5);
  curReading[readIndex] = analogRead(SEN_PIN_CUR);
  delay(20);
  vltReading[readIndex] = analogRead(SEN_PIN_VLT);
  delay(20);
  refReading[readIndex] = analogRead(SEN_PIN_REF);

  curTotal += curReading[readIndex];
  vltTotal += vltReading[readIndex];
  refTotal += refReading[readIndex];

  readIndex = (readIndex + 1) % NUM_READINGS;

  currentSenRef = (refTotal / NUM_READINGS) * SEN_GAIN_REF + REF_OFFSET;
  readCurrent = (curTotal / NUM_READINGS) * SEN_GAIN_CUR + CUR_OFFSET;
  readVoltage = (vltTotal / NUM_READINGS) * SEN_GAIN_VLT + VLT_OFFSET;

  updateValue(I_C_CUR, readCurrent, doPrintCurrent);
  updateValue(I_C_VLT, readVoltage, doPrintVoltage);
  updateValue(I_C_PWR, readVoltage * readCurrent, doPrintPower);

  if (SENSOR_READOUT) { //compiles to 252B
    Serial.print(readVoltage, 4);
    Serial.print(", ");
    Serial.print(currentSenRef, 4);
    Serial.print(", ");
    Serial.print((curTotal / NUM_READINGS) * SEN_GAIN_CUR);
    Serial.print(", ");
    Serial.print(CUR_OFFSET);
    Serial.print(", ");
    Serial.print(readCurrent);
    Serial.print(", ");
  }
  

  if (senReadIsValid && (state == ST_RUN || state == ST_QUIT)) {
    value[I_C_CHG] += value[I_C_CUR] * timeDif * H_PER_uS;
    value[I_C_NRG] += value[I_C_PWR] * timeDif * H_PER_uS;
    
    if(SDIsValid){
      logFile = SD.open(LOG_PATH, FILE_WRITE);
  
      logFile.print(readVoltage, 4);
      logFile.print(", ");
      logFile.print(currentSenRef, 4);
      logFile.print(", ");
      logFile.print(value[I_C_CHG], 4);
      logFile.print(", ");
      logFile.println(value[I_C_NRG], 4);
      
      logFile.close();
    }
  }

  if(!senReadIsValid && (readIndex + 1) == NUM_READINGS)
    senReadIsValid = true;
    
  printSen = false;
  lastSensorCheck = curTime;
}

/*
   If the load should be active based on the state, output control signal to the load to draw set amount of current
 */
void updateControl() {
  int temp = controlValue;
  if (state != ST_RUN && state != ST_QUIT) {
    controlValue = 0;
  } else {
    if (modeIsCP == 0 && value[I_C_CUR] < value[I_S_CUR] ||
        modeIsCP == 1 && value[I_C_PWR] < value[I_S_PWR] ) {
      temp += 1;
    } else {
      temp -= 1;
    }

    //don't set new value if the new value is outside of bounds
    if (temp < MAX_CONTROL && temp > MIN_CONTROL) {
      controlValue = temp;
    }
  }



  analogWrite(CTL_PIN, controlValue);

  if (SENSOR_READOUT) {
    Serial.print(controlValue);
    Serial.print(", ");
  }
}

/*
   Prints some string to a given location
*/
void printString(char * str, int x, int y, int color) {
  tft.setCursor(x, y);
  tft.setTextColor(color);
  tft.setTextSize(TEXT_SIZE);

  tft.print(str);
}

/*
   Updates value in value array
   ARGS: v - index of value to update
         newValue - the value to be written
         printUpdate - whether or not to write the new value
*/
void updateValue(byte v, double newValue, bool printUpdate) {
  byte field = valueField[v];
  value[v] = newValue;

  if (printUpdate) {
    printString(fieldLastWr[field], fieldX[field], fieldY[field], BLACK);
    printValue(v);
  }
}

/*
   Updates the cursor postion on the screen

   overwrites the last position with the background colour
   writes the cursor in at the new position
*/
void updateCursor(byte c) {
  printString(">", cursorX[cursorPos], cursorY[cursorPos], BLACK);

  if (c != CR_NONE) {
    printString(">", cursorX[c], cursorY[c], WHITE);
    cursorPos = c;
  }
}

/*
   Renders field string based on current field value and prints to set location on screen
   stores last written field string for possible erasure

   ARGS: byte v - Index of value to be printed
*/
void printValue(byte v) {
  byte field = valueField[v];
  dtostrf(value[v], FLOAT_MIN_L, FLOAT_PREC, fieldLastWr[valueField[v]]);
  memcpy(fieldLastWr[field] + FLOAT_MIN_L, valuePostfix[v], POSTFIX_L);

  printString(fieldLastWr[field], fieldX[field], fieldY[field], WHITE);
}

/*
   Refreshes text area and writes flash screen information
*/
void printFlash() {
  printString("BATT TEST\n v", TEXT_W, MENU_Y, WHITE);
  tft.print(VER);
}

/*
   Prints the current mode to the screen

   over write the last written mode with the background colour
*/
void printMode() {
  printString(modeStr[displayedModeIsCP], MODE_X, MODE_Y, BLACK);

  printString(modeStr[modeIsCP], MODE_X, MODE_Y, WHITE);
  displayedModeIsCP = modeIsCP;
}

/*
   Initialises the setup screen

   Sets some state variables as well as prints the menu's fields
*/
void initSetupDisplay() {
  CLEAR_TEXT_AREA;

  updateCursor(CR_MODE);

  printString("MODE:  ", TEXT_W * 2, MENU_Y, WHITE);

  printString("START \n", TEXT_W * 2, MENU_Y + TEXT_H * 3, WHITE);

  printMode();

  if (modeIsCP == 0) {
    printValue(I_S_CUR);
  } else if (modeIsCP == 1) {
    printValue(I_S_PWR);
  }

  printValue(I_S_COV);
  printValue(I_C_VLT);
}

/*
   Initialises the verifying screen

   Sets some state variables as well as prints the menu's fields
*/
void initVerifyDisplay() {
  CLEAR_TEXT_AREA;

  updateCursor(CR_NO);

  printString("START? ", TEXT_W * 2, MENU_Y , WHITE);

  printMode();

  if (modeIsCP == 0) {
    printValue(I_S_CUR);
  } else if (modeIsCP == 1) {
    printValue(I_S_PWR);
  }

  printValue(I_S_COV);
  printValue(I_C_VLT);

  printString("Y    N  ", TEXT_W * 2, MENU_Y + TEXT_H * 3, WHITE);
}

/*
   Initialises the run screen

   Sets some state variables as well as prints the menu's fields
*/
void initRunDisplay() {
  viewModeIsCP = modeIsCP;
  value[I_C_CHG] = 0;
  value[I_C_NRG] = 0;
  senReadIsValid = true;
  CLEAR_TEXT_AREA;

  updateCursor(CR_QUIT);

  printMode();

  if (viewModeIsCP == 0) {
    printValue(I_C_CHG);
    printValue(I_S_CUR);
    printValue(I_C_CUR);
  }

  if (viewModeIsCP == 1) {
    printValue(I_C_NRG);
    printValue(I_S_PWR);
    printValue(I_C_PWR);
  }

  printValue(I_S_COV);
  printValue(I_C_VLT);

  printString("STOP", TEXT_W * 2, MENU_Y + TEXT_H * 3, WHITE);
}

/*
   Initialises the quit verification screen

   Sets some state variables as well as prints the menu's fields
*/
void initQuitDisplay() {
  CLEAR_TEXT_AREA;
  updateCursor(CR_NO);

  printString("STOP?", TEXT_W, MENU_Y, WHITE);

  printString("Y    N  ", TEXT_W * 2, MENU_Y + TEXT_H * 3, WHITE);
}

/*
   Prints graph to screen

   uses adafruit TFT drive library to draw a cartesian coordinate system and plot whatever data you want
   just pass x and y and the graph will be drawn

   ARGS:
     &d name of your display object
     x = x data point
     y = y datapont
     gx = x graph location (lower left)
     gy = y graph location (lower left)
     w = width of graph
     h = height of graph
     xlo = lower bound of x axis
     xhi = upper bound of x asis
     xinc = division of x axis (distance not count)
     ylo = lower bound of y axis
     yhi = upper bound of y asis
     yinc = division of y axis (distance not count)
     title = title of graph
     xlabel = x asis label
     ylabel = y asis label
     gcolor = graph line colors
     acolor = axi ine colors
     pcolor = color of your plotted data
     tcolor = text color
     bcolor = background color
     &redraw = flag to redraw graph on fist call only

   RETURN: void
   Author: Kris Kasprzak - https://www.youtube.com/watch?v=YejRbIKe6e0
*/

void Graph(Adafruit_ST7735 &d, double x, double y, double gx, double gy, double w, double h, double xlo, double xhi, double xinc, double ylo, double yhi, double yinc, String title, String xlabel, String ylabel, unsigned int gcolor, unsigned int acolor, unsigned int pcolor, unsigned int tcolor, unsigned int bcolor, boolean &redraw) {
  double ydiv, xdiv;
  // initialize old x and old y in order to draw the first point of the graph
  // but save the transformed value
  // note my transform funcition is the same as the map function, except the map uses long and we need doubles
  //static double ox = (x - xlo) * ( w) / (xhi - xlo) + gx;
  //static double oy = (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
  double i;
  double temp;
  int rot, newrot;

  if (redraw == true) {

    redraw = false;
    ox = (x - xlo) * ( w) / (xhi - xlo) + gx;
    oy = (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
    // draw y scale
    for ( i = ylo; i <= yhi; i += yinc) {
      // compute the transform
      temp =  (i - ylo) * (gy - h - gy) / (yhi - ylo) + gy;

      if (i == 0) {
        d.drawLine(gx, temp, gx + w, temp, acolor);
      }
      else {
        d.drawLine(gx, temp, gx + w, temp, gcolor);
      }

      d.setTextSize(1);
      d.setTextColor(tcolor, bcolor);
      d.setCursor(gx - 30, temp);
      d.println(i, 1);
    }
    // draw x scale
    for (i = xlo; i <= xhi; i += xinc) {

      // compute the transform

      temp =  (i - xlo) * ( w) / (xhi - xlo) + gx;
      if (i == 0) {
        d.drawLine(temp, gy, temp, gy - h, acolor);
      }
      else {
        d.drawLine(temp, gy, temp, gy - h, gcolor);
      }

      d.setTextSize(1);
      d.setTextColor(tcolor, bcolor);
      d.setCursor(temp, gy + 10);
      // precision is default Arduino--this could really use some format control
      d.println(i, 1);
    }

    //now draw the labels
    d.setTextSize(1);
    d.setTextColor(tcolor, bcolor);
    d.setCursor(gx , gy - h - 30);
    d.println(title);

    d.setTextSize(1);
    d.setTextColor(acolor, bcolor);
    d.setCursor(gx - 10, gy + 10);
    d.println(xlabel);

    d.setTextSize(1);
    d.setTextColor(acolor, bcolor);
    d.setCursor(gx - 30, gy - h - 10);
    d.println(ylabel);


  }

  //graph drawn now plot the data
  // the entire plotting code are these few lines...
  // recall that ox and oy are initialized as static above
  x =  (x - xlo) * ( w) / (xhi - xlo) + gx;
  y =  (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
  d.drawLine(ox, oy, x, y, pcolor);
  d.drawLine(ox, oy + 1, x, y + 1, pcolor);
  d.drawLine(ox, oy - 1, x, y - 1, pcolor);
  ox = x;
  oy = y;
}
