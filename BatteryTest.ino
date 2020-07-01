#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include "BatteryTest.h"
#include "TimeBasedGraph.h"

//object definitions
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_PIN_CS, TFT_PIN_DC);
TimeBasedGraph graph(
  tft,
  INIT_MAX_T, NUM_INC_T,
  INIT_MIN_Y, INIT_RANGE, NUM_INC_Y, "V",
  GRAPH_X, GRAPH_Y, GRAPH_W, GRAPH_H,
  GRID_COLOUR, AXIS_COLOUR,
  POINT_COLOUR, BACK_COLOUR,
  TEXT_COLOUR);

//Interupt Flags / Values
volatile int encDir;

//flag to indicate new values
volatile bool newEnc;
volatile bool newBtn;

//enables for inputs
volatile bool enEnc;
volatile bool enBtn;

//State Variables
byte state;
byte cursorPos;

float controlValue;

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
int curWindow[NUM_READINGS];
int vltWindow[NUM_READINGS];
int refWindow[NUM_READINGS];

int curTotal;
int vltTotal;
int refTotal;

int readIndex;

//Values
float value[NUM_VALUES]; //storage for values referenced with indexes defined in header
char valuePostfix[][3] = {"A", "V", "W", "A", "V", "W", "Ah", "Wh"}; //units for respective values
byte valueField[] = {1, 2, 1, 3, 4, 3, 0, 0}; //field that each value belogns to

float currentSenRef; //Reference internal use only

//Field - positions on the screen where values are written
char fieldLastWr[NUM_FIELDS][FLD_STR_L]; //The last string printed to a given field - used to quickly clear that field for the next value
int fieldX[] = {TEXT_W, TEXT_W, TEXT_W, TEXT_W * 8, TEXT_W * 8};
int fieldY[] = { MENU_Y, MENU_Y + TEXT_H, MENU_Y + TEXT_H * 2, MENU_Y + TEXT_H, MENU_Y + TEXT_H * 2};

//non-volatile timing variables
unsigned long lastCursorFlash;
unsigned long lastSensorPrint;
unsigned long lastSensorCheck;
bool printSen;

//Graphing variables

//stores values previously printed to the screen
int graphStore[GRAPH_W];
int graphIndex = 0;

void setup() {
  File logFile;

  if (VERBOSE || SENSOR_READOUT)
    Serial.begin(9600);


  // set bus direction
  pinMode(BUS_PIN_DR, OUTPUT);

  digitalWrite(BUS_PIN_DR, BUS_WR);

  TCCR1B = TCCR1B & B11111000 | B00000001; // PWM frequency of pin 9 and 10 to 31372.55 Hz

  tft.begin();      // Init ST7735S chip, black tab

  tft.setRotation(3);
  tft.setTextWrap(false);

  tft.setTextColor(TEXT_COLOUR);
  tft.setTextSize(TEXT_SIZE);
  tft.fillScreen(BLACK);

  printFlash();

  //initiallize timers
  lastCursorFlash = 0;

  pinMode(ENC_PIN_A, INPUT);
  pinMode(ENC_PIN_B, INPUT);
  pinMode(ENC_PIN_C, INPUT);


  modeIsCP = 0;


  //Initialize interupts
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_A), doEncoder, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_B), doButton, FALLING);


  //intialize sd card
  if (VERBOSE)
    Serial.println("Initializing SD card...");

  if (USE_SD && SD.begin(SDC_PIN_CS)) {
    if (VERBOSE)
      Serial.println("SD card found");
    logFile = SD.open(LOG_PATH, FILE_WRITE);
    if (logFile) {
      SDIsValid = true;
      logFile.close();
      if (VERBOSE)
        Serial.println("log opened");
    } else {
      if (VERBOSE)
        Serial.println("log not availible");
    }
  } else {
    if (VERBOSE)
      Serial.println("SD card NOT found");
  }

  delay(1000);
  //init interupt flags
  newEnc = 0;
  newBtn = 0;

  enBtn = 0;

  graphIndex = 0;

  float t;
  float v;
  graph.drawGraphGrid();
  for(t = 0; t <= 60; t += 0.1){
    v = 5 * sin(2 * PI * t / 60) + 5;
    graph.drawGraphValue(t, v);
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
  float checkVal;
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
              checkVal = value[I_S_CUR] + SET_STEP * encDir; //Calculate new setvalue
              //if the new set value is outside of the bounds don't set it
              if ((encDir > 0 || checkVal > 0) && (encDir < 0 || checkVal < MAX_CURRENT)) {
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

  if (SENSOR_READOUT)
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
  bool pinState = digitalRead(ENC_PIN_C);

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

   if the discharge circuit is active calculate how much charge and enerGRAPH_Y was passed through
   the discharge circuit by multiplying the instantanious current / power by the time elapsed
   and a time conversion factor

   resets flags and updates time
*/
void checkSensors() {
  String dataString = "";
  unsigned long curTime = micros();
  unsigned long timeDif = curTime - lastSensorCheck;
  float readVoltage;
  float readCurrent;

  bool doPrintVoltage = printSen && state < NUM_ST && state != ST_QUIT;
  bool doPrintCurrent = printSen && state == ST_RUN && viewModeIsCP == 0;
  bool doPrintPower = printSen && state == ST_RUN && viewModeIsCP == 1;

  File logFile;

  curTotal -= curWindow[readIndex];
  vltTotal -= vltWindow[readIndex];
  refTotal -= refWindow[readIndex];

  delay(5);
  curWindow[readIndex] = analogRead(SEN_PIN_CUR);
  delay(50);
  vltWindow[readIndex] = analogRead(SEN_PIN_VLT);
  delay(50);
  refWindow[readIndex] = analogRead(SEN_PIN_REF);

  curTotal += curWindow[readIndex];
  vltTotal += vltWindow[readIndex];
  refTotal += refWindow[readIndex];

  readIndex = (readIndex + 1) % NUM_READINGS;

  currentSenRef = (refTotal / NUM_READINGS) * SEN_GAIN_REF + REF_OFFSET;
  readCurrent = (curTotal / NUM_READINGS) * SEN_GAIN_CUR + CUR_OFFSET;
  readVoltage = (vltTotal / NUM_READINGS) * SEN_GAIN_VLT + VLT_OFFSET;

  updateValue(I_C_CUR, readCurrent, doPrintCurrent);
  updateValue(I_C_VLT, readVoltage, doPrintVoltage);
  updateValue(I_C_PWR, readVoltage * readCurrent, doPrintPower);

  dataString += Serial.print(readVoltage, 4);
  if (SENSOR_READOUT) { //compiles to 252B
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

    if (USE_SD && SDIsValid) {
      logFile = SD.open(LOG_PATH, FILE_WRITE);
      if(logFile){
        if(VERBOSE)
          Serial.println("WRITING TO SD");
        logFile.print(readVoltage, 4);
        logFile.print(", ");
        logFile.print(currentSenRef, 4);
        logFile.print(", ");
        logFile.print(value[I_C_CHG], 4);
        logFile.print(", ");
        logFile.println(value[I_C_NRG], 4);

        logFile.close();
      }else{
        if(VERBOSE)
          Serial.println("FAILED TO REOPEN SD");
      }
    }
  }

  if (!senReadIsValid && (readIndex + 1) == NUM_READINGS)
    senReadIsValid = true;

  printSen = false;
  lastSensorCheck = curTime;
}

/*
   If the load should be active based on the state, output control signal to the load to draw set amount of current
*/
void updateControl() {
  float temp = controlValue;
  if (state != ST_RUN && state != ST_QUIT) {
    controlValue = 0;
  } else {
    if (modeIsCP == 0 && value[I_C_CUR] < value[I_S_CUR] ||
        modeIsCP == 1 && value[I_C_PWR] < value[I_S_PWR] ) {
      temp += 0.25;
    } else {
      temp -= 0.25;
    }

    //don't set new value if the new value is outside of bounds
    if (temp < MAX_CONTROL && temp > MIN_CONTROL) {
      controlValue = temp;
    }
  }



  analogWrite(CTL_PIN, (byte) controlValue);

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
void updateValue(byte v, float newValue, bool printUpdate) {
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
    printString(">", cursorX[c], cursorY[c], TEXT_COLOUR);
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

  printString(fieldLastWr[field], fieldX[field], fieldY[field], TEXT_COLOUR);
}

/*
   Refreshes text area and writes flash screen information
*/
void printFlash() {
  printString("BATT TEST\n v", TEXT_W, MENU_Y, TEXT_COLOUR);
  tft.print(VER);
}

/*
   Prints the current mode to the screen

   over write the last written mode with the background colour
*/
void printMode() {
  printString(modeStr[displayedModeIsCP], MODE_X, MODE_Y, BLACK);

  printString(modeStr[modeIsCP], MODE_X, MODE_Y, TEXT_COLOUR);
  displayedModeIsCP = modeIsCP;
}

/*
   Initialises the setup screen

   Sets some state variables as well as prints the menu's fields
*/
void initSetupDisplay() {
  CLEAR_TEXT_AREA;

  updateCursor(CR_MODE);

  printString("MODE:  ", TEXT_W * 2, MENU_Y, TEXT_COLOUR);

  printString("START \n", TEXT_W * 2, MENU_Y + TEXT_H * 3, TEXT_COLOUR);

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

  printString("START? ", TEXT_W * 2, MENU_Y , TEXT_COLOUR);

  printMode();

  if (modeIsCP == 0) {
    printValue(I_S_CUR);
  } else if (modeIsCP == 1) {
    printValue(I_S_PWR);
  }

  printValue(I_S_COV);
  printValue(I_C_VLT);

  printString("Y    N  ", TEXT_W * 2, MENU_Y + TEXT_H * 3, TEXT_COLOUR);
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

  printString("STOP", TEXT_W * 2, MENU_Y + TEXT_H * 3, TEXT_COLOUR);
}

/*
   Initialises the quit verification screen

   Sets some state variables as well as prints the menu's fields
*/
void initQuitDisplay() {
  CLEAR_TEXT_AREA;
  updateCursor(CR_NO);

  printString("STOP?", TEXT_W, MENU_Y, TEXT_COLOUR);

  printString("Y    N  ", TEXT_W * 2, MENU_Y + TEXT_H * 3, TEXT_COLOUR);
}
