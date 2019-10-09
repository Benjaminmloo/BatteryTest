#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

#define VER "0.0"

/*PINS USED*/
#define TFT_PIN_RS 11
#define TFT_PIN_CS 10
#define TFT_PIN_DC 9

#define ENC_PIN_A 0
#define ENC_PIN_B 6
#define ENC_PIN_C 1

#define SEN_PIN_V A0
#define SEN_PIN_C A1
#define SEN_PIN_REF A5

/*SENSOR GAIN & OFFSETS*/
#define REF_V 3.3 //reference voltage
#define DIV_V 6 //voltage divider compensation factor

#define CUR_V 10 //voltage conversion factor for current sensor
#define CUR_OFFSET (currentSenRef * -1 * CUR_V / 2.042) //The voltage that the current sensor outputs at 0A

#define ADC_DIV 1024

#define SEN_GAIN_V ((REF_V * DIV_V) / ADC_DIV)//range of supply / (range of analog read * max int value)
#define SEN_GAIN_C ((REF_V * CUR_V) / ADC_DIV)

#define H_PER_uS 2.777777777E-10

/*SCREEN VALUES*/
#define SCREEN_WIDTH 160 // tft display width, in pixels
#define SCREEN_HEIGHT 128 // tft display height, in pixels
#if SCREEN_WIDTH <= 160
  #define TEXT_SIZE 1
  #define TEXT_W 6
  #define TEXT_H 8 
#else
  #define TEXT_SIZE 2
  #define TEXT_W 12
  #define TEXT_H 16 
#endif


#define MENU_Y (SCREEN_HEIGHT - TEXT_H * 4)

#define MODE_X TEXT_W * 9
#define MODE_Y MENU_Y

//16bit colour values
#define LTBLUE    0xB6DF
#define LTTEAL    0xBF5F
#define LTGREEN   0xBFF7
#define LTCYAN    0xC7FF
#define LTRED     0xFD34
#define LTMAGENTA 0xFD5F
#define LTYELLOW  0xFFF8
#define LTORANGE  0xFE73
#define LTPINK    0xFDDF
#define LTPURPLE  0xCCFF
#define LTGREY    0xE71C

#define BLUE      0x001F
#define TEAL      0x0438
#define GREEN     0x07E0
#define CYAN      0x07FF
#define RED       0xF800
#define MAGENTA   0xF81F
#define YELLOW    0xFFE0
#define ORANGE    0xFC00
#define PINK      0xF81F
#define PURPLE    0x8010
#define GREY      0xC618
#define WHITE     0xFFFF
#define BLACK     0x0000

#define DKBLUE    0x000D
#define DKTEAL    0x020C
#define DKGREEN   0x03E0
#define DKCYAN    0x03EF
#define DKRED     0x6000
#define DKMAGENTA 0x8008
#define DKYELLOW  0x8400
#define DKORANGE  0x8200
#define DKPINK    0x9009
#define DKPURPLE  0x4010
#define DKGREY    0x4A49

/*STATE DEFFINITIONS*/

//Operation Mode
#define CC 0
#define CP 1

//States
#define ST_SETUP 0
#define ST_SET 1
#define ST_VERIFY 2
#define ST_RUN 3
#define ST_QUIT 4

#define NUM_ST 5

//cursor positions
#define CR_MODE 0
#define CR_SET_RATE 1
#define CR_SET_COV 2
#define CR_START 3
#define CR_NO 4
#define CR_YES CR_START
#define CR_QUIT CR_START

//Value array 
#define I_S_CUR 0
#define I_S_COV 1
#define I_S_PWR 2

#define I_C_CUR 3
#define I_C_VLT 4
#define I_C_PWR 5

#define NUM_VALUES 9
#define NUM_FIELDS 5

#define I_C_CHG 6
#define I_C_NRG 7 //energy

/*OPERATION VALUES*/

//Timing periods
#define CR_FLASH_T 100
#define DEBOUNCE_BTN_T 500
#define DEBOUNCE_ENC_T 45
#define SENSOR_PRINT_T 500

//Boundry values
#define MAX_CURRENT 19.0
#define SET_STEP 0.1

//Print values
#define FLOAT_PREC 2
#define FLOAT_MIN_L 5

#define FLD_STR_L 8
#define POSTFIX_L 3

/*MACROS*/
#define CLEAR_TEXT_AREA tft.fillRect(0, MENU_Y, SCREEN_WIDTH, SCREEN_HEIGHT - MENU_Y, BLACK)

//Screen deffinition
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_PIN_CS, TFT_PIN_DC, TFT_PIN_RS);

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
byte lastCursorPos;
byte initEncState;

bool mode;
bool viewMode; // mode being viewed on run 
bool displayedMode; //last mode displayed on screen
bool initialSenRead;


/*ARRAYS FOR VALUES*/
char modeStr[][3] = {"CC", "CP"};

int cursorX[] = {0, 0, 0, 0, TEXT_W * 5};
int cursorY[] = {MENU_Y, MENU_Y + TEXT_H, MENU_Y + TEXT_H * 2, MENU_Y + TEXT_H * 3, MENU_Y + TEXT_H * 3};

//Values
double value[NUM_VALUES]; //storage for values
char valuePostfix[][3] = {"A", "V", "W", "A", "V", "W", "Ah", "Wh"}; //units for respective values
byte valueField[] = {1, 2, 1, 3, 4, 3, 0, 0}; //field that each value belogns to

float currentSenRef; //Reference internal use only

//field - positions on the screen where values are written
char fieldLastWr[NUM_FIELDS][FLD_STR_L]; //The last string printed to a given field - used to quickly clear that field for the next value
int fieldX[] = {TEXT_W, TEXT_W, TEXT_W, TEXT_W * 8, TEXT_W * 8}; 
int fieldY[] = { MENU_Y, MENU_Y + TEXT_H, MENU_Y + TEXT_H * 2, MENU_Y + TEXT_H, MENU_Y + TEXT_H * 2};

//non-volatile timing variables
unsigned long lastCursorFlash;
unsigned long lastSensorPrint;
unsigned long lastSensorCheck;
bool printSen;
bool showCursor;


float checkVal;
float ox, oy;
bool redraw;

void setup() {
  Serial.begin(9600);
  tft.initR(INITR_BLACKTAB);

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
  lastCursorPos = 0;

  pinMode(ENC_PIN_A, INPUT_PULLUP);
  pinMode(ENC_PIN_B, INPUT_PULLUP);
  pinMode(ENC_PIN_C, INPUT_PULLUP);

  //initialize menu state
  state = ST_SETUP;
  mode = CC;
  cursorPos = CR_MODE;

  //Initialize interupts
  attachInterrupt(2, doEncoder, RISING);
  attachInterrupt(3, doButton, FALLING);
  delay(1000);

  //init interupt flags
  newEnc = 0;
  newBtn = 0;

  enBtn = 0;

  float x, y;
  redraw = true;
  for (x = 0; x <= 6.3; x += .1) {

    y = sin(x);
    //Graph(tft, x, y, 40, 140, 270, 120, 0, 6.5, 1, -1, 1, 0.2, "", "t", "V", DKBLUE, RED, YELLOW, WHITE, BLACK, redraw);

  }

  delay(1000);

  initSetupDisplay();
}
/*
 * Main Loop
 * 
 * General programtic structure is as follows
 * 
 * Accept interupts which asyncronously set flags for main loop
 * 
 * Given the current state check if a flag has been set make state/value changes accordingly 
 * if menu is changed update initialise the new screen\
 * if the cursor is moved repirint it in the new location
 * 
 * when the main loop is completed check the time to 
 */
void loop() {
  switch (state) {
    case ST_SETUP: //while in setup
      if (newBtn) { //on button press
        switch (cursorPos) {
          case CR_SET_RATE:
          case CR_SET_COV:
            state = ST_SET; // otherwise move to set state
            break;
          case CR_MODE:
            mode = !mode;
            if (mode == CC) {
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
              cursorPos = CR_SET_RATE;
            }
            break;
          case CR_SET_RATE:
            if (encDir > 0) {
              cursorPos = CR_MODE;
            } else if (encDir < 0) {
              cursorPos = CR_SET_COV;
            }
            break;
          case CR_SET_COV:
            if (encDir > 0) {
              cursorPos = CR_SET_RATE;
            } else if (encDir < 0) {
              cursorPos = CR_START;
            }
            break;
          case CR_START:
            if (encDir > 0) {
              cursorPos = CR_SET_COV;
            }
            break;
        }
        newEnc = false;
        updateCursor();
      }
      break;
    case ST_SET:
      if (newBtn) {
        state = ST_SETUP;
        newBtn = false;
      }

      if (newEnc) {
        switch (cursorPos) {
          case CR_SET_RATE:
            if (mode == CC) {
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
        if(cursorPos == CR_YES) {
          state = ST_RUN;
          initRunDisplay();
        } else if (cursorPos == CR_NO){
          state = ST_SETUP;
          initSetupDisplay();
        }
        newBtn = false;
      }

      if (newEnc) {
        if(cursorPos == CR_NO){
          cursorPos = CR_YES;
        }else{
          cursorPos = CR_NO;
        }
        
        updateCursor();
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
        viewMode = !viewMode;
        
        if (viewMode == CC) {
          updateValue(I_C_CHG, value[I_C_CHG], true);
          updateValue(I_S_CUR, value[I_S_CUR], true);
          updateValue(I_C_CUR, value[I_C_CUR], true);
        } else if (viewMode == CP) {
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
        } else if(cursorPos == CR_NO) {
          state = ST_RUN;
          initRunDisplay();
        }
        newBtn = false;
      }

      if (newEnc) {
        if(cursorPos == CR_NO){
          cursorPos = CR_YES;
        }else{
          cursorPos = CR_NO;
        }
        
        updateCursor();
        newEnc = false;
      }
      break;
  }

  checkTime();
  checkSensors();
}


/*
 * encoder interupt handler
 * 
 * waits for an interupt on pinA
 * then looks for and edge on pinB by 
 * accepting all of the pinA interupts that are seen for a single encoder tick 
 * and looking for a change in value on pinB
 * 
 * when a change of value is detected stop looking for a change 
 * until the next interupt that takes longer than the set DEBOUNCE_ENC_T
 */
void doEncoder() {
  cli();
  unsigned long curTime, timeDif;
  byte pinState = PIND & B10000000;
  curTime = millis();
  timeDif = curTime - lastEnc;
    
  if(curTime - lastEnc < DEBOUNCE_ENC_T){    
    if(enEnc){
      if(initEncState != pinState){
        newEnc = true;
        encDir = initEncState? -1 : 1;
        enEnc = false;
      }
    } 
    
  } else {
    initEncState = pinState & B10000000;
    enEnc = true;
  }
  
  
  lastEnc = curTime;
  sei();
}

/*
 * Button interupt handeler
 */
void doButton() {
  unsigned long curTime;
  cli();
  curTime = millis();
  
  if (curTime > lastBtn + DEBOUNCE_BTN_T) {
    newBtn = true;
    lastBtn = curTime;
  }
  sei();
}

/*
 * time management function
 * 
 * Updatees time sensetive flags not otherwise handled in interupts
 */
void checkTime() {
  unsigned long curTime = millis();

  if (curTime > lastCursorFlash + CR_FLASH_T) {
    lastCursorFlash = curTime;
    showCursor = !showCursor;
  }

  if (curTime > lastSensorPrint + SENSOR_PRINT_T) {
    lastSensorPrint = curTime;
    printSen = true;
  }

}

/*
 * updates values dependant on sensor readings
 * 
 * Reads analog pins that pretain to current and voltage of the discharge circuit
 * as well as the reference voltage for the current sensor
 * 
 * decides whether or not to print the values to the screen dependant on time and current state
 * 
 * if the discharge circuit is active calculate how much charge and energy was passed through
 * the discharge circuit by multiplying the instantanious current / power by the time elapsed
 * and a time conversion factor
 * 
 * resets flags and updates time
 */
void checkSensors() {
  unsigned long curTime = micros();
  unsigned long timeDif = curTime - lastSensorCheck;
  
  currentSenRef = analogRead(SEN_PIN_REF) * SEN_GAIN_V;
  float readCurrent = analogRead(SEN_PIN_C) * SEN_GAIN_C + CUR_OFFSET;
  float readVoltage = analogRead(SEN_PIN_V) * SEN_GAIN_V;
  
  bool doPrintCurrent = printSen && state == ST_RUN && viewMode == CC;
  bool doPrintPower = printSen && state == ST_RUN && viewMode == CP;
  bool doPrintVoltage = printSen && state < NUM_ST && state != ST_QUIT;

  updateValue(I_C_CUR, readCurrent, doPrintCurrent);
  updateValue(I_C_VLT, readVoltage, doPrintVoltage);

  updateValue(I_C_PWR, readVoltage * readCurrent, doPrintPower); 
  
  if(!initialSenRead && state == ST_RUN){
    value[I_C_CHG] += value[I_C_CUR] * timeDif * H_PER_uS;
    value[I_C_NRG] += value[I_C_PWR] * timeDif * H_PER_uS;
  }


  initialSenRead = false;
  printSen = false;
  lastSensorCheck = curTime;
}

/*
 * Updates the cursor postion on the screen
 * 
 * overwrites the last position with the background colour
 * writes the cursor in at the new position
 */
void updateCursor(){
  printString('>', cursorX[lastCursorPos], cursorY[lastCursorPos], BLACK);

  printString('>', cursorX[cursorPos], cursorY[cursorPos], WHITE);
  lastCursorPos = cursorPos;
}


/*
 * Prints some string to a given location
 */
void printString(char * str, int x, int y, int color) {
  tft.setCursor(x, y);
  tft.setTextColor(color);
  tft.setTextSize(TEXT_SIZE);

  tft.print(str);
}

/*
 * Updates value in value array
 * ARGS: v - index of value to update
 *       newValue - the value to be written
 *       printUpdate - whether or not to write the new value
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
 * Renders field string based on current field value and prints to set location on screen
 * stores last written field string for possible erasure
 * 
 * ARGS: byte v - Index of value to be printed
*/
void printValue(byte v) {
  byte field = valueField[v];
  dtostrf(value[v], FLOAT_MIN_L, FLOAT_PREC, fieldLastWr[valueField[v]]);
  memcpy(fieldLastWr[field] + FLOAT_MIN_L, valuePostfix[v], POSTFIX_L);

  printString(fieldLastWr[field], fieldX[field], fieldY[field], WHITE);
}

/*
 * Refreshes text area and writes flash screen information
 */
void printFlash() {
  printString("BATT TEST\n v", TEXT_W, MENU_Y, WHITE);
  tft.print(VER);
}

/*
 * Prints the current mode to the screen
 * 
 * over write the last written mode with the background colour
 */
void printMode() {
  printString(modeStr[displayedMode], MODE_X, MODE_Y, BLACK);
  
  printString(modeStr[mode], MODE_X, MODE_Y, WHITE);
  displayedMode = mode;
}

/*
 * Initialises the setup screen 
 * 
 * Sets some state variables as well as prints the menu's fields
 */
void initSetupDisplay() {
  CLEAR_TEXT_AREA;
  
  cursorPos = CR_MODE;
  updateCursor();

  printString("MODE:  ", TEXT_W * 2, MENU_Y, WHITE);
  
  printString("START \n", TEXT_W * 2, MENU_Y + TEXT_H * 3, WHITE);

  printMode();

  if (mode == CC) {
    printValue(I_S_CUR);
  } else if (mode == CP) {
    printValue(I_S_PWR);
  }

  printValue(I_S_COV);
  printValue(I_C_VLT);

  updateCursor();
}

/*
 * Initialises the verifying screen 
 * 
 * Sets some state variables as well as prints the menu's fields
 */
void initVerifyDisplay() {
  CLEAR_TEXT_AREA;
  
  cursorPos = CR_NO;
  updateCursor();
  
  printString("START? ", TEXT_W * 2, MENU_Y , WHITE);

  printMode();

  if (mode == CC) {
    printValue(I_S_CUR);
  } else if (mode == CP) {
    printValue(I_S_PWR);
  }

  printValue(I_S_COV);
  printValue(I_C_VLT);
  
  printString("Y    N  ", TEXT_W * 2, MENU_Y + TEXT_H * 3, WHITE);
}


/*
 * Initialises the run screen 
 * 
 * Sets some state variables as well as prints the menu's fields
 */
void initRunDisplay() {
  viewMode = mode;
  value[I_C_CHG] = 0;
  value[I_C_NRG] = 0;
  initialSenRead= true;
  CLEAR_TEXT_AREA;
  
  cursorPos = CR_QUIT;
  updateCursor();
  
  printMode();

  if (viewMode == CC) {
    printValue(I_C_CHG);
    printValue(I_S_CUR);
    printValue(I_C_CUR);
  }

  if (viewMode == CP) {
    printValue(I_C_NRG);
    printValue(I_S_PWR);
    printValue(I_C_PWR);
  }

  printValue(I_S_COV);
  printValue(I_C_VLT);

  printString("STOP", TEXT_W * 2, MENU_Y + TEXT_H * 3, WHITE);
}


/*
 * Initialises the quit verification screen 
 * 
 * Sets some state variables as well as prints the menu's fields
 */
void initQuitDisplay() {
  CLEAR_TEXT_AREA;
  cursorPos = CR_NO;
  updateCursor();
 
  printString("STOP?", TEXT_W, MENU_Y, WHITE);

  printString("Y    N  ", TEXT_W * 2, MENU_Y + TEXT_H * 3, WHITE);
}

/*
 * Prints graph to screen
 * 
 * uses adafruit TFT drive library to draw a cartesian coordinate system and plot whatever data you want
 * just pass x and y and the graph will be drawn
 *
 * ARGS:
 *   &d name of your display object
 *   x = x data point
 *   y = y datapont
 *   gx = x graph location (lower left)
 *   gy = y graph location (lower left)
 8   w = width of graph
 *   h = height of graph
 *   xlo = lower bound of x axis
 *   xhi = upper bound of x asis
 *   xinc = division of x axis (distance not count)
 *   ylo = lower bound of y axis
 *   yhi = upper bound of y asis
 *   yinc = division of y axis (distance not count)
 *   title = title of graph
 *   xlabel = x asis label
 *   ylabel = y asis label
 *   gcolor = graph line colors
 *   acolor = axi ine colors
 *   pcolor = color of your plotted data
 *   tcolor = text color
 *   bcolor = background color
 *   &redraw = flag to redraw graph on fist call only
 *
 * RETURN: void
 * Author: Kris Kasprzak - https://www.youtube.com/watch?v=YejRbIKe6e0
 */

void Graph( Adafruit_ST7735 &d, double x, double y, double gx, double gy, double w, double h, double xlo, double xhi, double xinc, double ylo, double yhi, double yinc, String title, String xlabel, String ylabel, unsigned int gcolor, unsigned int acolor, unsigned int pcolor, unsigned int tcolor, unsigned int bcolor, boolean &redraw) {
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
    d.setCursor(gx , gy + 20);
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
