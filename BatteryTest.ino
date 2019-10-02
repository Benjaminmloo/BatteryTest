#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_TFTLCD.h>

#define VER "0.0"

#define TFT_PIN_CS A3
#define TFT_PIN_CD A2 
#define TFT_PIN_WR A1
#define TFT_PIN_RD A0
#define TFT_PIN_RT -1

#define ENC_PIN_A 0
#define ENC_PIN_B 12
#define ENC_PIN_C 1

#define SEN_PIN_V A0
#define SEN_PIN_C A1
#define SEN_PIN_REF A5

#define VREF 3.3 //actual reference voltage
#define VDIV 6 //voltage divider compensation factor

#define VCUR 10

#define CUR_OFFSET -1 * (curRef / 2.042) * VCUR

#define ADC_DIV 1024

#define SEN_GAIN_V ((VREF * VDIV) / (ADC_DIV))//range of supply / (range of analog read * max int value)
#define SEN_GAIN_C ((VREF * VCUR) / ADC_DIV)
#define SEN_GAIN_REF (VREF * VDIV / ADC_DIV)

#define SCREEN_WIDTH 320 // tft display width, in pixels
#define SCREEN_HEIGHT 240 // tft display height, in pixels

#define MENU_Y 170

#define CC 0
#define CP 1

#define ST_SETUP 0
#define ST_SET 1
#define ST_VERIFY 2
#define ST_RUN 3
#define ST_QUIT 4

#define SU_CR_MODE 0
#define SU_CR_SET_RATE 1
#define SU_CR_SET_COV 2
#define SU_CR_START 3

#define SU_CR_POSITIONS 4

#define TICK_LENGTH 100
#define DEBOUNCE_DELAY 700

#define MAX_CURRENT 19.0
#define SET_STEP 0.1
#define FLOAT_PREC 2
#define FLOAT_MINL 5

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

Adafruit_TFTLCD tft(TFT_PIN_CS, TFT_PIN_CD, TFT_PIN_WR, TFT_PIN_RD, TFT_PIN_RT);


volatile int encDir;

volatile bool newEnc;
volatile bool newBtn;

volatile bool enEnc;
volatile bool enBtn;

byte state;
byte cursorPos;

bool mode;
bool viewMode;
bool setSel;
bool quit;
bool negCurrent;
bool redraw;

byte bufferPos;
bool curCharBuffer;
char charBuffer [2][96];
char tempBuffer [10];
float ox,oy;

float setCurrent;
float setPower;

float curCurrent;
float curVoltage;
float curPower;
float curRef;

float curCharge;
float curEnergy;

float cutOffV;


float checkVal;

unsigned long lastTick;
unsigned long lastBtn;
unsigned long lastEnc;
bool showCursor;


void setup() {
  Serial.begin(9600);
  
  tft.reset();
  tft.begin(0x9325);

  tft.setRotation(3);
  
  tft.setTextColor(WHITE);
  tft.setTextSize(0x02);
  tft.fillScreen(BLACK);
  
  dispFlash();
  
//initiallize timers
  lastTick = 0;
  lastBtn = 0;
  lastEnc = 0;
  
  pinMode(ENC_PIN_A, INPUT_PULLUP);
  pinMode(ENC_PIN_B, INPUT_PULLUP);
  pinMode(ENC_PIN_C, INPUT_PULLUP);

//initialize menu state
  state = ST_SETUP;
  mode = CC;
  cursorPos = SU_CR_MODE;

//Initialize interupts
  attachInterrupt(2, doEncoder, RISING);
  attachInterrupt(3, doButton, FALLING);
  delay(1000);

//init interupt flags
  newEnc = 0;
  newBtn = 0;
  
  enEnc = 0;
  enBtn = 0;
  
  float x, y;
  redraw = true;
  for (x = 0; x <= 6.3; x += .1) {

    y = sin(x);
    Graph(tft, x, y, 40, 140, 270, 120, 0, 6.5, 1, -1, 1, 0.2, "", "t", "V", DKBLUE, RED, YELLOW, WHITE, BLACK, redraw);

  }
  
  delay(1000);
  
  dispSetup();
}

void loop() {
  switch(state){
    case ST_SETUP: //while in setup
      if(newBtn){  //on button press
        switch(cursorPos){
          case SU_CR_SET_RATE:
          case SU_CR_SET_COV:
            state = ST_SET; // otherwise move to set state
            break;
          case SU_CR_MODE:
            mode = !mode;
            break;
          case SU_CR_START: //if curson is on start change to that state and reset cursor
            state = ST_VERIFY;
            quit = 0;
            break;
        }
        newBtn = false;
      }
      
      if(newEnc){
        if(!(cursorPos == 0 && encDir < 0 || cursorPos == SU_CR_POSITIONS - 1 && encDir > 0)){
          cursorPos = cursorPos + encDir;
        }
        newEnc = false;
      }
      
      dispSetup();
      break;
    case ST_SET:
      if(newBtn){
        state = ST_SETUP;
        newBtn = false;
      }
     
      if(newEnc){
        switch(cursorPos){
          case SU_CR_SET_RATE:
            if(mode == CC){
                checkVal = setCurrent + SET_STEP * encDir;
                
                if((encDir > 0 || checkVal > 0) && (encDir < 0 || checkVal < MAX_CURRENT))
                  setCurrent = checkVal;
            } else {
                checkVal = setPower + SET_STEP * encDir;
                
                if(checkVal > 0)
                  setPower = checkVal;
              
            }
          break;
          case SU_CR_SET_COV:
            checkVal = cutOffV + SET_STEP * encDir;
            if((encDir > 0 || checkVal > 0) && (encDir < 0 || checkVal < curVoltage))
              cutOffV = checkVal;
          break;
        }
        newEnc = false;
      }
      dispSetup();
    break;
    case ST_VERIFY:
      if(newBtn){
        if(quit){
          state = ST_RUN;
          viewMode = mode;
          
          cursorPos = 0;
          curCharge = 0;
          curEnergy = 0;
        }else {
          state = ST_SETUP;
        }
        newBtn = false;
      }

      if(newEnc){
        quit = !quit;

        newEnc = false;
      }
      dispVerify();
    break;
    case ST_RUN:
      if(newBtn){
        state = ST_QUIT;
        quit = 0;
        
        newBtn = false;
      }

      if(newEnc){
        viewMode = !viewMode;

        newEnc = false;
      }

    dispRun();
    break;
    case ST_QUIT:
      if(newBtn){
        if(quit){
          state = ST_SETUP;
        }else {
          state = ST_RUN;
        }
        newBtn = false;
      }

      if(newEnc){
        quit = !quit;

        newEnc = false;
      }

    dispQuit();
    break;
  }

  checkTime();
  checkSensors();
}

void doEncoder() {
  if(digitalRead(ENC_PIN_A) == digitalRead(ENC_PIN_B)){
    encDir = -1;
  } else {
    encDir = 1;
  }
  newEnc = true;
}

void doButton(){
  if(enBtn){
    enBtn = false;
    if(digitalRead(ENC_PIN_C == 0)){
      newBtn = true;
    }
  }
}

void checkTime(){
  unsigned long curTime = millis();
  
  if(curTime > lastTick + TICK_LENGTH){
    lastTick = curTime;
    showCursor = !showCursor;
  }
  
  if(!enBtn && curTime > lastBtn + DEBOUNCE_DELAY){
    lastBtn = curTime;
    enBtn = true;
  }
}

void checkSensors(){
  curCurrent = analogRead(SEN_PIN_C) * SEN_GAIN_C + CUR_OFFSET;
  if(curCurrent < 0){
    curCurrent = 0;
    negCurrent = true;
  }else{
    negCurrent = false;
  }
  curVoltage = analogRead(SEN_PIN_V) * SEN_GAIN_V;
  curRef = analogRead(SEN_PIN_REF) * SEN_GAIN_V;
}

void dispReset(){
  tft.setCursor(0, MENU_Y);
  tft.setTextColor(BLACK);
  tft.setTextSize(0x02);
  tft.print(charBuffer[!curCharBuffer]);

  bufferPos = 0;
}

void addToBuffer(char newStr[]){
  int i = 0;
  
  do{
    charBuffer[curCharBuffer][bufferPos++] = newStr[i++];
  }while(newStr[i] != 0);
  
  charBuffer[curCharBuffer][bufferPos] = 0;
}

void dispBuffer(){
  tft.setCursor(0, MENU_Y);
  tft.setTextColor(WHITE);
  tft.setTextSize(0x02);
  tft.print(charBuffer[curCharBuffer]);
  
  curCharBuffer = !curCharBuffer;
}

void dispFlash(){
  addToBuffer("BATT TEST\nv");
  addToBuffer(VER);
  
  dispReset();
  dispBuffer();
}

void dispSetup(){

  addToBuffer(cursorPos == SU_CR_MODE && (state != ST_SET || showCursor) ? ">" : " ");
  addToBuffer("MODE:  ");
  addToBuffer(mode == CC ? "CC\n" : "CP\n");

  if(mode == CC){
    addToBuffer(cursorPos == SU_CR_SET_RATE && (state != ST_SET || showCursor) ? ">" : " ");
    
    addToBuffer(dtostrf(setCurrent, FLOAT_MINL, FLOAT_PREC, tempBuffer));
    addToBuffer("A\n");
  }
  
  if(mode == CP){
    addToBuffer(cursorPos == SU_CR_SET_RATE && (state != ST_SET || showCursor) ? ">" : " ");
    
    addToBuffer(dtostrf(setPower, FLOAT_MINL, FLOAT_PREC, tempBuffer));
    addToBuffer("W\n");
  }

  addToBuffer(cursorPos == SU_CR_SET_COV && (state != ST_SET || showCursor) ? ">" : " ");
  
  addToBuffer(dtostrf(cutOffV, FLOAT_MINL, FLOAT_PREC, tempBuffer));
  addToBuffer("V ");

  
  addToBuffer(dtostrf(curVoltage, FLOAT_MINL, FLOAT_PREC, tempBuffer));
  addToBuffer("V\n");    

  addToBuffer(cursorPos == SU_CR_START ? ">" : " ");
  addToBuffer("START \n");
  
  dispReset();
  dispBuffer();
}

void dispVerify(){
  dispReset();

  addToBuffer(" START? ");
  
  
  if(mode == CC){
    addToBuffer("CC\n");
    
    addToBuffer(dtostrf(setCurrent, FLOAT_MINL, FLOAT_PREC, tempBuffer));
    addToBuffer("A ");
    
    addToBuffer(dtostrf(curCurrent, FLOAT_MINL, FLOAT_PREC, tempBuffer));
    addToBuffer("A\n");
  }
  
  if(mode == CP){
    addToBuffer("CP\n");
    
    addToBuffer(dtostrf(setPower, FLOAT_MINL, FLOAT_PREC, tempBuffer));
    addToBuffer("W ");
    
    addToBuffer(dtostrf(curPower, FLOAT_MINL, FLOAT_PREC, tempBuffer));
    addToBuffer("W\n");
  }


  addToBuffer(dtostrf(cutOffV, FLOAT_MINL, FLOAT_PREC, tempBuffer));
  addToBuffer("V ");

  addToBuffer(dtostrf(curVoltage, FLOAT_MINL, FLOAT_PREC, tempBuffer));
  addToBuffer("V\n");
  
  addToBuffer(quit && showCursor ? ">" : " ");
  addToBuffer("Y  ");

  addToBuffer(!quit && showCursor ? ">" : " ");
  addToBuffer("N  ");

  
  dispReset();
  dispBuffer();
}

void dispRun(){
  dispReset();

  if(viewMode == CC){
    
    addToBuffer(dtostrf(curCharge, FLOAT_MINL, FLOAT_PREC, tempBuffer));
    addToBuffer("Ah   ");
    
    addToBuffer(mode == CC ? "CC\n" : "CP\n");
    
     
    addToBuffer(dtostrf(setCurrent, FLOAT_MINL, FLOAT_PREC, tempBuffer));
    addToBuffer("A ");
    
    addToBuffer(dtostrf(curCurrent, FLOAT_MINL, FLOAT_PREC, tempBuffer));
    addToBuffer("A\n");
  }
  
  if(viewMode == CP){
    addToBuffer(dtostrf(curEnergy, FLOAT_MINL, FLOAT_PREC, tempBuffer));
    addToBuffer("Wh   ");
    
    addToBuffer(mode == CC ? "CC\n" : "CP\n");
     
    addToBuffer(dtostrf(setPower, FLOAT_MINL, FLOAT_PREC, tempBuffer));
    addToBuffer("W ");
    
    addToBuffer(dtostrf(curPower, FLOAT_MINL, FLOAT_PREC, tempBuffer));
    addToBuffer("W\n");
  }
  
  addToBuffer(dtostrf(cutOffV, FLOAT_MINL, FLOAT_PREC, tempBuffer));
  addToBuffer("V ");

  addToBuffer(dtostrf(curVoltage, FLOAT_MINL, FLOAT_PREC, tempBuffer));
  addToBuffer("V\n");
    
  addToBuffer(">STOP\n");
  dispBuffer();
}

void dispQuit(){
  dispReset();

  addToBuffer("   QUIT?\n\n  ");
  
  addToBuffer(quit && showCursor ? ">" : " ");
  addToBuffer("Y  ");

  addToBuffer(!quit && showCursor ? ">" : " ");
  addToBuffer("N  ");
  dispBuffer();
}

/*
 * Graph: uses adafruit TFT drive library to draw a cartesian coordinate system and plot whatever data you want
 * just pass x and y and the graph will be drawn
 *
 * ARGS:
 *   &d name of your display object
 *   x = x data point
 *   y = y datapont
 *   gx = x graph location (lower left)
 *   gy = y graph location (lower left)
 *   w = width of graph
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
 * Author:kris kasprzak - https://www.youtube.com/watch?v=YejRbIKe6e0
 */

void Graph( Adafruit_TFTLCD &d, double x, double y, double gx, double gy, double w, double h, double xlo, double xhi, double xinc, double ylo, double yhi, double yinc, String title, String xlabel, String ylabel, unsigned int gcolor, unsigned int acolor, unsigned int pcolor, unsigned int tcolor,unsigned int bcolor, boolean &redraw) {
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
      // precision is default Arduino--this could really use some format control
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
