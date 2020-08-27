/*PINS USED*/
#define SDC_PIN_CS 6
#define TFT_PIN_CS 10
#define TFT_PIN_RS 7
#define TFT_PIN_DC 8
#define BUS_PIN_DR A0

#define ENC_PIN_A 2
#define ENC_PIN_B 3 //encoder button
#define ENC_PIN_C 4

#define SEN_PIN_REF A5
#define SEN_PIN_VLT A6
#define SEN_PIN_CUR A7

#define CTL_PIN 9

#define ST_READ 0
#define ST_CALIBRATE 1

#define EXPERIMENTAL 0
#define EXPECTED 1

#define NUM_STP 10
#define MAX_VLT 10.0
#define MAX_CUR 1.0

#define VLT_PER_STP MAX_VLT/NUM_STP
#define AMP_PER_STP MAX_CUR/NUM_STP

/*SENSOR GAIN & OFFSETS*/
#define REF_V 5.0 //reference voltage

//gains for coltage dividers adjust as needed for discrepencies in resistor values
#define DIV_VLT 6.0 //voltage divider on pin A0
#define DIV_REF 6.0//voltage divider on pin A1 compensation factor

#define CUR_V 10.0 //voltage conversion factor for current sensor

#define CUR_OFFSET -21.6957302//-1 * (REF_V * CUR_V / 2.0) //The voltage that the current sensor outputs at 0A
#define VLT_OFFSET  0.0673882 //0.0
#define REF_OFFSET  0.0

#define ADC_DIV 1023

#define SEN_GAIN_VLT 0.0287832//((REF_V * DIV_VLT) / ADC_DIV)//range of supply / (range of analog read * max int value)
#define SEN_GAIN_REF ((REF_V * DIV_REF) / ADC_DIV)
#define SEN_GAIN_CUR 0.0424654 //((REF_V * CUR_V) / ADC_DIV)

bool newBtn;
bool calibrated;

byte state;

//the raw number from adc
int curRawValue;
int vltRawValue;
int refRawValue;

float vltGain = SEN_GAIN_VLT;
float curGain = SEN_GAIN_CUR;

float vltOffs = VLT_OFFSET;
float curOffs = CUR_OFFSET;

//adjusted to be human readable
float curReadable;
float vltReadable;
float refReadable;

//values averaged for steadier reading of noisy ADC
float curAveraged;
float vltAveraged;
float refAveraged;

float vltSamples[NUM_STP][2];
float curSamples[NUM_STP][2];

void setup() {
  Serial.begin(9600);

  pinMode(ENC_PIN_A, INPUT);
  pinMode(ENC_PIN_B, INPUT);
  pinMode(ENC_PIN_C, INPUT);

  pinMode(SEN_PIN_REF, INPUT);
  pinMode(SEN_PIN_VLT, INPUT);
  pinMode(SEN_PIN_CUR, INPUT);

}
int i;
unsigned long lastSensorCheck;

void loop() {
  int i;
  unsigned long curTime = micros();
  unsigned long timeDif = curTime - lastSensorCheck;
  String dataString = "";
  float m, b;
  while(digitalRead(ENC_PIN_B)){
    dataString= "";

    delay(5);
    curRawValue = analogRead(SEN_PIN_CUR);
    delay(50);
    vltRawValue = analogRead(SEN_PIN_VLT);
    //delay(50);
    //refRawValue = analogRead(SEN_PIN_REF);

    curReadable = curRawValue * curGain + curOffs;
    vltReadable = vltRawValue * vltGain + vltOffs;
    //refReadable = refTotal * SEN_GAIN_REF + REF_OFFSET;

    #define NEW_VALUE_WEIGHT 0.1

    curAveraged = curAveraged * (1 - NEW_VALUE_WEIGHT) + curReadable * NEW_VALUE_WEIGHT;
    vltAveraged = vltAveraged * (1 - NEW_VALUE_WEIGHT) + vltReadable * NEW_VALUE_WEIGHT;


    dataString += String(curTime);
    dataString += ",";
    dataString += String(vltAveraged, 4);
    dataString += ",";
    dataString += String(curAveraged, 4);
    dataString += ",";
    dataString += String(vltAveraged * curAveraged, 4);
    Serial.println(dataString);
  }
  delay(1000);

  for(i = 0; i < NUM_STP; i++){
    Serial.print("Set voltage to: ");
    Serial.println(i * VLT_PER_STP);
    Serial.println("press button to continue");

    while(digitalRead(ENC_PIN_B))
      delay(100);

    vltSamples[i][EXPECTED] = i * VLT_PER_STP;
    vltSamples[i][EXPERIMENTAL] = analogRead(SEN_PIN_VLT);

    Serial.print(vltSamples[i][EXPECTED]);
    Serial.print(", ");
    Serial.println(vltSamples[i][EXPERIMENTAL]);

    delay(1000);
  }

  calculateLOBF(vltSamples, &vltGain, &vltOffs);

  for(i = 0; i < NUM_STP; i++){
    Serial.print("Set current to: ");
    Serial.println(i * AMP_PER_STP);
    Serial.println("press button to continue");

    while(digitalRead(ENC_PIN_B))
      delay(100);

    curSamples[i][EXPECTED] = i * AMP_PER_STP;
    curSamples[i][EXPERIMENTAL] = analogRead(SEN_PIN_CUR);

    Serial.print(curSamples[i][EXPECTED]);
    Serial.print(", ");
    Serial.println(curSamples[i][EXPERIMENTAL]);

    delay(1000);
  }

  calculateLOBF(curSamples, &curGain, &curOffs);
  calibrated = true;

  while(digitalRead(ENC_PIN_B))
    delay(100);

  delay(1000);
}

void calculateLOBF(float data[][2], float *m, float *b){
  float sumExpected = 0;
  float sumExperimental = 0;
  float sumExperimentalSqrd = 0;
  float sumProduct = 0;
  for(i = 0; i < NUM_STP; i++){
    sumExpected += data[i][EXPECTED];
    sumExperimental += data[i][EXPERIMENTAL];
    sumExperimentalSqrd += data[i][EXPERIMENTAL] * data[i][EXPERIMENTAL];
    sumProduct += data[i][EXPECTED] * data[i][EXPERIMENTAL];
  }

    Serial.print("Ey: ");
    Serial.print(sumExpected);
    Serial.print(" Ex: ");
    Serial.print(sumExperimental);
    Serial.print(" Exy: ");
    Serial.print(sumProduct);
    Serial.print(" Ex^2: ");
    Serial.println(sumExperimentalSqrd);

  *m =
    ((NUM_STP * sumProduct) - (sumExpected * sumExperimental))
    /((NUM_STP * sumExperimentalSqrd) - (sumExperimental * sumExperimental));

  *b =
    (sumExpected - *m * sumExperimental) / NUM_STP;

  Serial.println("m: ");
  Serial.println(*m, 7);
  Serial.println(" b: ");
  Serial.println(*b, 7);
}
