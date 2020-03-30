
#define VER "0.1"

/*PINS USED*/
#define SDC_PIN_CS 10
#define TFT_PIN_CS 6
#define TFT_PIN_RS 7
#define TFT_PIN_DC 8

#define ENC_PIN_A 2
#define ENC_PIN_B 3 //encoder button
#define ENC_PIN_C 4

#define SEN_PIN_REF A5
#define SEN_PIN_VLT A6
#define SEN_PIN_CUR A7

#define CTL_PIN 9

/*SENSOR GAIN & OFFSETS*/
#define REF_V 5.0 //reference voltage

//gains for coltage dividers adjust as needed for discrepencies in resistor values
#define DIV_VLT 6.0 //voltage divider on pin A0 
#define DIV_REF 6.0//voltage divider on pin A1 compensation factor

#define CUR_V 10.0 //voltage conversion factor for current sensor

#define CUR_OFFSET (currentSenRef * CUR_V / 2.0) + 0.15 //The voltage that the current sensor outputs at 0A
#define VLT_OFFSET  0.0
#define REF_OFFSET  0.0

#define ADC_DIV 1023

#define SEN_GAIN_VLT ((REF_V * DIV_VLT) / ADC_DIV)//range of supply / (range of analog read * max int value)
#define SEN_GAIN_REF ((REF_V * DIV_REF) / ADC_DIV)
#define SEN_GAIN_CUR -1 * ((REF_V * CUR_V) / ADC_DIV)

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

#define GRAPH_X 40
#define GRAPH_Y MENU_Y - 20
#define GRAPH_W SCREEN_WIDTH - 50
#define GRAPH_H MENU_Y - 35

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
#define CR_NONE 0
#define CR_MODE 1
#define CR_SET_RATE 2
#define CR_SET_COV 3
#define CR_START 4
#define CR_NO 5
#define CR_YES CR_START
#define CR_QUIT CR_START

//Value array
#define I_S_CUR 0
#define I_S_COV 1
#define I_S_PWR 2

#define I_C_CUR 3
#define I_C_VLT 4
#define I_C_PWR 5

#define I_C_CHG 6
#define I_C_NRG 7 //energy

#define NUM_VALUES 9
#define NUM_READINGS 5
#define NUM_FIELDS 5


/*OPERATION VALUES*/
#define VERBOSE false
#define SENSOR_READOUT false

//Timing periods
#define CR_FLASH_T 100
#define DEBOUNCE_BTN_T 200
#define DEBOUNCE_ENC_T 45
#define SENSOR_PRINT_T 500

//Boundry values
#define MAX_CURRENT 9.0
#define MIN_CURRENT 0.0

#define MAX_CONTROL 160
#define MIN_CONTROL 0

#define SET_STEP  0.1


//Print values
#define FLOAT_PREC 2
#define FLOAT_MIN_L 5

#define FLD_STR_L 8
#define POSTFIX_L 3

#define LOG_PATH "batterylog.txt"

/*MACROS*/
#define CLEAR_TEXT_AREA tft.fillRect(0, MENU_Y, SCREEN_WIDTH, SCREEN_HEIGHT - MENU_Y, BLACK)


/*STRUCTS*/
//struct  
