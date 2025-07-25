/* ************************************************************************
 *
 *   common header file
 *
 *   (c) 2012-2025 by Markus Reschke
 *   based on code from Markus Frejek and Karl-Heinz K�bbeler
 *
 * ************************************************************************ */


/*
 *  include header files
 */

/* basic includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

/* AVR */
#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>


/* source management */
#define COMMON_H



/* ************************************************************************
 *   constants for operation and UI
 * ************************************************************************ */


/* UI feedback mode for TestKey() (bitfield) */
#define CURSOR_NONE           0b00000000     /* no cursor */
#define CURSOR_STEADY         0b00000001     /* steady cursor */
#define CURSOR_BLINK          0b00000010     /* blinking cursor */
#define CHECK_OP_MODE         0b00000100     /* consider operation mode */
#define CHECK_KEY_TWICE       0b00001000     /* check for two short presses of the test key */
#define CHECK_BAT             0b00010000     /* check battery */
#define CURSOR_TEXT           0b00100000     /* show hint instead of cursor */


/* keys (test push button etc.) */
#define KEY_NONE              0    /* no key or error */
#define KEY_TIMEOUT           0    /* timeout */
#define KEY_SHORT             1    /* test push button: short key press */
#define KEY_LONG              2    /* test push button: long key press */
#define KEY_TWICE             3    /* test push button: two short key presses */
#define KEY_RIGHT             4    /* rotary encoder: right turn */
                                   /* push buttons: increase */
#define KEY_LEFT              5    /* rotary encoder: left turn */
                                   /* push buttons: decrease */
#define KEY_INCDEC            6    /* push buttons: increase and decrease */

/* virtual keys */
#define KEY_COMMAND           100  /* remote command (from serial interface) */
#define KEY_MAINMENU          101  /* main menu */
#define KEY_POWER_ON          102  /* just powered up */
#define KEY_POWER_OFF         103  /* power off */
#define KEY_PROBE             104  /* probe component */
#define KEY_EXIT              105  /* exit (menu) */
#define KEY_DEFAULTS          106  /* set defaults (adjustments) */


/* operation mode/state flags (bitfield) */
#define OP_NONE               0b00000000     /* no flags */
#define OP_AUTOHOLD           0b00000001     /* auto-hold mode (instead of continuous) */
#define OP_EXT_REF            0b00000100     /* external voltage reference used */
#define OP_SPI                0b00001000     /* SPI is set up */
#define OP_I2C                0b00010000     /* I2C is set up */
#define OP_AUTOHOLD_TEMP      0b00100000     /* temporary auto-hold mode */


/* operation control/signaling flags (bitfield) */
#define OP_BREAK_KEY          0b00000001     /* exit key processing */
#define OP_OUT_LCD            0b00000010     /* output to display */
#define OP_OUT_SER            0b00000100     /* output to TTL serial */
#define OP_RX_LOCKED          0b00001000     /* RX buffer locked */
#define OP_RX_OVERFLOW        0b00010000     /* RX buffer overflow */
#define OP_PWR_TIMEOUT        0b00100000     /* auto-power-off for auto-hold mode */


/* UI line modes (bitfield) */
#define LINE_STD              0b00000000     /* standard mode */
#define LINE_KEY              0b00000001     /* wait for key press */
#define LINE_KEEP             0b00000010     /* keep first line */


/* storage modes (ID and bitfield) */
#define STORAGE_LOAD          0b00000001     /* load adjustment values (ID) */
#define STORAGE_SAVE          0b00000010     /* save adjustment values (ID) */
#define STORAGE_SHORT         0b00000100     /* short menu (flag) */ 


/* SPI */
/* clock rate flags (bitfield) */
#define SPI_CLOCK_R0          0b00000001     /* divider bit 0 (SPR0) */
#define SPI_CLOCK_R1          0b00000010     /* divider bit 1 (SPR1) */
#define SPI_CLOCK_2X          0b00000100     /* double clock rate (SPI2X) */


/* I2C */
#define I2C_ERROR             0         /* bus error */
#define I2C_OK                1         /* operation done */
#define I2C_START             1         /* start condition */
#define I2C_REPEATED_START    2         /* repeated start condition */
#define I2C_DATA              1         /* data byte */
#define I2C_ADDRESS           2         /* address byte */
#define I2C_ACK               1         /* acknowledge */
#define I2C_NACK              2         /* not-acknowledge */


/* TTL serial */
/* control */
#define SER_RX_PAUSE          1         /* pause RX */
#define SER_RX_RESUME         2         /* resume RX */

/* special characters */
#define CHAR_XON              17        /* software flow control: XON */
#define CHAR_XOFF             19        /* software flow control: XOFF */


/* modes for probe pinout */
#define PROBES_PWM            0         /* PWM output */
#define PROBES_ESR            1         /* ESR measurement */
#define PROBES_RCL            2         /* monitoring RCL */
#define PROBES_RINGTESTER     3         /* ring tester */
#define PROBES_DIODE          4         /* diode */


/* E series */
#define E6                    6         /* E6 */
#define E12                  12         /* E12 */
#define E24                  24         /* E24 */
#define E48                  48         /* E48 */
#define E96                  96         /* E96 */
#define E192                192         /* E192 */


/* alignment */
#define ALIGN_LEFT            3         /* align left */
#define ALIGN_RIGHT           4         /* align right */



/* ************************************************************************
 *   constants for arrays in variables.h
 * ************************************************************************ */


/* string buffer sizes */
#define OUT_BUFFER_SIZE       12        /* 11 chars + terminating 0 */
#define RX_BUFFER_SIZE        11        /* 10 chars + terminating 0 */

/* number of entries in data tables */
#define NUM_PREFIXES          8         /* unit prefixes */
#define NUM_LARGE_CAP         46        /* large cap factors */
#define NUM_SMALL_CAP         9         /* small cap factors */
#define NUM_PWM_FREQ          8         /* PWM frequencies */
#define NUM_INDUCTOR          32        /* inductance factors */
#define NUM_TIMER1            5         /* Timer1 prescalers and bits */
#define NUM_PROBE_COLORS      3         /* probe colors */
#define NUM_E6                6         /* E6 norm values */
#define NUM_E12              12         /* E12 norm values */
#define NUM_E24              24         /* E24 norm values */
#define NUM_E96              96         /* E24 norm values */
#define NUM_COLOR_CODES      10         /* color codes */
#define NUM_EIA96_MULT        9         /* EIA-96 multiplier codes */
#define NUM_LOGIC_TYPES       6         /* logic families and voltages */

/* IR code buffer size */
#define IR_CODE_BYTES         6         /* 6 bytes = 48 bit */



/* ************************************************************************
 *   constants for remote commands
 * ************************************************************************ */


/*
 *  command IDs
 */

/* basic commands */
#define CMD_NONE              0    /* no command */
#define CMD_VER               1    /* print firmware version */
#define CMD_OFF               2    /* power off */

/* probing commands */
#define CMD_PROBE             10   /* probe component */
#define CMD_COMP              11   /* return component type ID */
#define CMD_MSG               12   /* return error message */
#define CMD_QTY               13   /* return component quantity */
#define CMD_NEXT              14   /* select next component */
#define CMD_TYPE              15   /* return more sepcific type */
#define CMD_HINT              16   /* return hints on special features */
#define CMD_MHINT             17   /* return hints on measurements */
#define CMD_PIN               18   /* return pinout */
#define CMD_R                 20   /* return resistance */
#define CMD_C                 21   /* return capacitance */
#define CMD_L                 22   /* return inductance */
#define CMD_ESR               23   /* return ESR */
#define CMD_I_L               24   /* return I_leak */
#define CMD_V_F               25   /* return V_f */
#define CMD_V_F2              26   /* return V_f of low current measurement */
#define CMD_C_D               27   /* return C_D */
#define CMD_I_R               28   /* return I_R */
#define CMD_R_BE              29   /* return R_BE */
#define CMD_H_FE              30   /* return hFE */
#define CMD_H_FE_R            31   /* return reverse hFE */
#define CMD_V_BE              32   /* return V_BE */
#define CMD_I_CEO             33   /* return I_CEO */
#define CMD_V_TH              34   /* return V_th */
#define CMD_C_GS              35   /* return C_GS */
#define CMD_R_DS              36   /* return R_DS */
#define CMD_V_GS_OFF          37   /* return V_GS(off) */
#define CMD_I_DSS             38   /* return I_DSS */
#define CMD_C_GE              39   /* return C_GE */
#define CMD_V_GT              40   /* return V_GT */
#define CMD_V_T               41   /* return V_T */
#define CMD_R_BB              42   /* return R_BB */
#define CMD_I_C               43   /* return I_C */
#define CMD_I_E               44   /* return I_E */
#define CMD_V_Z               45   /* return V_Z */
#define CMD_V_L               46   /* return V_loss */
#define CMD_V_F_CLAMP         47   /* return V_f of clamping diode */
#define CMD_C_BE              48   /* return C_BE */



/*
 *  flags for Info_Type
 */

#define INFO_NONE             0b00000000     /* no flags set */

/* resistor */
#define INFO_R_L              0b00000001     /* measured inductance */

/* diode */
#define INFO_D_R_BE           0b00000001     /* detected B-E resistor */
#define INFO_D_BJT_NPN        0b00000010     /* possible NPN BJT */
#define INFO_D_BJT_PNP        0b00000100     /* possible PNP BJT */
#define INFO_D_I_R            0b00001000     /* measured reverse leakage current */
#define INFO_D_CAP1           0b00010000     /* diode #1: measured capacitance */
#define INFO_D_CAP2           0b00100000     /* diode #2: measured capacitance */

/* BJT */
#define INFO_BJT_D_FB         0b00000001     /* detected flyback diode */
#define INFO_BJT_R_BE         0b00000010     /* detected B-E resistor */
#define INFO_BJT_SCHOTTKY     0b00000100     /* detected Schottky-clamped BJT */

/* FET/IGBT */
#define INFO_FET_D_FB         0b00000001     /* detected body/flyback diode */
#define INFO_FET_SYM          0b00000010     /* symmetrical drain and source */
#define INFO_FET_V_TH         0b00000100     /* measured Vth */
#define INFO_FET_C_GS         0b00001000     /* measured C_GS */
#define INFO_FET_R_DS         0b00010000     /* measured R_DS_on */



/* ************************************************************************
 *   constants for probing
 * ************************************************************************ */


/* probe IDs */
#define PROBE_1               0    /* probe #1 */
#define PROBE_2               1    /* probe #2 */
#define PROBE_3               2    /* probe #3 */


/* component IDs */
/* non-components */
#define COMP_NONE             0
#define COMP_ERROR            1
/* passive components */
#define COMP_RESISTOR        10
#define COMP_CAPACITOR       11
#define COMP_INDUCTOR        12
/* 2 pin semiconductors */
#define COMP_DIODE           20
#define COMP_ZENER           21
/* 3 pin semiconductors */
#define COMP_BJT             30
#define COMP_FET             31
#define COMP_IGBT            32
#define COMP_TRIAC           33
#define COMP_THYRISTOR       34
#define COMP_PUT             35
#define COMP_UJT             36


/* error type IDs */
#define TYPE_DISCHARGE        1    /* discharge error */
#define TYPE_DETECTION        2    /* detection error */


/* FET types, also used for IGBTs (bitfield) */
#define TYPE_N_CHANNEL        0b00000001     /* n channel */
#define TYPE_P_CHANNEL        0b00000010     /* p channel */
#define TYPE_ENHANCEMENT      0b00000100     /* enhancement mode */
#define TYPE_DEPLETION        0b00001000     /* depletion mode */
#define TYPE_MOSFET           0b00010000     /* MOSFET */
#define TYPE_JFET             0b00100000     /* JFET */
#define TYPE_SYMMETRICAL      0b01000000     /* symmetrical drain/source */


/* BJT types (bitfield) */ 
#define TYPE_NPN              0b00000001     /* NPN */
#define TYPE_PNP              0b00000010     /* PNP */
#define TYPE_PARASITIC        0b00000100     /* parasitic BJT */


/* diode types (bitfield) */
#define TYPE_STANDARD         0b00000001     /* standard diode */


/* semiconductor flags (bitfield) */
#define HFE_COMMON_EMITTER    0b00000001     /* hFE: common emitter circuit */
#define HFE_COMMON_COLLECTOR  0b00000010     /* hFE: common collector circuit */
#define HFE_CIRCUIT_MASK      0b00000011     /* mask for hFE circuit flags */


/* flags for semicondutor detection logic (bitfield) */
#define DONE_NONE             0b00000000     /* detected nothing / not sure yet */
#define DONE_SEMI             0b00000001     /* detected semi */
#define DONE_ALTSEMI          0b00000010     /* detected alternative semi */


/* multiplicator table IDs */
#define TABLE_SMALL_CAP       1              /* table for small caps */
#define TABLE_LARGE_CAP       2              /* table for large caps */
#define TABLE_INDUCTOR        3              /* table for inductors */


/* bit flags for PullProbe() (bitfield) */
#define PULL_DOWN             0b00000000     /* pull down */
#define PULL_UP               0b00000001     /* pull up */
#define PULL_1MS              0b00001000     /* pull for 1ms */
#define PULL_10MS             0b00010000     /* pull for 10ms */



/* ************************************************************************
 *   constants for display output
 * ************************************************************************ */


/* custom chars/symbols */
#define LCD_CHAR_ZERO         0    /* unused */
#define LCD_CHAR_DIODE_AC     1    /* diode icon '>|' */
#define LCD_CHAR_DIODE_CA     2	   /* diode icon '|<' */
#define LCD_CHAR_CAP          3    /* capacitor icon '||' */
#define LCD_CHAR_OMEGA        4    /* omega */
#define LCD_CHAR_MICRO        5    /* � (micro) */
#define LCD_CHAR_RESISTOR_L   6    /* resistor icon left part '[' */
#define LCD_CHAR_RESISTOR_R   7    /* resistor icon right part ']' */

/* optional custom chars */
#define LCD_CHAR_1_INV        8    /* 1 (reversed color) */
#define LCD_CHAR_2_INV        9    /* 2 (reversed color) */
#define LCD_CHAR_3_INV       10    /* 3 (reversed color) */
#define LCD_CHAR_X_INV       11    /* x (reversed color) */
#define LCD_CHAR_BAT_LL      12    /* battery icon left part: low */
#define LCD_CHAR_BAT_LH      13    /* battery icon left part: high */
#define LCD_CHAR_BAT_RL      14    /* battery icon right part: low */
#define LCD_CHAR_BAT_RH      15    /* battery icon right part: high */


/* basic component symbols */
#define SYMBOL_BJT_NPN        0    /* BJT npn */
#define SYMBOL_BJT_PNP        1    /* BJT pnp */
#define SYMBOL_MOSFET_ENH_N   2    /* MOSFET enhancement mode, n-channel */
#define SYMBOL_MOSFET_ENH_P   3    /* MOSFET enhancement mode, p-channel */
#define SYMBOL_MOSFET_DEP_N   4    /* MOSFET depletion mode, n-channel */
#define SYMBOL_MOSFET_DEP_P   5    /* MOSFET depletion mode, p-channel */
#define SYMBOL_JFET_N         6    /* JFET n-channel */
#define SYMBOL_JFET_P         7    /* JFET p-channel */
#define SYMBOL_IGBT_ENH_N     8    /* IGBT enhancement mode, n-channel */
#define SYMBOL_IGBT_ENH_P     9    /* IGBT enhancement mode, p-channel */
#define SYMBOL_SCR           10    /* SCR / thyristor */
#define SYMBOL_TRIAC         11    /* TRIAC */
#define SYMBOL_PUT           12    /* PUT */
#define SYMBOL_UJT           13    /* UJT */

/* additional component symbols */
#define SYMBOL_QUESTIONMARK  14    /* question mark */
#define SYMBOL_DIODE_ZENER   15    /* Zener diode */
#define SYMBOL_CRYSTAL       16    /* quartz crystal */
#define SYMBOL_ONEWIRE       17    /* OneWire device */

/* number of component symbols */
#ifdef SYMBOLS_EXTRA
  #define NUM_SYMBOLS        18    /* basic plus additional symbols */
#else
  #define NUM_SYMBOLS        14    /* basic symbols */
#endif


/* pinout positions (bitfield) */
#define PIN_NONE              0b00000000     /* no output */
#define PIN_LEFT              0b00000001     /* left */
#define PIN_RIGHT             0b00000010     /* right */
#define PIN_BOTTOM            0b00000100     /* bottom */
#define PIN_TOP               0b00001000     /* top */
#define PIN_CENTER            0b00010000     /* center (vertical) */
#define PIN_ALT_CENTER        0b00100000     /* UI_PINOUT_ALT: center (horizontal) */



/* ************************************************************************
 *   constants for additional hardware
 * ************************************************************************ */


/* passive buzzer */
#define BUZZER_FREQ_LOW       0              /* 2.5 kHz */
#define BUZZER_FREQ_HIGH      1              /* 5 kHz */


/* port pins of PCF8574 I2C IO chip */
#define PCF8574_P0            0b00000000     /* pin #0 */
#define PCF8574_P1            0b00000001     /* pin #1 */
#define PCF8574_P2            0b00000010     /* pin #2 */
#define PCF8574_P3            0b00000011     /* pin #3 */
#define PCF8574_P4            0b00000100     /* pin #4 */
#define PCF8574_P5            0b00000101     /* pin #5 */
#define PCF8574_P6            0b00000110     /* pin #6 */
#define PCF8574_P7            0b00000111     /* pin #7 */


/* port pins of 74HC164 serial to parallel shift register */
#define LOGIC_74HC164_Q0      0b00000000     /* pin #0 */
#define LOGIC_74HC164_Q1      0b00000001     /* pin #1 */
#define LOGIC_74HC164_Q2      0b00000010     /* pin #2 */
#define LOGIC_74HC164_Q3      0b00000011     /* pin #3 */
#define LOGIC_74HC164_Q4      0b00000100     /* pin #4 */
#define LOGIC_74HC164_Q5      0b00000101     /* pin #5 */
#define LOGIC_74HC164_Q6      0b00000110     /* pin #6 */
#define LOGIC_74HC164_Q7      0b00000111     /* pin #7 */


/* port pins of 74HC595 serial to parallel shift register */
#define LOGIC_74HC595_Q0      0b00000000     /* pin #0 */
#define LOGIC_74HC595_Q1      0b00000001     /* pin #1 */
#define LOGIC_74HC595_Q2      0b00000010     /* pin #2 */
#define LOGIC_74HC595_Q3      0b00000011     /* pin #3 */
#define LOGIC_74HC595_Q4      0b00000100     /* pin #4 */
#define LOGIC_74HC595_Q5      0b00000101     /* pin #5 */
#define LOGIC_74HC595_Q6      0b00000110     /* pin #6 */
#define LOGIC_74HC595_Q7      0b00000111     /* pin #7 */



/* ************************************************************************
 *   structures
 * ************************************************************************ */


/* tester modes, states, offsets and values */
typedef struct
{
  uint8_t           OP_Mode;       /* operation mode & state flags */
  volatile uint8_t  OP_Control;    /* operation control & signal flags */
  #ifdef SAVE_POWER
  uint8_t           SleepMode;     /* MCU sleep mode */
  #endif
  uint8_t           Samples;       /* number of ADC samples */
  uint8_t           AutoScale;     /* flag to disable/enable ADC auto scaling */
  uint8_t           Ref;           /* track reference source used lastly */
  uint16_t          Bandgap;       /* voltage of internal bandgap reference (mV) */
  uint16_t          Vcc;           /* voltage of Vcc (mV) */
  #ifndef BAT_NONE
  uint16_t          Vbat;          /* battery voltage (mV) */
  uint8_t           BatTimer;      /* timer for battery check (100ms) */
  #endif
  #ifdef SW_DISPLAY_ID
  uint16_t          DisplayID;     /* ID of display controller */
  #endif
} Config_Type;


/* basic adjustment offsets and values (stored in EEPROM) */
typedef struct
{
  uint16_t          RiL;           /* internal pin resistance of MCU in low mode (0.1 Ohms) */
  uint16_t          RiH;           /* internal pin resistance of MCU in high mode (0.1 Ohms) */

  #ifdef R_MULTIOFFSET
    uint16_t        RZero[3];      /* resistance of probe leads (2 in series) (0.01 Ohms) */
  #else
    uint16_t        RZero;         /* resistance of probe leads (2 in series) (0.01 Ohms) */
  #endif

  #ifdef CAP_MULTIOFFSET
    uint8_t         CapZero[3];    /* capacitance zero offsets (PCB+leads) (pF) */
  #else
    uint8_t         CapZero;       /* capacitance zero offset (PCP+leads) (pF) */
  #endif

  int8_t            RefOffset;     /* voltage offset of bandgap reference (mV) */
  int8_t            CompOffset;    /* voltage offset of analog comparator (mV) */
  uint8_t           Contrast;      /* contrast value of display */
  uint8_t           CheckSum;      /* checksum for stored values */
} Adjust_Type;


/* touch screen adjustment values (stored in EEPROM) */
typedef struct
{
  uint16_t          X_Start;       /* X start value */
  uint16_t          X_Stop;        /* X stop value */
  uint16_t          Y_Start;       /* Y start value */
  uint16_t          Y_Stop;        /* Y stop value */
  uint8_t           CheckSum;      /* checksum for stored values */
} Touch_Type;


/* user interface */
typedef struct
{
  /* display */
  uint8_t           LineMode;      /* line mode for LCD_NextLine() */
  uint8_t           CharPos_X;     /* current character x position */
  uint8_t           CharPos_Y;     /* current character y position */
                                   /* top left is 1/1 */
  uint8_t           CharMax_X;     /* max. characters per line */
  uint8_t           CharMax_Y;     /* max. number of lines */
  uint8_t           MaxContrast;   /* maximum contrast */

  /* color support */
  #ifdef LCD_COLOR
  uint16_t          PenColor;      /* pen color */
    #if defined (UI_COLORED_TITLES) || defined (UI_COLORED_VALUES)
    uint16_t          OldColor;      /* old color */
    #endif
  #endif


  /* fancy pinout with symbols */
  #ifdef SW_SYMBOLS
  uint8_t           SymbolLine;    /* line for output */
  uint8_t           SymbolSize_X;  /* x size in characters */
  uint8_t           SymbolSize_Y;  /* y size in characters */
  uint8_t           SymbolPos_X;   /* x char position (left) */
  uint8_t           SymbolPos_Y;   /* y char position (top) */
  #endif

  /* additional keys (push buttons etc.) */
  #ifdef HW_KEYS
  uint8_t           KeyOld;        /* former key */
  uint8_t           KeyStep;       /* step size (1-7) */
  uint8_t           KeyStepOld;    /* former step size */
  #endif

  /* rotary encoder */
  #ifdef HW_ENCODER
  uint8_t           EncState;      /* last AB status */
  uint8_t           EncDir;        /* turning direction */
  uint8_t           EncPulses;     /* number of Gray code pulses */
  uint8_t           EncTicks;      /* time counter */
  #endif

  /* increase/decrease push buttons */
  #ifdef HW_INCDEC_KEYS
  /* no additional variables needed */
  #endif

  /* touch screen */
  #ifdef HW_TOUCH
  uint16_t          TouchRaw_X;    /* raw touch screen x position */
  uint16_t          TouchRaw_Y;    /* raw touch screen y position */
  uint8_t           TouchPos_X;    /* charater x position */
  uint8_t           TouchPos_Y;    /* charater y position */
  #endif

  /* key hints */
  #ifdef UI_KEY_HINTS
  unsigned char     *KeyHint;      /* string pointer (EEPROM) */
  #endif

} UI_Type;


/* probes */
typedef struct
{
  /* probe IDs */
  uint8_t           ID_1;          /* probe-1 */
  uint8_t           ID_2;          /* probe-2 */
  uint8_t           ID_3;          /* probe-3 */

  /* backup probe IDs */
  uint8_t           ID2_1;         /* probe-1 */
  uint8_t           ID2_2;         /* probe-2 */
  uint8_t           ID2_3;         /* probe-3 */

  /* register bits for switching probes and test resistors */
  uint8_t           Rl_1;          /* Rl mask for probe-1 */
  uint8_t           Rl_2;          /* Rl mask for probe-2 */
  uint8_t           Rl_3;          /* Rl mask for probe-3 */
  uint8_t           Rh_1;          /* Rh mask for probe-1 */
  uint8_t           Rh_2;          /* Rh mask for probe-2 */
  uint8_t           Rh_3;          /* Rh mask for probe-3 */
  uint8_t           Pin_1;         /* pin mask for probe-1 */
  uint8_t           Pin_2;         /* pin mask for probe-2 */
  uint8_t           Pin_3;         /* pin mask for probe-3 */
  uint8_t           Ch_1;          /* ADC MUX input channel for probe-1 */
  uint8_t           Ch_2;          /* ADC MUX input channel for probe-2 */
  uint8_t           Ch_3;          /* ADC MUX input channel for probe-3 */
} Probe_Type;


/* checking/probing */
typedef struct
{
  uint8_t           Found;         /* component type */ 
  uint8_t           Type;          /* component specific subtype */
  uint8_t           Done;          /* flag for transistor detection done */
  uint8_t           AltFound;      /* alternative component type */
  uint8_t           Resistors;     /* number of resistors found */
  uint8_t           Diodes;        /* number of diodes found */
  uint8_t           Probe;         /* error: probe pin */ 
  uint16_t          U;             /* error: voltage in mV */
  #ifdef SW_SYMBOLS
  uint8_t           Symbol;        /* symbol ID */
  uint8_t           AltSymbol;     /* symbol ID for alternative component */
  #endif
} Check_Type;


/* resistor */
typedef struct
{
  uint8_t           A;             /* probe pin #1 */
  uint8_t           B;             /* probe pin #2 */
  int8_t            Scale;         /* exponent of factor (value * 10^x) */
  unsigned long     Value;         /* resistance */
} Resistor_Type;


/* capacitor */
typedef struct
{
  uint8_t           A;             /* probe pin #1 */
  uint8_t           B;             /* probe pin #2 */
  int8_t            Scale;         /* exponent of factor (value * 10^x) */
  unsigned long     Value;         /* capacitance incl. zero offset */
  unsigned long     Raw;           /* capacitance excl. zero offset */
  uint16_t          I_leak_Value;  /* leakage current: value (in A) */
  int8_t            I_leak_Scale;  /* leakage current: exponent (10^x) */
  #ifdef SW_C_VLOSS
  uint16_t          U_loss;        /* voltage loss (in 0.1%) */
  #endif
} Capacitor_Type;


/* inductor */
typedef struct
{
  int8_t            Scale;         /* exponent of factor (value * 10^x) */
  unsigned long     Value;         /* inductance */  
} Inductor_Type;


/* diode */
typedef struct
{
  uint8_t           A;             /* probe pin connected to anode */
  uint8_t           C;             /* probe pin connected to cathode */
  uint16_t          V_f;           /* forward voltage in mV (high current) */
  uint16_t          V_f2;          /* forward voltage in mV (low current) */
} Diode_Type;


/* common semiconductors */
typedef struct
{
  uint8_t           A;             /* probe connected to pin A [0-2] */
  uint8_t           B;             /* probe connected to pin B [0-2] */
  uint8_t           C;             /* probe connected to pin C [0-2] */
  uint8_t           DesA;          /* designator for pin A (char) */
  uint8_t           DesB;          /* designator for pin B (char) */
  uint8_t           DesC;          /* designator for pin C (char) */
  uint8_t           Flags;         /* misc flags */
  uint16_t          U_1;           /* voltage #1 */
  int16_t           U_2;           /* voltage #2 (+/-) */
  int16_t           U_3;           /* voltage #3 (+/-) */
  uint32_t          F_1;           /* factor #1 */
  #ifdef SW_REVERSE_HFE
  uint32_t          F_2;           /* factor #2 */
  #endif
  uint32_t          I_value;       /* current */
  int8_t            I_scale;       /* exponent of factor (value * 10^x) */
  uint32_t          C_value;       /* capacitance */
  int8_t            C_scale;       /* exponent of factor (value * 10^x) */
} Semi_Type;

/* 
  Mapping

           BJT          FET          SCR          Triac        IGBT
  ----------------------------------------------------------------------
  A        Base         Gate         Gate         Gate         Gate
  B        Collector    Drain        Anode        MT2          Collector
  C        Emitter      Source       Cathode      MT1          Emitter
  U_1      V_BE (mV)    R_DS (0.01)  V_GT (mV)    V_GT (mV)
  U_2      I_E (�A)     V_th (mV)                              V_th (mV)
  U_3      I_C/E (�A)   V_GS(off)
  F_1      hFE                                    MT2 (mV)
  F_2      hFEr
  I_value  I_CEO        I_DSS
  I_scale  I_CEO        I_DSS
  C_value  C_EB/BE
  C_scale  C_EB/BE

           Zener
  ----------------------------------------------------------------------
  U_1      V_Z (mV)
*/


/* special semiconductors */
typedef struct
{
  uint8_t           A;             /* probe connected to pin A [0-2] */
  uint8_t           B;             /* probe connected to pin B [0-2] */
  uint8_t           C;             /* probe connected to pin C [0-2] */
  uint16_t          U_1;           /* voltage #1 */
  uint16_t          U_2;           /* voltage #2 */
} AltSemi_Type;

/* 
  Mapping

          PUT         UJT
  ------------------------------------------------------------------
  A       Gate        Emitter
  B       Anode       B2
  C       Cathode     B1
  U_1�    V_f
  U_2     V_T
*/


/* additional component data for remote commands */
typedef struct
{
  /* common stuff */
  uint8_t           Quantity;      /* component quantity */
  uint8_t           Selected;      /* selected component */
  uint8_t           Flags;         /* misc flags */
  void              *Comp1;        /* pointer to component #1 */
  void              *Comp2;        /* pointer to component #2 */
  uint16_t          Val1;          /* value #1 */
} Info_Type;

/* 
  Mapping

          R   C           D    BJT        FET    IGBT
  -----------------------------------------------------------------
  Comp1   R1  C           D1   D_FB       D_FB   D_FB
  Comp2   R2              D2
  Val1        ESR (0.01)       V_BE (mV)
*/


/* SPI */
typedef struct
{
  uint8_t           ClockRate;     /* clock rate bits */
} SPI_Type;


/* I2C */
typedef struct
{
  uint8_t           Byte;          /* address/data byte */
  uint8_t           Timeout;       /* ACK timeout in 10�s */
} I2C_Type;


/* remote command */
typedef struct
{
  uint8_t                ID;       /* command ID */
  const unsigned char    *Cmd;     /* storage address of command string */
} Cmd_Type;



/* ************************************************************************
 *   EOF
 * ************************************************************************ */
