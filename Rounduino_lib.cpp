/** ===========================================================
 * Rounduino
 *
 * \file    Rounduino_lib.c
 * \author  jh
 * \date    xx.02.2015
 *
 * \uC      ATmega32U4
 * \f_clk   ext. 8MHz
 *
 * \version 1.0
 *
 * \brief   Source file to control the DINGGLABS Rounduino
 *
 * @{
 ============================================================== */

/* includes --------------------------------------------------- */
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <SPI.h>
#include <Wire.h>

#include "Rounduino_lib.h"

/* private typedef -------------------------------------------- */
/* private define --------------------------------------------- */
/* private macro ---------------------------------------------- */

/* private variables ------------------------------------------ */
/* symbol list */
static struct symbol symbolListMemory[MAX_NUMBER_OF_SYMBOLS] = {0};

/* piezo interrupt counter */
static volatile unsigned int timerInterruptCounterPiezo = 0;

/* global variables ------------------------------------------- */
/* list pointer */
struct symbol *head = NULL;
struct symbol *tail = NULL;

/* customSymbol */
byte customSymbol[CUSTOM_SYMBOL_SIZE][CUSTOM_SYMBOL_WIDTH / NUMBER_OF_PIXELS_PER_BYTE] = {0};

/* time */
struct rtc Time;

/* piezo */
boolean piezoOn = false;                 // variable to turn the piezo on and off
unsigned int piezoFrequencyDivisor = STD_PIEZO_FREQUENCY_DIVISOR; // divisor to manipulate piezo tone

/* private function prototypes -------------------------------- */
/* functions to handle the device itself */
ISR(INT2_vect);
ISR(INT3_vect);
ISR(INT6_vect);
ISR(TIMER3_COMPA_vect);  // piezo interrupt handler

static void initTimerInterrupt_CTC_0();
static void initTimerInterrupt_CTC_1();
static void initTimerInterrupt_CTC_3(); // piezo interrupt initialization

/* functions to manipulate the symbols in the symbol list */
static struct symbol *addSymbol(struct symbol *s);
static void addSymbolHead(struct symbol *s);
static void addSymbolTail(struct symbol *s);
static void addSymbolInFront(struct symbol *s1, struct symbol *s2);
static void addSymbolBehind(struct symbol *s1, struct symbol *s2);

static struct symbol *popSymbol(struct symbol *s);
static struct symbol *popSymbolHead(void);
static struct symbol *popSymbolTail(void);

static void putSymbolHead(struct symbol *s);
static void putSymbolTail(struct symbol *s);
static void putSymbolInFront(struct symbol *s1, struct symbol *s2);
static void putSymbolBehind(struct symbol *s1, struct symbol *s2);

static void replaceSymbol(struct symbol *s1, struct symbol *s2);

static void deleteSymbol(struct symbol *s);

// static void bubblesortSymbolsY(struct symbol s[], int length); //todo
// static void bubblesortSymbolsX(struct symbol s[], int length); //todo

/* functions to get informations of the symbol list */
static byte getListPosOfFirstEmptySymbol();
// static byte getListPosOfSymbol(struct symbol *s);

/* functions to handle the display */
static void initDisplay();
static void displayStartup();

static void setDisplayAddress(int x, int y);
static void setDisplayAddress(int x, int x_, int y, int y_);
static void writeCommand(byte command);
static void writeData(byte data);
static void writeByte(byte byteValue);

/* function to handle ADCs */
static void initADC();

/* functions to handle the RTC */
static void initRTC();
static void setHourTimeFormat24(boolean b);

static int bcd2bin(int val);
static int bin2bcd(int val);

/* other functions */
static int sgn(int a);
/* ------------------------------------------------------------ */

/* functions to handle the device itself ---------------------- */
/** ===========================================================
 * \fn      initRounduino
 * \brief   Rounduino setup-function with all initializations
 *
 * \param   -
 * \return  -
 ============================================================== */
void initRounduino()
{
  #ifdef DEBUGGING
  Serial.begin(BAUDRATE);
  while(!Serial);
  Serial.println("-------------------------");
  Serial.println("init Rounduino");
  #endif
    
  /* enable all interrupts */
  sei();

  /* initialize pins */
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);

  pinMode(D5, OUTPUT);
  pinMode(D8, OUTPUT);
  pinMode(D9, OUTPUT);
  pinMode(D10, OUTPUT);

  pinMode(BUTTON1, INPUT);
  digitalWrite(BUTTON1, HIGH);   // enable internal pullup
  pinMode(BUTTON2, INPUT);
  digitalWrite(BUTTON2, HIGH);   // enable internal pullup
//  pinMode(BUTTON3, INPUT);
//  digitalWrite(BUTTON3, HIGH);   // enable internal pullup
  clearBit(BUTTON3_DDR, BUTTON3_PIN_NR); // input
  setBit(BUTTON3_PORT, BUTTON3_PIN_NR);  // enable internal pullup

  pinMode(V_SCAN, INPUT);
  pinMode(RTC_MFP, INPUT);

  pinMode(EN_BOOST, OUTPUT);
  digitalWrite(EN_BOOST, LOW);   // LOW = OFF, HIGH = ON
  pinMode(D_C, OUTPUT);          // 1 = data, 0 = command
  pinMode(RST, OUTPUT);
  digitalWrite(RST, HIGH);       // default is HIGH (enabled)
  pinMode(PIEZO, OUTPUT);
  
  /* initialize HW SPI */
  SPI.begin();                   // (DATA, SCK and CS)
  SPI.setClockDivider(SPI_CLOCK_DIV2);  // t_byte = divider / f_clk * 8 (default: divider = 4)

  /* initialize I2C/TWI */
  Wire.begin();
  
  /* initialize clock */
  initRTC();
  getCurrentTime();  // initialize time structure

  /* initialize ADC */
  initADC(); 
  
  /* initialize timer interrupt for the piezo */
  initTimerInterrupt_CTC_3();
  
  /* initialize display */
  initDisplay();
  clearDisplay();
}

/** ===========================================================
 * \fn      turnOff
 * \brief   turns off the device by putting it in a power down
 *          sleep-mode, it will wake up again if INTN occures
 *
 * \requ    <avr/interrupt.h> and <avr/sleep.h>
 *
 * \param   -
 * \return  -
 ============================================================== */
void turnOff()
{
  /* disable peirphery */
  displaySleep(true);

  /* all interrupts have to be disabled during turn off configurations */
  cli();

  /* define that a low-level of INT2 and 3 generates an interrupt request */
  EICRA &= ~(1 << ISC21) & ~(1 << ISC20);  // clear bit 5 and 4 to choose low-level-trigger
  EICRA &= ~(1 << ISC31) & ~(1 << ISC30);  // clear bit 7 and 6 to choose low-level-trigger
//  EICRB &= ~(1 << ISC61) & ~(1 << ISC60);  // clear bit 5 and 4 to choose low-level-trigger

  /* clear interrupt flag */
  EIFR |= (1 << INTF2) | (1 << INTF3);       //set bit 2 and 3

  /* enable external interrupt INT2 and 3 (to wake up later) */
  EIMSK |= (1 << INT2) | (1 << INT3);

  /* choose "Power-down" sleep mode */
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  //  SMCR |= (1 << SM1);    // set bit 2

  /* enable sleep-mode */
  sleep_enable();         
  //  SMCR |= (1 << SE);     // set bit 0

  /* reenable interrupts */
  sei();

  /* system actually sleeps here */
  sleep_cpu();

  /* zzzZZZzzzZZZ... */

  /* INT2 or 3 (BUTTON 1 & 2) occured -> wakes up! system continues here... */
  
  /* enable peirphery */
  displaySleep(false);
}

/** ===========================================================
 * \fn      ISR (external interrupt INT2)
 * \brief   interrupt service routine (handler) for the wake-up-
 *          interrupt (INT2)
 *
 * \requ    <avr/sleep.h>
 *
 * \param   'INT2 vector'
 * \return  -
 ============================================================== */
ISR(INT2_vect)
{
  /* disable sleep-mode */
  sleep_disable();
  //  SMCR &= ~(1 << SM1)      // reset bit 0

  /* disable external interrupts */
  EIMSK &= ~(1 << INT2) & ~(1 << INT3) & ~(1 << INT6);
}

/** ===========================================================
 * \fn      ISR (external interrupt INT3)
 * \brief   interrupt service routine (handler) for the wake-up-
 *          interrupt (INT3)
 *
 * \requ    <avr/sleep.h>
 *
 * \param   'INT3 vector'
 * \return  -
 ============================================================== */
ISR(INT3_vect)
{
  /* disable sleep-mode */
  sleep_disable();
  //  SMCR &= ~(1 << SM1)      // reset bit 0

  /* disable external interrupts */
  EIMSK &= ~(1 << INT2) & ~(1 << INT3) & ~(1 << INT6);
}

/** ===========================================================
 * \fn      ISR (external interrupt INT6)
 * \brief   interrupt service routine (handler) for the wake-up-
 *          interrupt (INT6)
 *
 * \requ    <avr/sleep.h>
 *
 * \param   'INT6 vector'
 * \return  -
 ============================================================== */
ISR(INT6_vect)
{  
  /* disable sleep-mode */
  sleep_disable();
  //  SMCR &= ~(1 << SM1)      // reset bit 0

  /* disable external interrupts */
  EIMSK &= ~(1 << INT2) & ~(1 << INT3) & ~(1 << INT6);
}

/** ===========================================================
 * \fn      ISR (timer3 OCIE3A interrupt TIMER3_COMPA)
 * \brief   interrupt service routine (handler) which will be 
 *          called every few micro seconds to control the
 *          piezo
 *          (f_isr = fcpu / (prescaler * cnt))
 *
 * \param   'TIMER3_COMPA vector'
 * \return  -
 ============================================================== */
ISR(TIMER3_COMPA_vect)
{
  /* increase global timer interrupt counter */
  timerInterruptCounterPiezo++; 
      
  /* check if it is time to handle the piezo */
  if (timerInterruptCounterPiezo >=  piezoFrequencyDivisor)
  {
    /* reset global piezo timer interrupt counter */
    timerInterruptCounterPiezo = 0;
  
    /* toggle the piezo on base of current desired state */
    if (piezoOn) PIEZO_PORT ^= (1 << PIEZO_PIN);  // PIEZO = PC7
    else PIEZO_PORT &= ~(1 << PIEZO_PIN);         // turn piezo off (high active)
  }
}

/** ===========================================================
* \fn      initTimerInterrupt_CTC_0
* \brief   initializes the timer 0 and releated Clear Timer on
*          Compare match (CTC) interrupt
*          (ATmega32U4)
*
* \param   -
* \return  -
============================================================== */
static void initTimerInterrupt_CTC_0()
{
 /* init timer0 and releated interrupt (t_isr = 1/(8MHz/(prescaler*cnt))) */
 TCCR0A = (1 << WGM01);      // enable CTC (Clear Timer on Compare match) mode
 TCCR0B = 0x02;              // prescaler = 8 (0x01 -> /1, 0x02 -> /8, 0x03 -> /64, ...)
   
 TIMSK0 = (1 << OCIE0A);     // set bit 1 to enable timer0 output compare match A interrupt (TIMER0_COMPA)
 OCR0A = INTERRUPT_FREQUENCY_DIVISOR_0;  // define output compare register A value (0... 255)

 /* reset timer 0 counter register */
 TCNT0 = 0x00;
}

/** ===========================================================
* \fn      initTimerInterrupt_CTC_1
* \brief   initializes the timer 1 and releated Clear Timer on
*          Compare match (CTC) interrupt
*          (ATmega32U4)
*
* \param   -
* \return  -
============================================================== */
static void initTimerInterrupt_CTC_1()
{
 /* init timer1 and releated interrupt (t_isr = 1/(8MHz/(prescaler*cnt))) */
 TCCR1A = 0x00;         // enable CTC (Clear Timer on Compare match) mode
 TCCR1B = ((1 << WGM12) | 0x02);  // prescaler = 8 (0x01 -> /1, 0x02 -> /8, 0x03 -> /64, ...)
   
 TIMSK1 = (1 << OCIE1A);     // set bit 1 to enable timer1 output compare match A interrupt (TIMER1_COMPA)
 OCR1A = INTERRUPT_FREQUENCY_DIVISOR_1;  // define output compare register A value (0... 65535)

 /* reset timer 1 counter register */
 TCNT1 = 0x00;
}

/** ===========================================================
 * \fn      initTimerInterrupt_CTC_3
 * \brief   initializes the timer 3 and releated Clear Timer on
 *          Compare match (CTC) interrupt
 *          (ATmega32U4)
 *
 * \param   -
 * \return  -
 ============================================================== */
static void initTimerInterrupt_CTC_3()
{
  /* init timer2 and releated interrupt (t_isr = 1/(8MHz/(prescaler*cnt))) */
  TCCR3A = 0x00;      // enable CTC (Clear Timer on Compare match) mode
  TCCR3B = ((1 << WGM32) | 0x02);      // prescaler = 8 (0x01 -> /1, 0x02 -> /8, 0x03 -> /32, 0x04 -> /64, ...)
    
  TIMSK3 = (1 << OCIE3A);     // set bit 1 to enable timer0 output compare match A interrupt (TIMER0_COMPA)
  OCR3A = INTERRUPT_FREQUENCY_DIVISOR_2;  // define output compare register A value (0... 255)

  /* reset timer 0 counter register */
  TCNT3 = 0x00;
}

/* functions to generate symbols ------------------------------ */
/** ===========================================================
 * \fn      createCircleSymbol
 * \brief   adds a circle symbol in the symbol list
 *
 * \requ    addSymbolTail()
 *
 * \param   (byte) x, y top left coordinate of the symbol
 *          (byte) brightness (0... MAX_BRIGHTNESS)
 * \return  -
 ============================================================== */
void createCircleSymbol(byte x, byte y, byte b)
{
  /* limit input paramters */
  if (x >= NUMBER_OF_PIXELS_PER_ROW) x = NUMBER_OF_PIXELS_PER_ROW - 1;
  if (y >= NUMBER_OF_PIXELS_PER_COLUMN) y = NUMBER_OF_PIXELS_PER_COLUMN - 1;
  if (b > MAX_BRIGHTNESS) b = MAX_BRIGHTNESS;

  /* recalculate symbol position if rotated */
  #ifdef ROTATED_CW
  byte temp = x;
  x = NUMBER_OF_PIXELS_PER_COLUMN - y - FONTSIZE;
  y = temp;
  #endif
  
  #ifdef ROTATED_ACW
  byte temp = y;
  y = NUMBER_OF_PIXELS_PER_ROW - x - FONTWIDTH;
  x = temp;
  #endif
  
  // circle c = 0
  struct symbol s = {(byte)(getNumberOfSymbols() + 1), false, (byte)(PICTURE_NAME_OFFSET + PICTURE_ARRAY_POS_CIRCLE), x, y, FONTWIDTH_BIG, FONTSIZE_BIG, b, NULL, NULL};
  addSymbolTail(&s);
}

/** ===========================================================
 * \fn      createTriangleSymbol
 * \brief   adds a triangle symbol in the symbol list
 *
 * \requ    addSymbolTail()
 *
 * \param   (byte) x, y top left coordinate of the symbol
 *          (byte) brightness (0... MAX_BRIGHTNESS)
 * \return  -
 ============================================================== */
void createTriangleSymbol(byte x, byte y, byte b)
{
  /* limit input paramters */
  if (x >= NUMBER_OF_PIXELS_PER_ROW) x = NUMBER_OF_PIXELS_PER_ROW - 1;
  if (y >= NUMBER_OF_PIXELS_PER_COLUMN) y = NUMBER_OF_PIXELS_PER_COLUMN - 1;
  if (b > MAX_BRIGHTNESS) b = MAX_BRIGHTNESS;

  /* recalculate symbol position if rotated */
  #ifdef ROTATED_CW
  byte temp = x;
  x = NUMBER_OF_PIXELS_PER_COLUMN - y - FONTSIZE;
  y = temp;
  #endif
  
  #ifdef ROTATED_ACW
  byte temp = y;
  y = NUMBER_OF_PIXELS_PER_ROW - x - FONTWIDTH;
  x = temp;
  #endif
  
  // circle c = 0
  struct symbol s = {(byte)(getNumberOfSymbols() + 1), false, (byte)(PICTURE_NAME_OFFSET + PICTURE_ARRAY_POS_TRIANGLE), x, y, FONTWIDTH_BIG, FONTSIZE_BIG, b, NULL, NULL};
  addSymbolTail(&s);
}

/** ===========================================================
 * \fn      createSquareSymbol
 * \brief   adds a square symbol in the symbol list
 *
 * \requ    addSymbolTail()
 *
 * \param   (byte) x, y top left coordinate of the symbol
 *          (byte) brightness (0... MAX_BRIGHTNESS)
 * \return  -
 ============================================================== */
void createSquareSymbol(byte x, byte y, byte b)
{
  /* limit input paramters */
  if (x >= NUMBER_OF_PIXELS_PER_ROW) x = NUMBER_OF_PIXELS_PER_ROW - 1;
  if (y >= NUMBER_OF_PIXELS_PER_COLUMN) y = NUMBER_OF_PIXELS_PER_COLUMN - 1;
  if (b > MAX_BRIGHTNESS) b = MAX_BRIGHTNESS;

  /* recalculate symbol position if rotated */
  #ifdef ROTATED_CW
  byte temp = x;
  x = NUMBER_OF_PIXELS_PER_COLUMN - y - FONTSIZE;
  y = temp;
  #endif
  
  #ifdef ROTATED_ACW
  byte temp = y;
  y = NUMBER_OF_PIXELS_PER_ROW - x - FONTWIDTH;
  x = temp;
  #endif
  
  // circle c = 0
  struct symbol s = {(byte)(getNumberOfSymbols() + 1), false, (byte)(PICTURE_NAME_OFFSET + PICTURE_ARRAY_POS_SQUARE), x, y, FONTWIDTH_BIG, FONTSIZE_BIG, b, NULL, NULL};
  addSymbolTail(&s);
}

/** ===========================================================
 * \fn      createLArrowSymbol
 * \brief   adds a left arrow symbol in the symbol list
 *
 * \requ    addSymbolTail()
 *
 * \param   (byte) x, y top left coordinate of the symbol
 *          (byte) brightness (0... MAX_BRIGHTNESS)
 * \return  -
 ============================================================== */
void createLArrowSymbol(byte x, byte y, byte b)
{
  /* limit input paramters */
  if (x >= NUMBER_OF_PIXELS_PER_ROW) x = NUMBER_OF_PIXELS_PER_ROW - 1;
  if (y >= NUMBER_OF_PIXELS_PER_COLUMN) y = NUMBER_OF_PIXELS_PER_COLUMN - 1;
  if (b > MAX_BRIGHTNESS) b = MAX_BRIGHTNESS;

  /* recalculate symbol position if rotated */
  #ifdef ROTATED_CW
  byte temp = x;
  x = NUMBER_OF_PIXELS_PER_COLUMN - y - FONTSIZE;
  y = temp;
  #endif
  
  #ifdef ROTATED_ACW
  byte temp = y;
  y = NUMBER_OF_PIXELS_PER_ROW - x - FONTWIDTH;
  x = temp;
  #endif
  
  // circle c = 0
  struct symbol s = {(byte)(getNumberOfSymbols() + 1), false, (byte)(PICTURE_NAME_OFFSET + PICTURE_ARRAY_POS_LARROW), x, y, FONTWIDTH_BIG, FONTSIZE_BIG, b, NULL, NULL};
  addSymbolTail(&s);
}

/** ===========================================================
 * \fn      createRArrowSymbol
 * \brief   adds a right arrow symbol in the symbol list
 *
 * \requ    addSymbolTail()
 *
 * \param   (byte) x, y top left coordinate of the symbol
 *          (byte) brightness (0... MAX_BRIGHTNESS)
 * \return  -
 ============================================================== */
void createRArrowSymbol(byte x, byte y, byte b)
{
  /* limit input paramters */
  if (x >= NUMBER_OF_PIXELS_PER_ROW) x = NUMBER_OF_PIXELS_PER_ROW - 1;
  if (y >= NUMBER_OF_PIXELS_PER_COLUMN) y = NUMBER_OF_PIXELS_PER_COLUMN - 1;
  if (b > MAX_BRIGHTNESS) b = MAX_BRIGHTNESS;

  /* recalculate symbol position if rotated */
  #ifdef ROTATED_CW
  byte temp = x;
  x = NUMBER_OF_PIXELS_PER_COLUMN - y - FONTSIZE;
  y = temp;
  #endif
  
  #ifdef ROTATED_ACW
  byte temp = y;
  y = NUMBER_OF_PIXELS_PER_ROW - x - FONTWIDTH;
  x = temp;
  #endif
  
  // circle c = 0
  struct symbol s = {(byte)(getNumberOfSymbols() + 1), false, (byte)(PICTURE_NAME_OFFSET + PICTURE_ARRAY_POS_RARROW), x, y, FONTWIDTH_BIG, FONTSIZE_BIG, b, NULL, NULL};
  addSymbolTail(&s);
}

/** ===========================================================
 * \fn      createBigNumberSymbol
 * \brief   adds a big number symbol in the symbol list
 *
 * \requ    addSymbolTail()
 *
 * \param   (uchar) character
 *          (byte)  x, y top left coordinate of the character
 *          (byte)  brightness (0... MAX_BRIGHTNESS)
 * \return  -
 ============================================================== */
void createBigNumberSymbol(unsigned char c, byte x, byte y, byte b)
{
  /* limit input paramters */
  if (c > '?') c = '?';
  else if (c < ' ') c = ' ';
  if (x >= NUMBER_OF_PIXELS_PER_ROW - FONTWIDTH_BIG) x = NUMBER_OF_PIXELS_PER_ROW - FONTWIDTH_BIG - 1;
  if (y >= NUMBER_OF_PIXELS_PER_COLUMN - FONTSIZE_BIG) y = NUMBER_OF_PIXELS_PER_COLUMN - FONTSIZE_BIG - 1;
  if (b > MAX_BRIGHTNESS) b = MAX_BRIGHTNESS;

  /* recalculate symbol position if rotated */
  #ifdef ROTATED_CW
  byte temp = x;
  x = NUMBER_OF_PIXELS_PER_COLUMN - y - FONTSIZE_BIG;
  y = temp;
  #endif
  
  #ifdef ROTATED_ACW
  byte temp = y;
  y = NUMBER_OF_PIXELS_PER_ROW - x - FONTWIDTH_BIG;
  x = temp;
  #endif
  
  struct symbol s = {(byte)(getNumberOfSymbols() + 1), false, (byte)(c - ASCII_FIRST_CHARACTER_OFFSET), x, y, FONTWIDTH_BIG, FONTSIZE_BIG, b, NULL, NULL};
  addSymbolTail(&s);
}

/** ===========================================================
 * \fn      drawBigNumbers
 * \brief   adds multiple big number symbols in the symbol list
 *          and draws them on given position on the display
 *
 * \requ    createBigNumberSymbol(), drawSymbols()
 *
 * \param   (uchar[]) constant string (char array)
 *          (byte)    x, y top left coordinate of the string
 *          (byte)    brightness (0... MAX_BRIGHTNESS)
 *          
 * \return  -
 ============================================================== */
void drawBigNumbers(const unsigned char str[], byte x, byte y, byte b)
{  
  byte ofCnt = 0;  // overflow counter
  
  /* limit input paramters */
  if (x >= NUMBER_OF_PIXELS_PER_ROW - FONTWIDTH_BIG) x = NUMBER_OF_PIXELS_PER_ROW - FONTWIDTH_BIG - 1;
  if (y >= NUMBER_OF_PIXELS_PER_COLUMN - FONTSIZE_BIG) y = NUMBER_OF_PIXELS_PER_COLUMN - FONTSIZE_BIG - 1;
  if (b > MAX_BRIGHTNESS) b = MAX_BRIGHTNESS;

  for (unsigned int i = 0; str[i] != '\0'; i++)
  {     
    byte currentX = x + i * (FONTWIDTH_BIG - CHAR_SPACING_OFFSET_BIG);
    
    /* check if end of row woulde be reached */
    if (currentX + FONTWIDTH_BIG > NUMBER_OF_PIXELS_PER_ROW)
    {
      /* limit owerflow counter */
      ofCnt %=  (NUMBER_OF_PIXELS_PER_ROW / (FONTWIDTH_BIG - CHAR_SPACING_OFFSET_BIG));
      if (ofCnt == 0) y += FONTSIZE_BIG;
      
      currentX = ofCnt * (FONTWIDTH_BIG - CHAR_SPACING_OFFSET_BIG);
      ofCnt++;
    }
    
    //blup
    if (i % 2 == 0)
    {
      
    }
    createBigNumberSymbol((byte)str[i], currentX, y, b);
  }

  drawSymbols();
}

/** ===========================================================
 * \fn      createCharSymbol
 * \brief   adds a char symbol in the symbol list
 *
 * \requ    addSymbolTail()
 *
 * \param   (uchar) character
 *          (byte)  x, y top left coordinate of the character
 *          (byte)  brightness (0... MAX_BRIGHTNESS)
 * \return  -
 ============================================================== */
void createCharSymbol(unsigned char c, byte x, byte y, byte b)
{
  /* limit input paramters */
  if (c > '~') c = ' ';
  else if (c < ' ') c = ' ';
  if (x >= NUMBER_OF_PIXELS_PER_ROW - FONTWIDTH) x = NUMBER_OF_PIXELS_PER_ROW - FONTWIDTH - 1;
  if (y >= NUMBER_OF_PIXELS_PER_COLUMN - FONTSIZE) y = NUMBER_OF_PIXELS_PER_COLUMN - FONTSIZE - 1;
  if (b > MAX_BRIGHTNESS) b = MAX_BRIGHTNESS;

  /* recalculate symbol position if rotated */
  #ifdef ROTATED_CW
  byte temp = x;
  x = NUMBER_OF_PIXELS_PER_COLUMN - y - FONTSIZE;
  y = temp;
  #endif
  
  #ifdef ROTATED_ACW
  byte temp = y;
  y = NUMBER_OF_PIXELS_PER_ROW - x - FONTWIDTH;
  x = temp;
  #endif
  
  struct symbol s = {(byte)(getNumberOfSymbols() + 1), false, (byte)c, x, y, FONTWIDTH, FONTSIZE, b, NULL, NULL};
  addSymbolTail(&s);
}

/** ===========================================================
 * \fn      drawString
 * \brief   adds multiple char symbols in the symbol list
 *          and draws them on given position on the display
 *
 * \requ    createCharSymbol(), drawSymbols()
 *
 * \param   (uchar[]) constant string (char array)
 *          (byte)    x, y top left coordinate of the string
 *          (byte)    brightness (0... MAX_BRIGHTNESS)
 *          
 * \return  -
 ============================================================== */
void drawString(const unsigned char str[], byte x, byte y, byte b)
{  
  byte ofCnt = 0;  // overflow counter

  /* limit input paramters */
  if (x >= NUMBER_OF_PIXELS_PER_ROW - FONTWIDTH) x = NUMBER_OF_PIXELS_PER_ROW - FONTWIDTH - 1;
  if (y >= NUMBER_OF_PIXELS_PER_COLUMN - FONTSIZE) y = NUMBER_OF_PIXELS_PER_COLUMN - FONTSIZE - 1;
  if (b > MAX_BRIGHTNESS) b = MAX_BRIGHTNESS;
  
  for (unsigned int i = 0; str[i] != '\0'; i++)
  {     
    byte currentX = x + i * (FONTWIDTH - CHAR_SPACING_OFFSET);
    
    /* check if end of row woulde be reached */
    if (currentX + FONTWIDTH > NUMBER_OF_PIXELS_PER_ROW)
    {
      /* limit owerflow counter */
      ofCnt %=  (NUMBER_OF_PIXELS_PER_ROW / (FONTWIDTH - CHAR_SPACING_OFFSET));
      if (ofCnt == 0) y += FONTSIZE;
      
      currentX = ofCnt * (FONTWIDTH - CHAR_SPACING_OFFSET);
      ofCnt++;
    }
    
    //blup
    if (i % 2 == 0)
    {
      
    }
    createCharSymbol((byte)str[i], currentX, y, b);
  }

  drawSymbols();
}

/** ===========================================================
 * \fn      createCustomSymbol
 * \brief   adds the customSymbol in the symbol list or
 *          overwrites existing
 *
 * \requ    addSymbolTail(), replaceSymbol()
 *
 * \param   (byte) x, y top left coordinate of the customSymbol
 *          (byte) brightness (0... MAX_BRIGHTNESS)
 * \return  -
 ============================================================== */
void createCustomSymbol(byte x, byte y, byte b)
{ 
  /* limit input paramters */
  if (x >= NUMBER_OF_PIXELS_PER_ROW - CUSTOM_SYMBOL_WIDTH) x = NUMBER_OF_PIXELS_PER_ROW - CUSTOM_SYMBOL_WIDTH - 1;
  if (y >= NUMBER_OF_PIXELS_PER_COLUMN - CUSTOM_SYMBOL_SIZE) y = NUMBER_OF_PIXELS_PER_COLUMN - CUSTOM_SYMBOL_SIZE - 1;
  if (b > MAX_BRIGHTNESS) b = MAX_BRIGHTNESS;

  /* recalculate symbol position if rotated */
  #ifdef ROTATED_CW
  byte temp = x;
  x = NUMBER_OF_PIXELS_PER_COLUMN - y - CUSTOM_SYMBOL_SIZE;
  y = temp;
  #endif
  
  #ifdef ROTATED_ACW
  byte temp = y;
  y = NUMBER_OF_PIXELS_PER_ROW - x - CUSTOM_SYMBOL_WIDTH;
  x = temp;
  #endif
  
  /* define a customSymbol */
  struct symbol s = {(byte)(getNumberOfSymbols() + 1), false, CUSTOM_SYMBOL_NAME_OFFSET, x, y, CUSTOM_SYMBOL_WIDTH, CUSTOM_SYMBOL_SIZE, b, NULL, NULL};
  
  /* check existing */
  struct symbol *ptr = getCustomSymbol();
  
  if (ptr != NULL) replaceSymbol(ptr, &s);
  else addSymbolTail(&s);
}

/* functions to manipulate the symbols in the symbol list ----- */
/** ===========================================================
 * \fn      addSymbol
 * \brief   adds a symbol in the symbol list and returns its
 *          address
 *
 * \param   (*symbol) pointer on a symbol
 * \return  (*symbol) pointer on new symbol in the symbol list
 ============================================================== */
static struct symbol *addSymbol(struct symbol *s)
{
  /* get position of first empty symbol in the symbol list */
  byte p = getListPosOfFirstEmptySymbol();
  
  /* check if there's some place left in the list */
  if (getNumberOfSymbols() < MAX_NUMBER_OF_SYMBOLS)
  {
    /* add symbol on the first empty position in the symbol list */
    symbolListMemory[p] = *s;
    return &symbolListMemory[p];
  }
  else
  {
    #ifdef DEBUGGING
    Serial.println("CustomSymbol's full! Couldn't add the new symbol");
    #endif

    return NULL;
  }
}

/** ===========================================================
 * \fn      addSymbolHead
 * \brief   adds a symbol at the beginning of the symbol list
 *
 * \param   (*symbol) pointer on a symbol
 * \return  -
 ============================================================== */
static void addSymbolHead(struct symbol *s)
{
  /* add s in the symbol list */
  struct symbol *s_ = addSymbol(s);
  
  putSymbolHead(s_);
}

/** ===========================================================
 * \fn      addSymbolTail
 * \brief   adds a symbol at the end of the symbol list
 *
 * \param   (*symbol) pointer on a symbol
 * \return  -
 ============================================================== */
static void addSymbolTail(struct symbol *s)
{
  /* add s in the symbol list */
  struct symbol *s_ = addSymbol(s);
  
  putSymbolTail(s_);
}

/** ===========================================================
 * \fn      addSymbolInFront
 * \brief   adds a symbol(s1) in front of a symbol (s2) within
 *          the symbol list
 *
 * \param   (*symbol) pointer on a symbol 
 *          (*symbol) pointer on a symbol in the symbol list
 * \return  -
 ============================================================== */
static void addSymbolInFront(struct symbol *s1, struct symbol *s2)
{
  /* add s1 in the symbol list */
  struct symbol *s1_ = addSymbol(s1);
  
  /* check if s2 is head */
  if (s2 == head)
  {
    head = s1_;
    s1_ -> prev = NULL;
  }
  else
  {
    (s2 -> prev) -> next = s1_;
    s1_ -> prev = (s2 -> prev);
  }
  
  s1_ -> next = s2;
  s2 -> prev = s1_;
}

/** ===========================================================
 * \fn      addSymbolBehind
 * \brief   adds a symbol(s1) behind a symbol (s2) within
 *          the symbol list
 *
 * \param   (*symbol) pointer on a symbol 
 *          (*symbol) pointer on a symbol in the symbol list
 * \return  -
 ============================================================== */
static void addSymbolBehind(struct symbol *s1, struct symbol *s2)
{
  /* add s1 in the symbol list */
  struct symbol *s1_ = addSymbol(s1);
    
  /* check if s2 is tail */
  if (s2 == tail)
  {
    tail = s1_;
    s1_ -> next = NULL;
  }
  else
  {
    (s2 -> next) -> prev = s1_;
    s1_ -> next = (s2 -> next);
  }
  
  s1_ -> prev = s2;
  s2 -> next = s1_;
}

/** ===========================================================
 * \fn      popSymbol
 * \brief   takes away a symbol in the symbol list
 *          (use putSymbolHead() or putSymbolTail() to reinsert
 *          taken symbol in the symbol list)
 *
 * \param   (*symbol) pointer on a symbol in the symbol list
 * \return  (*symbol) pointer on taken symbol
 ============================================================== */
static struct symbol *popSymbol(struct symbol *s)
{ 
  /* check if the list is not empty */
  if (head != NULL)
  {
    /* check if s is the only symbol in the list */
    if (head == tail)
    {
      head = NULL;
      tail = NULL;
    }
    else
    {
      /* check if s is the first symbol in the list */
      if (s == head)
      {
        head = s -> next;
        head -> prev = NULL;
      }
            
      /* check if s is the last symbol in the list */
      else if (s == tail)
      {
       tail = s -> prev;
       tail -> next = NULL; 
      }
      else
      {
        (s -> prev) -> next = s -> next;
        (s -> next) -> prev = s -> prev;
      }
    }
  }
  
  return s;
}

/** ===========================================================
 * \fn      popSymbolHead
 * \brief   takes away the symbol at the begining of the list
 *          (use putSymbolHead() or putSymbolTail() to reinsert
 *          taken symbol in the symbol list)
 *
 * \param   -
 * \return  (*symbol) pointer on taken symbol
 ============================================================== */
static struct symbol *popSymbolHead()
{ 
  struct symbol *ptr = NULL;
  
  /* check if the list is not empty */
  if (head != NULL)
  {
    ptr = head;
    head = head -> next;
    
    if (head == NULL) tail = NULL;
    else head -> prev = NULL;
  }
  
  return ptr;
}

/** ===========================================================
 * \fn      popSymbolTail
 * \brief   takes away the symbol at the end of the list
 *          (use putSymbolHead() or putSymbolTail() to reinsert
 *          taken symbol in the symbol list)
 *
 * \param   -
 * \return  (*symbol) pointer on taken symbol
 ============================================================== */
static struct symbol *popSymbolTail()
{   
  struct symbol *ptr = NULL;
  
  /* check if the list is not empty */
  if (tail != NULL)
  {
    ptr = tail;
    tail = tail -> prev;
    
    if (tail == NULL) head = NULL;
    else tail -> next = NULL;
  }
  
  return ptr;
}

/** ===========================================================
 * \fn      putSymbolHead
 * \brief   puts a symbol at the beginning of the symbol list
 *
 * \param   (*symbol) pointer on a symbol in the symbol list
 * \return  -
 ============================================================== */
static void putSymbolHead(struct symbol *s)
{
  /* check if the list is empty */
  if (head == NULL) tail = s;
  else head -> prev = s;
  
  s -> next = head;
  head = s;
  s -> prev = NULL;
}

/** ===========================================================
 * \fn      putSymbolTail
 * \brief   puts a symbol at the end of the symbol list
 *
 * \param   (*symbol) pointer on a symbol in the symbol list
 * \return  -
 ============================================================== */
static void putSymbolTail(struct symbol *s)
{  
  /* check if the list is empty */
  if (tail == NULL) head = s;
  else tail -> next = s;
  
  s -> prev = tail;
  tail = s;
  s -> next = NULL;
}

/** ===========================================================
 * \fn      putSymbolInFront
 * \brief   puts a symbol (s1) in front of another symbol (s2)
 *          in the symbol list
 *
 * \param   (*symbol) pointer on a symbol in the symbol list
 *          (*symbol) pointer on a symbol in the symbol list
 * \return  -
 ============================================================== */
static void putSymbolInFront(struct symbol *s1, struct symbol *s2)
{ 
  /* check if s2 is head */
  if (s2 == head)
  {
    head = s1;
    s1 -> prev = NULL;
  }
  else
  {
    (s2 -> prev) -> next = s1;
    s1 -> prev = (s2 -> prev);
  }
  
  s1 -> next = s2;
  s2 -> prev = s1;
}

/** ===========================================================
 * \fn      putSymbolBehind
 * \brief   puts a symbol (s1) behind another symbol (s2) in
 *          the symbol list
 *
 * \param   (*symbol) pointer on a symbol in the symbol list
 *          (*symbol) pointer on a symbol in the symbol list
 * \return  -
 ============================================================== */
static void putSymbolBehind(struct symbol *s1, struct symbol *s2)
{ 
  /* check if s2 is tail */
  if (s2 == tail)
  {
    tail = s1;
    s1 -> next = NULL;
  }
  else
  {
    (s2 -> next) -> prev = s1;
    s1 -> next = (s2 -> next);
  }
  
  s1 -> prev = s2;
  s2 -> next = s1;
}

/** ===========================================================
 * \fn      moveSymbolAbove
 * \brief   moves a symbol (s1) in the symbol list behind
 *          another symbol (s2) of the symbol list
 *
 * \param   (*symbol) pointer on a symbol in the symbol list
 *          (*symbol) pointer on a symbol in the symbol list
 * \return  -
 ============================================================== */
void moveSymbolAbove(struct symbol *s1, struct symbol *s2)
{ 
  putSymbolBehind(popSymbol(s1), s2);
}

/** ===========================================================
 * \fn      moveSymbolUnder
 * \brief   moves a symbol (s1) in the symbol list in front of
 *          another symbol (s2) of the symbol list
 *
 * \param   (*symbol) pointer on a symbol in the symbol list
 *          (*symbol) pointer on a symbol in the symbol list
 * \return  -
 ============================================================== */
void moveSymbolUnder(struct symbol *s1, struct symbol *s2)
{ 
  putSymbolInFront(popSymbol(s1), s2);
}

/** ===========================================================
 * \fn      replaceSymbol
 * \brief   replaces a symbol (s1) with another symbol (s2) in
 *          the symbol list
 *
 * \param   (*symbol) pointer on a symbol
 *          (*symbol) pointer on a symbol in the symbol list
 * \return  -
 ============================================================== */
static void replaceSymbol(struct symbol *s1, struct symbol *s2)
{
  addSymbolInFront(s2, s1);
  removeSymbol(s1);
}

/** ===========================================================
 * \fn      deleteSymbol
 * \brief   deletes a symbol in the symbol list by writing a 0
 *          to its id
 *
 * \param   (*symbol) pointer on a symbol in the symbol list
 * \return  -
 ============================================================== */
static void deleteSymbol(struct symbol *s)
{
  s -> b = 0;  // clear also its brightness to make shure it will never be shown
  s -> id = 0;
}

/** ===========================================================
 * \fn      removeSymbol
 * \brief   removes a symbol in the symbol list
 *
 * \param   (*symbol) pointer on a symbol in the symbol list
 * \return  -
 ============================================================== */
void removeSymbol(struct symbol *s)
{
  deleteSymbol(s);
  
  /* check if the list is not empty */
  if (head != NULL)
  {
    /* check if s is the only symbol in the list */
    if (head == tail)
    {
      head = NULL;
      tail = NULL;
    }
    else
    {
      /* check if s is the first symbol in the list */
      if (s == head)
      {
        head = s -> next;
        head -> prev = NULL;
      }
            
      /* check if s is the last symbol in the list */
      else if (s == tail)
      {
       tail = s -> prev;
       tail -> next = NULL; 
      }
      else
      {
        (s -> prev) -> next = s -> next;
        (s -> next) -> prev = s -> prev;
      }
    }
  }
}

/** ===========================================================
 * \fn      removeSymbolHead
 * \brief   removes the symbol at the beginning of the list
 *
 * \param   -
 * \return  -
 ============================================================== */
void removeSymbolHead()
{
  deleteSymbol(head);
  
  /* check if the list is not empty */
  if (head != NULL)
  {    
    /* check if head is the only symbol in the list */
    if (head == tail)
    {
      head = NULL;
      tail = NULL;
    }
    else
    {
      head = head -> next;
      head -> prev = NULL;
    }
  }
}

/** ===========================================================
 * \fn      removeSymbolTail
 * \brief   removes the symbol at the end of the list
 *
 * \param   -
 * \return  -
 ============================================================== */
void removeSymbolTail()
{
  deleteSymbol(tail);
  
  /* check if the list is not empty */
  if (tail != NULL)
  {
    /* check if tail ist the only symbol in the list */
    if (tail == head)
    {
      head = NULL;
      tail = NULL;
    }
    else
    {
      tail = tail -> prev;
      tail -> next = NULL;
    }
  }
}

/** ===========================================================
 * \fn      clearSymbolList
 * \brief   deletes all symbols of the symbol list
 *
 * \param   -
 * \return  -
 ============================================================== */
void clearSymbolList()
{
  for (struct symbol *ptr = head; ptr != NULL; ptr = ptr -> next)
  {
    deleteSymbol(ptr);
  }
  
  head = NULL;
  tail = NULL;
}

//blup: todo with next-ptr (if necessary)
///** ===========================================================
// * \fn      bubblesortSymbolsY
// * \brief   bubblesorts the symbols in the symbol list by y
// *
// * \param   (*symbol) pointer on the symbol list
// *          (int)     lengt of the list to sort
// * \return  -
// ============================================================== */
//static void bubblesortSymbolsY(struct symbol s[], int length)
//{
//  int i, j;
//  
//  #ifdef DEBUGGING
//  Serial.println("bubblesort y");
//  #endif
//  
//  for (i = length - 1; i > 0; i--)
//  {
//    for (j = 1; j <= i; j++)
//    {      
//      if (s[j - 1].y > s[j].y)
//      {
//        swapSymbols(&s[j - 1], &s[j]);
//      }
//    }
//  }
//}
//
///** ===========================================================
// * \fn      bubblesortSymbolsX
// * \brief   bubblesorts the symbols in the symbol list by x
// *
// * \param   (*symbol) pointer on the symbol list
// *          (int)     lengt of the list to sort
// * \return  -
// ============================================================== */
//static void bubblesortSymbolsX(struct symbol s[], int length)
//{
//  int i, j;
//  
//  #ifdef DEBUGGING
//  Serial.println("bubblesort x");
//  #endif
//  
//  for (i = length - 1; i > 0; i--)
//  {
//    for (j = 1; j <= i; j++)
//    {
//      if (s[j - 1].x > s[j].x)
//      {
//        swapSymbols(&s[j - 1], &s[j]);
//      }
//    }
//  }
//}

/* functions to get informations of the symbol list ----------- */
/** ===========================================================
 * \fn      getNumberOfSymbols
 * \brief   returns the number of symbols in the symbol list
 *
 * \param   -
 * \return  (byte) number of symbols in the symbol list
 ============================================================== */
byte getNumberOfSymbols()
{
  byte cnt = 0;
  
  for (struct symbol *ptr = head; ptr != NULL; ptr = ptr -> next)
  {
    if ((ptr -> id) > 0) cnt++;
  }
  
  return cnt;
}

/** ===========================================================
 * \fn      getListPosOfFirstEmptySymbol
 * \brief   returns the number of the position of the first
 *          empty symbol in the symbol list memory
 *
 * \param   -
 * \return  (byte) position of first empty symbol
 ============================================================== */
static byte getListPosOfFirstEmptySymbol()
{
  byte pos = 0;
  
  for (byte i = 0; i < sizeof(symbolListMemory) / sizeof(symbol); i++)
  {
    if (symbolListMemory[i].id == 0) break;
    else pos++;
  }
  
  return pos;
}

///** ===========================================================
// * \fn      getListPosOfSymbol
// * \brief   returns the number of the position of the given
// *          symbol in the symbol list memory
// *
// * \param   -
// * \return  (byte) position of first empty symbol
// ============================================================== */
// static byte getListPosOfSymbol(struct symbol *s)
// {
//   byte pos = 0;
  
//   for (byte i = 0; i < sizeof(symbolListMemory) / sizeof(symbol); i++)
//   {
//     if (symbolListMemory[i].id == (s -> id)) break;
//     else pos++;
//   }
  
//   return pos;
// }

/** ===========================================================
 * \fn      getCustomSymbol
 * \brief   returns a pointer on the customSymbol in the
 *          symbol list or NULL if there is no customSymbol
 *
 * \param   -
 * \return  (*symbol) symbol pointer on customSymbol or NULL
 ============================================================== */
struct symbol *getCustomSymbol()
{
  struct symbol *res = NULL;
  
  for (struct symbol *ptr = head; ptr != NULL; ptr = ptr -> next)
  {
    if ((ptr -> c) == CUSTOM_SYMBOL_NAME_OFFSET)
    {
      res = ptr;
      break;
    }
  }
  
  return res;   
}

/** ===========================================================
 * \fn      showSymbolList
 * \brief   serial-prints the linked symbol list
 *
 * \param   -
 * \return  -
 ============================================================== */
 #ifdef DEBUGGING
void showSymbolList()
{
  struct symbol *ptr = head;
  
  if (head != NULL)
  {
    Serial.print("                                                                          head id = ");
    Serial.println(head -> id);
      
    for (byte l = 0; ptr != NULL; l++)
    {
      Serial.print(l);
      if (l >= 10) Serial.print(":");
      else Serial.print(": ");
      Serial.print(" id = ");
      Serial.print(ptr -> id);
      if (ptr -> id >= 100) Serial.print(" c = ");
      else if (ptr -> id >= 10) Serial.print("  c = ");
      else Serial.print("   c = ");
      Serial.print(ptr -> c);
      Serial.print("  x = ");
      Serial.print(ptr -> x);
      if (ptr -> x >= 100) Serial.print(" y = ");
      else if (ptr -> x >= 10) Serial.print("  y = ");
      else Serial.print("   y = ");
      Serial.print(ptr -> y);
      if (ptr -> y >= 100) Serial.print(" w = ");
      else if (ptr -> y >= 10) Serial.print("  w = ");
      else Serial.print("   w = ");
      Serial.print(ptr -> w);
      if (ptr -> w >= 100) Serial.print(" h = ");
      else if (ptr -> w >= 10) Serial.print("  h = ");
      else Serial.print("   h = ");
      Serial.print(ptr -> h);
      if (ptr -> h >= 100) Serial.print(" b = ");
      else if (ptr -> h >= 10) Serial.print("  b = ");
      else Serial.print("   b = ");
      Serial.print(ptr -> b);
      if (ptr -> b >= 10) Serial.print(" prev id = ");
      else Serial.print("  prev id = ");
      Serial.print((ptr -> prev) -> id);
      Serial.print("  next id = ");
      Serial.println((ptr -> next) -> id);
      
      ptr = ptr -> next;
    }
    
    Serial.print("                                                           tail id = ");
    Serial.println(tail -> id);
    Serial.flush();
  }
  else Serial.println("Symbol list is empty!");
}
#endif

/* functions to manipulate symbol parameters ------------------ */
/** ===========================================================
 * \fn      moveSymbol
 * \brief   moves a symbol in the list on the display
 *
 * \param   (*symbol) pointer on a symbol in the symbol list
 *          (int) signed value for x and y
 * \return  -
 ============================================================== */
void moveSymbol(struct symbol *s, int x, int y)
{
  /* recalculate symbol position if rotated */
  #ifdef ROTATED_CW
  int temp = x;
  x = -y;//NUMBER_OF_PIXELS_PER_COLUMN - y - (s -> h);
  y = temp;
  #endif
  
  #ifdef ROTATED_ACW
  int temp = y;
  y = -x;//NUMBER_OF_PIXELS_PER_ROW - x - (s -> w);
  x = temp;
  #endif
  
  s -> x += x;
  s -> y += y;
  
  s -> dr = false;
}

/** ===========================================================
 * \fn      swapSymbols
 * \brief   swaps the content of two symbols
 *
 * \param   (*symbol) pointer on two symbols
 * \return  -
 ============================================================== */
void swapSymbols(struct symbol *s1, struct symbol *s2)
{
  struct symbol tmp = *s2;
  *s2 = *s1;
  *s1 = tmp;
}

/** ===========================================================
 * \fn      swapSymbolPositions
 * \brief   swaps the position of two symbols in the symbol
 *          list
 *
 * \param   (*symbol) pointer on two symbols in the symbol list
 * \return  -
 ============================================================== */
void swapSymbolPositions(struct symbol *s1, struct symbol *s2)
{
  byte tmpX = s2 -> x;
  byte tmpY = s2 -> y;
  s2 -> x = s1 -> x;
  s2 -> y = s1 -> y;
  s1 -> x = tmpX;
  s1 -> y = tmpY;
}

/* symbol parameter set-function */
void setSymbolID(symbol *s, byte id) {s -> id = id; s -> dr = false;}
void setSymbolDrawed(symbol *s, boolean dr) {s -> dr = dr;}
void setSymbolName(symbol *s, byte c) {s -> c = c; s -> dr = false;}
void setSymbolPosX(symbol *s, byte x) {s -> x = x; s -> dr = false;}
void setSymbolPosY(symbol *s, byte y) {s -> y = y; s -> dr = false;}
void setSymbolWidth(symbol *s, byte w) {s -> w = w; s -> dr = false;}
void setSymbolHeight(symbol *s, byte h) {s -> h = h; s -> dr = false;}
void setSymbolBrightness(symbol *s, byte b) {s -> b = b; s -> dr = false;}

/* symbol parameter get-function */
byte getSymbolID(symbol *s) {return s -> id;}
boolean getSymbolDrawed(symbol *s) {return s -> dr;}
byte getSymbolName(symbol *s) {return s -> c;}
byte getSymbolPosX(symbol *s) {return s -> x;}
byte getSymbolPosY(symbol *s) {return s -> y;}
byte getSymbolWidth(symbol *s) {return s -> w;}
byte getSymbolHeight(symbol *s) {return s -> h;}
byte getSymbolBrigthness(symbol *s) {return s -> b;}

/** ===========================================================
 * \fn      setSymbolBrightnesses
 * \brief   changes the brightness of all symbols in the symbol
 *          list 
 *
 * \param   (byte) brightness (0... MAX_BRIGHTNESS)
 * \return  -
 ============================================================== */
void setSymbolBrightnesses(byte b)
{
  /* limit input paramters */
  if (b > MAX_BRIGHTNESS) b = MAX_BRIGHTNESS;

  for (struct symbol *ptr = head; ptr != NULL; ptr = ptr -> next)
  {
    setSymbolBrightness(ptr, b);
  }
}

/* functions to handle the customSymbol */
/** ===========================================================
 * \fn      fillCustomSymbol
 * \brief   fills the whole customSymbol content
 *
 * \param   -
 * \return  -
 ============================================================== */
void fillCustomSymbol()
{  
  for (int y = 0 ; y < CUSTOM_SYMBOL_SIZE; y++)
  {
    for (int x = 0 ; x < CUSTOM_SYMBOL_WIDTH / NUMBER_OF_PIXELS_PER_BYTE; x++)
    { 
      customSymbol[y][x] = 0xFF;
    }
  }
}

/** ===========================================================
 * \fn      clearCustomSymbol
 * \brief   clears the whole customSymbol content
 *
 * \param   -
 * \return  -
 ============================================================== */
void clearCustomSymbol()
{  
  for (int y = 0 ; y < CUSTOM_SYMBOL_SIZE; y++)
  {
    for (int x = 0 ; x < CUSTOM_SYMBOL_WIDTH / NUMBER_OF_PIXELS_PER_BYTE; x++)
    { 
      customSymbol[y][x] = 0;
    }
  }
}

/** ===========================================================
 * \fn      addPixel
 * \brief   sets a single monochrome pixel in the customSymbol
 *
 * \param   (byte) x coordinate of the pixel within the customSymbol
 *                 (0... CUSTOM_SYMBOL_WIDTH - 1)
 *          (byte) y coordinate of the pixel within the customSymbol
 *                 (0... CUSTOM_SYMBOL_SIZE - 1)
 * \return  -
 ============================================================== */
void addPixel(byte x, byte y)
{
  /* limit input paramters */
  if (x >= CUSTOM_SYMBOL_WIDTH) x = CUSTOM_SYMBOL_WIDTH - 1;
  if (y >= CUSTOM_SYMBOL_SIZE) y = CUSTOM_SYMBOL_SIZE - 1;
  
  /* recalculate position if rotated */
  #ifdef ROTATED_CW
  byte temp = x;
  x = CUSTOM_SYMBOL_SIZE - 1 - y;
  y = temp;
  #endif
  
  #ifdef ROTATED_ACW
  byte temp = y;
  y = CUSTOM_SYMBOL_WIDTH - 1 - x;
  x = temp;
  #endif

  /* calculate number of required shifts on base of pixel location and number of pixels per byte */
  byte numberOfRequiredShifts = 8 - NUMBER_OF_BITS_PER_PIXEL * ((x % NUMBER_OF_PIXELS_PER_BYTE) + 1);  // 8 - 1*((0,1,2,3,4,5,6,7)+1)

  /* set bit of matching customSymbol byte by shifting */
  customSymbol[y][x / NUMBER_OF_PIXELS_PER_BYTE] |= (1 << numberOfRequiredShifts);
}

/** ===========================================================
 * \fn      clearPixel
 * \brief   resets a single monochrome pixel in the customSymbol
 *
 * \param   (byte) x coordinate of the pixel within the customSymbol
 *                 (0... CUSTOM_SYMBOL_WIDTH - 1)
 *          (byte) y coordinate of the pixel within the customSymbol
 *                 (0... CUSTOM_SYMBOL_SIZE - 1)
 * \return  -
 ============================================================== */
void clearPixel(byte x, byte y)
{
  /* limit input paramters */
  if (x >= CUSTOM_SYMBOL_WIDTH) x = CUSTOM_SYMBOL_WIDTH - 1;
  if (y >= CUSTOM_SYMBOL_SIZE) y = CUSTOM_SYMBOL_SIZE - 1;
  
  /* recalculate position if rotated */
  #ifdef ROTATED_CW
  byte temp = x;
  x = CUSTOM_SYMBOL_SIZE - 1 - y;
  y = temp;
  #endif
  
  #ifdef ROTATED_ACW
  byte temp = y;
  y = CUSTOM_SYMBOL_WIDTH - 1 - x;
  x = temp;
  #endif
  
  /* calculate number of required shifts on base of pixel location and number of pixels per byte */
  byte numberOfRequiredShifts = 8 - NUMBER_OF_BITS_PER_PIXEL * ((x % NUMBER_OF_PIXELS_PER_BYTE) + 1);  // 8 - 1*((0,1,2,3,4,5,6,7)+1)

  /* set bit of matching customSymbol byte by shifting */
  customSymbol[y][x / NUMBER_OF_PIXELS_PER_BYTE] &= ~(1 << numberOfRequiredShifts);
}

/** ===========================================================
 * \fn      getPixel
 * \brief   gets the state of a single pixel from the customSymbol
 *
 * \param   (byte) x coordinate of the pixel
 *                 (0... CUSTOM_SYMBOL_WIDTH - 1)
 *          (byte) y coordinate of the pixel
 *                 (0... CUSTOM_SYMBOL_SIZE - 1) 
 * \return  (bool) true = 1 (set), false = 0 (reset)
 ============================================================== */
boolean getPixel(byte x, byte y)
{
  /* limit input paramters */
  if (x >= CUSTOM_SYMBOL_WIDTH) x = CUSTOM_SYMBOL_WIDTH - 1;
  if (y >= CUSTOM_SYMBOL_SIZE) y = CUSTOM_SYMBOL_SIZE - 1;
  
  /* recalculate position if rotated */
  #ifdef ROTATED_CW
  byte temp = x;
  x = CUSTOM_SYMBOL_SIZE - 1 - y;
  y = temp;
  #endif
  
  #ifdef ROTATED_ACW
  byte temp = y;
  y = CUSTOM_SYMBOL_WIDTH - 1 - x;
  x = temp;
  #endif

  boolean b = false;

  /* define number of required shifts on base of pixel location */
  byte numberOfRequiredShifts = 8 - NUMBER_OF_BITS_PER_PIXEL * ((x % NUMBER_OF_PIXELS_PER_BYTE) + 1);  // 8 - 1*((0,1,2,3,4,5,6,7)+1)

  /* get pixel brightness of matching customSymbol byte */
  b |= customSymbol[y][x / NUMBER_OF_PIXELS_PER_BYTE] >> numberOfRequiredShifts;
  b &= 0xFF >> (8 - NUMBER_OF_BITS_PER_PIXEL);  // mask the brightness value

  return b;
}

/** ===========================================================
 * \fn      rotateCustomSymbolContentCW
 * \brief   rotates the customSymbol content by 90° clockwise
 *          (customSymbol have to be symmetrical!)
 *          (f(x, y) = (max-y, x))
 *
 * \param   -
 * \return  -
 ============================================================== */
void rotateCustomSymbolContentCW()
{
  #ifdef DEBUGGING
  Serial.println("rotate customSymbol content CW");
  #endif

  boolean pixelA, pixelB;
  byte currentX, currentY, newX, newY;
  const byte maxXY = (CUSTOM_SYMBOL_SIZE - 1);

  for (int y = 0 ; y < CUSTOM_SYMBOL_SIZE / 2; y++)
  {
    for (int x = 0 ; x < CUSTOM_SYMBOL_WIDTH / 2; x++)
    { 
      pixelA = getPixel(x, y);

      currentX = maxXY - y;
      currentY = x;

      pixelB = getPixel(currentX, currentY);
      if (pixelA) addPixel(currentX, currentY);
      else clearPixel(currentX, currentY);

      newX = maxXY - currentY;
      newY = currentX;
      currentX = newX;
      currentY = newY;

      pixelA = getPixel(currentX, currentY);
      if (pixelB) addPixel(currentX, currentY);
      else clearPixel(currentX, currentY);

      newX = maxXY - currentY;
      newY = currentX;
      currentX = newX;
      currentY = newY;

      pixelB = getPixel(currentX, currentY);
      if (pixelA) addPixel(currentX, currentY);
      else clearPixel(currentX, currentY);

      newX = maxXY - currentY;
      newY = currentX;
      currentX = newX;
      currentY = newY;

      pixelA = getPixel(currentX, currentY);
      if (pixelB) addPixel(currentX, currentY);
      else clearPixel(currentX, currentY);
    }  
  }
}

/** ===========================================================
 * \fn      rotateCustomSymbolContentACW
 * \brief   rotates the customSymbol content by 90° against clockwise
 *          (customSymbol have to be symmetrical!)
 *          (f(x, y) = (y, max-x))
 *
 * \param   -
 * \return  -
 ============================================================== */
void rotateCustomSymbolContentACW()
{
  #ifdef DEBUGGING
  Serial.println("rotate customSymbol content ACW");
  #endif

  boolean pixelA, pixelB;
  byte currentX, currentY, newX, newY;
  const byte maxXY = (CUSTOM_SYMBOL_SIZE - 1);

  for (int y = 0 ; y < CUSTOM_SYMBOL_SIZE / 2; y++)
  {
    for (int x = 0 ; x < CUSTOM_SYMBOL_WIDTH / 2; x++)
    { 
      pixelA = getPixel(x, y);

      currentX = y;
      currentY = maxXY - x;

      pixelB = getPixel(currentX, currentY);
      if (pixelA) addPixel(currentX, currentY);
      else clearPixel(currentX, currentY);

      newX = currentY;
      newY = maxXY - currentX;
      currentX = newX;
      currentY = newY;

      pixelA = getPixel(currentX, currentY);
      if (pixelB) addPixel(currentX, currentY);
      else clearPixel(currentX, currentY);

      newX = currentY;
      newY = maxXY - currentX;
      currentX = newX;
      currentY = newY;

      pixelB = getPixel(currentX, currentY);
      if (pixelA) addPixel(currentX, currentY);
      else clearPixel(currentX, currentY);

      newX = currentY;
      newY = maxXY - currentX;
      currentX = newX;
      currentY = newY;

      pixelA = getPixel(currentX, currentY);
      if (pixelB) addPixel(currentX, currentY);
      else clearPixel(currentX, currentY);
    }  
  }
}

/** ===========================================================
 * \fn      addLine
 * \brief   writes a line on given coordinates into the customSymbol
 *          (uses the Bresenham's line-drawing algorithm)
 *
 * \param   (int) x0, y0 start coordinate of the line
 *          (int) x1, y1 end coordinate of the line
 * \return  -
 ============================================================== */
void addLine(int x0, int y0, int x1, int y1)
{ 
  int dx = x1 - x0;
  int dy = y1 - y0;
  int dx_abs = abs(dx);
  int dy_abs = abs(dy);
  int s_dx = sgn(dx);
  int s_dy = sgn(dy);
  int px = x0;
  int py = y0;
  int x, y;

  /* limit input paramters */
  if (dx_abs >= CUSTOM_SYMBOL_WIDTH) dx_abs = CUSTOM_SYMBOL_WIDTH - 1;
  if (dy_abs >= CUSTOM_SYMBOL_SIZE) dy_abs = CUSTOM_SYMBOL_SIZE - 1;

  x = dy_abs >> 1;  // dy_abs/2
  y = dx_abs >> 1;  // dx_abs/2

  addPixel(x0, y0); // set first pixel

  if (dx_abs >= dy_abs) // the line is more horizontal than vertical
  {
    for(int i = 0; i < dx_abs; i++)
    {
      y += dy_abs;
      if (y >= dx_abs)
      {
        y -= dx_abs;
        py += s_dy;
      }
      px += s_dx;
      addPixel(px, py);
    }
  }
  else // the line is more vertical than horizontal
  {
    for(int i = 0; i < dy_abs; i++)
    {
      x += dx_abs;
      if (x >= dy_abs)
      {
        x -= dy_abs;
        px += s_dx;
      }
      py += s_dy;
      addPixel(px, py);
    }
  }
}

/** ===========================================================
 * \fn      sgn
 * \brief   signum function
 *
 * \param   (int) value
 * \return  (int) 1 if value > 0
 *                0 if value = 0
 *               -1 if value < 0
 ============================================================== */
static int sgn(int a)
{
  if (a > 0) return 1;
  if (a < 0) return -1;
  return 0;
}

/** ===========================================================
 * \fn      addHLine
 * \brief   writes a horizontal line on given coordinate into
 *          the customSymbol
 *
 * \requ    addLine()
 *
 * \param   (byte) x, y start coordinate of the line
 *          (byte) length of the line
 * \return  -
 ============================================================== */
void addHLine(byte x, byte y, byte l)
{
  addLine(x, y, x + l - 1, y);
}

/** ===========================================================
 * \fn      addVLine
 * \brief   writes a vertical line on given coordinate into the
 *          customSymbol
 *
 * \requ    addLine()
 *
 * \param   (byte) x, y start coordinate of the line
 *          (byte) height of the line
 * \return  -
 ============================================================== */
void addVLine(byte x, byte y, byte h)
{
  addLine(x, y, x, y + h - 1);
}

/** ===========================================================
 * \fn      addRectangle
 * \brief   draws a rectangle on given coordinate into the
 *          customSymbol
 *
 * \requ    addHLine(), addVLine(), drawCustomSymbol()
 *
 * \param   (byte) x, y top left coordinate of the rectangle
 *          (byte) width of the rectangle
 *          (byte) height of the rectangle
 * \return  -
 ============================================================== */
void addRectangle(byte x, byte y, byte w, byte h)
{
  addHLine(x, y, w);
  addVLine(x, y, h);
  addHLine(x, y + h - 1, w);
  addVLine(x + w - 1, y, h);
  
  drawCustomSymbol();
}

/** ===========================================================
 * \fn      addFilledRectangle
 * \brief   draws a filled rectangle on given coordinates into
 *          the customSymbol
 *
 * \requ    addHLine(), drawCustomSymbol();
 *
 * \param   (byte) x, y top left coordinate of the rectangle
 *          (byte) width of the rectangle
 *          (byte) height of the rectangle
 * \return  -
 ============================================================== */
void addFilledRectangle(byte x, byte y, byte w, byte h)
{
  for (byte i = y; i < y + h; i++)
  {
    addHLine(x, i, w);
  }
  
  drawCustomSymbol();
}

/** ===========================================================
 * \fn      addCircle
 * \brief   draws a circle on given coordinate with given
 *          radius into the customSymbol
 *
 * \requ    addPixel(), drawCustomSymbol();
 *
 * \param   (byte) x0, y0 origo of the circle
 *          (byte) radius of the circle
 * \return  -
 ============================================================== */
void addCircle(byte x0, byte y0, byte r)
{
  int f = 1 - r;
  int ddF_x = 1;
  int ddF_y = -2 * r;
  int x = 0;
  int y = r;

  addPixel(x0, y0+r);
  addPixel(x0, y0-r);
  addPixel(x0+r, y0);
  addPixel(x0-r, y0);

  while (x < y)
  {
    if (f >= 0)
    {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    addPixel(x0 + x, y0 + y);
    addPixel(x0 - x, y0 + y);
    addPixel(x0 + x, y0 - y);
    addPixel(x0 - x, y0 - y);
    addPixel(x0 + y, y0 + x);
    addPixel(x0 - y, y0 + x);
    addPixel(x0 + y, y0 - x);
    addPixel(x0 - y, y0 - x);
  }
  
  drawCustomSymbol();
}

/** ===========================================================
 * \fn      addFilledCircle
 * \brief   draws a filled circle on given coordinate with
 *          given radius into the customSymbol
 *
 * \requ    addHLine(), addLine(), drawCustomSymbol();
 *
 * \param   (byte) x0, y0 origo of the circle
 *          (byte) radius of the circle
 * \return  -
 ============================================================== */
void addFilledCircle(byte x0, byte y0, byte r)
{
  int f = 1 - r;      //-9
  int ddF_x = 1;      //1
  int ddF_y = -2 * r; //-20
  int x = 0;          //0
  int y = r;          //10

  addHLine(x0 - r, y0, 2 * r + 1);

  while (x < y)
  {
    if (f >= 0)
    {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    addLine(x0 + x, y0 + y, x0 - x, y0 + y);
    addLine(x0 + x, y0 - y, x0 - x, y0 - y);
    addLine(x0 + y, y0 + x, x0 - y, y0 + x);
    addLine(x0 + y, y0 - x, x0 - y, y0 - x);
  }
  
  drawCustomSymbol();
}

/** ===========================================================
 * \fn      addTriangle
 * \brief   draws a triangle on given coordinates into the
 *          customSymbol
 *
 * \requ    addLine(), drawCustomSymbol();
 *
 * \param   (byte) x0-x2, y0-y2 coordinates of the triangle
 *                corners
 * \return  -
 ============================================================== */
void addTriangle(byte x0, byte y0, byte x1, byte y1, byte x2, byte y2)
{
  addLine(x0, y0, x1, y1);
  addLine(x0, y0, x2, y2);
  addLine(x1, y1, x2, y2);
  
  drawCustomSymbol();
}

/** ===========================================================
 * \fn      addFilledTriangle
 * \brief   draws a filled triangle on given coordinates into
 *          the customSymbol
 *          (uses the Bresenham's line-drawing algorithm)
 *
 * \requ    addLine(), drawCustomSymbol();
 *
 * \param   (byte) x0-x2, y0-y2 coordinates of the triangle
 *                corners
 * \return  -
 ============================================================== */
void addFilledTriangle(byte x0, byte y0, byte x1, byte y1, byte x2, byte y2)
{
  /* do the same as in the addLine() function but draw lines to
   the static corner x2,y2 instaed of single pixels */
  int dx = x1 - x0;
  int dy = y1 - y0;
  int dx_abs = abs(dx);
  int dy_abs = abs(dy);
  int s_dx = sgn(dx);
  int s_dy = sgn(dy);
  int x = dy_abs >> 1;
  int y = dx_abs >> 1;
  int px = x0;
  int py = y0;

  if (dx_abs >= dy_abs) // the line is more horizontal than vertical
  {
    for(int i = 0; i < dx_abs; i++)
    {
      y += dy_abs;
      if (y >= dx_abs)
      {
        y -= dx_abs;
        py += s_dy;
      }
      px += s_dx;
      addLine(x2, y2, px, py);
    }
  }
  else // the line is more vertical than horizontal
  {
    for(int i = 0; i < dy_abs; i++)
    {
      x += dx_abs;
      if (x >= dy_abs)
      {
        x -= dy_abs;
        px += s_dx;
      }
      py += s_dy;
      addLine(x2, y2, px, py);
    }
  }

  /* do the same again, but with x0,y0 as the static corner to get rid of some empty pixels */
  dx = x2 - x1;
  dy = y2 - y1;
  dx_abs = abs(dx);
  dy_abs = abs(dy);
  s_dx = sgn(dx);
  s_dy = sgn(dy);
  x = dy_abs >> 1;
  y = dx_abs >> 1;
  px = x1;
  py = y1;

  if (dx_abs >= dy_abs) // the line is more horizontal than vertical
  {
    for(int i = 0; i < dx_abs; i++)
    {
      y += dy_abs;
      if (y >= dx_abs)
      {
        y -= dx_abs;
        py += s_dy;
      }
      px += s_dx;
      addLine(x0, y0, px, py);
    }
  }
  else // the line is more vertical than horizontal
  {
    for(int i = 0; i < dy_abs; i++)
    {
      x += dx_abs;
      if (x >= dy_abs)
      {
        x -= dy_abs;
        px += s_dx;
      }
      py += s_dy;
      addLine(x0, y0, px, py);
    }
  } 
  
  drawCustomSymbol();
}

/** ===========================================================
 * \fn      addRandomPattern
 * \brief   draws an ellipse with small parameterson or a
 *          random pattern on base of int-overflows
 *
 * \requ    addPixel(), drawCustomSymbol();
 *
 * \param   (byte) x, y origo
 *          (byte) xr, yr radii
 * \return  -
 ============================================================== */
void addRandomPattern(byte x0, byte y0, byte xr, byte yr)
{
  for(int y =- yr; y <= yr; y++)
  {
    for(int x =- xr; x <= xr; x++)
    {
      if(x*x*yr*yr + y*y*xr*xr <= yr*yr*xr*xr) addPixel(x0 + x, y0 + y);
    }
  }
  
  drawCustomSymbol();
}

/** ===========================================================
 * \fn      showCustomSymbol
 * \brief   serial-prints the customSymbol
 *
 * \param   -
 * \return  -
 ============================================================== */
#ifdef DEBUGGING
void showCustomSymbol()
{
  for (byte y = 0; y < CUSTOM_SYMBOL_SIZE; y++)
  {
    Serial.println();
    for (byte x = 0; x < CUSTOM_SYMBOL_WIDTH / NUMBER_OF_PIXELS_PER_BYTE; x++)
    {      
      for (byte mask = 0b10000000; mask > 0; mask = mask >> 1)
      {
        if ((customSymbol[y][x] & mask) > 0) Serial.print(1);
        else Serial.print(0);
      }
    }
  }
  Serial.println();
}
#endif

/* functions to handle the display outputs */
/** ===========================================================
 * \fn      drawSymbol
 * \brief   shows given symbol on the display
 *
 * \param   -
 * \return  -
 ============================================================== */
void drawSymbol(struct symbol *symbolPtr)
{
  byte interestingByte = 0;
  byte cy, cx, mask;
  boolean firstPixelFlag = true;
  byte data = 0;
  
  /* check if symbols has not be drawn yet */
  if (!(symbolPtr -> dr))
  {
    /* run through every row within a symbol */
    for (cy = 0; cy < (symbolPtr -> h); cy++)
    {
      /* set current byte address */
      setDisplayAddress((symbolPtr -> x) / 2, (symbolPtr -> y) + cy);
      
      /* run through every byte within a symbol */
      for (cx = 0; cx < (symbolPtr -> w) / NUMBER_OF_PIXELS_PER_BYTE; cx++)
      {        
        /* check type of symbol and get current interesting byte */
        if ((symbolPtr -> c) < ASCII_FIRST_CHARACTER_OFFSET)
        {
          /* 32 bit ASCII number */
          interestingByte = pgm_read_byte(&(numbers[(symbolPtr -> c)][cy][cx]));
        }
        else if ((symbolPtr -> c) < PICTURE_NAME_OFFSET)
        {
          /* 16 bit ASCII character */
          interestingByte = pgm_read_byte(&(characters[(symbolPtr -> c) - ASCII_FIRST_CHARACTER_OFFSET][cy][cx]));
        }
        else if ((symbolPtr -> c) == CUSTOM_SYMBOL_NAME_OFFSET)
        {
          /* customSymbol */
          interestingByte = customSymbol[cy][cx];
        }
        else
        {
          /* picture */
          interestingByte = pgm_read_byte(&(pictures[(symbolPtr -> c)][cy][cx]));
        }

        /* decompress interesting byte and send data bytes */
        for (mask = (1 << (NUMBER_OF_PIXELS_PER_BYTE - 1)); mask > 0; mask = mask >> NUMBER_OF_BITS_PER_PIXEL)
        //for (mask = 0b10000000; mask > 0; mask = mask >> 1)
        { 
          /* send a byte every second pixel (defined by the display controller (SSD1329)) */
          if (!firstPixelFlag)  // ...second pixel
          {
            /* check current bit */
            if((interestingByte & mask) > 0)
            {
              /* combine both nibbles */
              data |= (symbolPtr -> b);
            }
            
            /* send data byte */
            writeData(data);
            
            /* reset data byte */
            data = 0;
          }
          else  // ...first pixel
          {
            /* check current bit */
            if((interestingByte & mask) > 0)
            {
              /* cache first nibble */
              data = (symbolPtr -> b) << 4;
            }
          }
          
          /* toggle first pixel flag */
          firstPixelFlag = !firstPixelFlag;
        } // ...end of mask-loop
        
        /* reset interesting byte */
        interestingByte = 0;
        
      } // ...end of cx-loop
    } // ...end of cy-loop
    
//    delay(DRAW_SYMBOL_DELAY);

    /* symbol has now been drawn */
    (symbolPtr -> dr) = true;
  }
}

/** ===========================================================
 * \fn      drawSymbols
 * \brief   shows all symbols of the symbol list on the display
 *
 * \requ    drawSymbol()
 *
 * \param   -
 * \return  -
 ============================================================== */
void drawSymbols()
{  
  /* run through whole linked symbol list */
  for (struct symbol *symbolPtr = head; symbolPtr != NULL; symbolPtr = symbolPtr -> next)
  { 
    drawSymbol(symbolPtr);
  }
}

/** ===========================================================
 * \fn      drawCustomSymbol
 * \brief   shows the customSymbol on the display if existing
 *
 * \requ    drawSymbol(), getCustomSymbol()
 *
 * \param   -
 * \return  -
 ============================================================== */
void drawCustomSymbol()
{
  if (getCustomSymbol() != NULL) drawSymbol(getCustomSymbol());
}

/** ===========================================================
 * \fn      redrawSymbols
 * \brief   redraws all symbols of the symbol list on the
 *          display
 *
 * \requ    drawSymbol()
 *
 * \param   -
 * \return  -
 ============================================================== */
void redrawSymbols()
{
  /* run through whole linked symbol list */
  for (struct symbol *symbolPtr = head; symbolPtr != NULL; symbolPtr = symbolPtr -> next)
  { 
    /* reset drawed boolean */
    symbolPtr -> dr = false;
  }
  
  drawSymbols();
}

/** ===========================================================
 * \fn      fillDisplay
 * \brief   fills the display
 *
 * \param   -
 * \return  -
 ============================================================== */
void fillDisplay()
{
  #ifdef DEBUGGING
  Serial.println("fill display");
  #endif

  setDisplayAddress(0, 0);

  for (int y = 0; y < NUMBER_OF_PIXELS_PER_COLUMN; y++)
  {
    for (int x = 0; x < NUMBER_OF_PIXELS_PER_ROW / 2; x++)
    {
      writeData(0xFF);
    }
  }
}

/** ===========================================================
 * \fn      clearDisplay
 * \brief   clears the display
 *
 * \param   -
 * \return  -
 ============================================================== */
void clearDisplay()
{
  #ifdef DEBUGGING
  Serial.println("clear display");
  #endif
  
  setDisplayAddress(0, 0);

  for (int y = 0; y < NUMBER_OF_PIXELS_PER_COLUMN; y++)
  {
    for (int x = 0; x < NUMBER_OF_PIXELS_PER_ROW / 2; x++)
    {
      writeData(0);
    }
  }
}

/* functions to handle the display ---------------------------- */
/** ===========================================================
 * \fn      initDisplay
 * \brief   initializes the OLED display by writing some
 *          commands to the display controller
 *
 * \param   -
 * \return  -
 ============================================================== */
static void initDisplay()
{
  #ifdef DEBUGGING
  Serial.println("start init");
  #endif
  
  //Startup sequence
  displayStartup();
    
  //DISPLAY OFF
  displayOff();

  //Stop scroll
  stopScroll();

  //Set Display Address
  setDisplayAddress(0, 0);

  //Set Contrast Current
  writeCommand(0x81);
  writeCommand(CONTRAST);    // 0... 255 (127 = default)

  //Set second pre-charge speed
  writeCommand(0x82);
  writeCommand(0xFE);    // 255 (doubling disabled)

  //  //Set Master Icon Control
  //  writeCommand(0x90);
  //  writeCommand(0x20);    // enable only V_CION charge pump circuit

  //Set Re_map
  writeCommand(0xA0);
  writeCommand(0x51);    // column re-map, no nibble re-map, horizontal address increment, COM re-map, COM split odd even
//  writeCommand(0x45);    // column re-map, no nibble re-map, vertical address increment, COM re-map, COM split odd even

  //Set Display Start Line
  writeCommand(0xA1);
  writeCommand(0x00);    // 0... 127

  //Set Display Offset
  writeCommand(0xA2);
  writeCommand(0x00);    // 0... 127

  //Set Display Mode
  writeCommand(0xA4);  // normal mode (POR)
//  writeCommand(0xA5);  // entire display on
//  writeCommand(0xA6);  // entire display off
//  writeCommand(0xA7);  // entire display inverted

  //Set MUX Radio
  writeCommand(0xA8);
  writeCommand(0x7F);    // 127 (max)

  //Set first pre_charge phase length
  writeCommand(0xB1);
  writeCommand(0x53);    // default

  //Set Frame Frequency (DCLKs per ROW)
  writeCommand(0xB2);
  writeCommand(0x23);    // (0x14... 4E)

  //Set Front Clock Divider/Oscillator Frequency
  writeCommand(0xB3);
  writeCommand(0x51);    // 5 -> (500kHz / 5) and 1 -> divided by 2

  //Set Default Gray Scale table
  writeCommand(0xB7);

  //Set Second Pre_Charge Period
  writeCommand(0xBB);
  writeCommand(0x07);    // 7 DCLKs

  //Set first pre_charge voltage
  writeCommand(0xBC);
  writeCommand(0x1F);    // 0.63 * VCC

  //Set VCOMH
  writeCommand(0xBE);
  writeCommand(0x1F);    // 0.63 * VCC

  //Set Command Lock
  writeCommand(0xFD);
  writeCommand(0x12);    // disable locking the MCU from entering commands (0x16 = locked)

  //DISPLAY ON
  displayOn();
}

/** ===========================================================
 * \fn      displayStartup
 * \brief   start up sequence of the display
 ============================================================== */
static void displayStartup()
{
  delay(100);  // VDD power up delay
  
  digitalWrite(RST, LOW);
  delay(1);
  digitalWrite(RST, HIGH);
  delay(1);
  
  digitalWrite(EN_BOOST, HIGH); 
  delay(100);  // VCC power up delay
}

///** ===========================================================
// * \fn      displayShutdown
// * \brief   shut down sequence of the display
// ============================================================== */
//static void displayShutdown()
//{
//  digitalWrite(EN_BOOST, LOW); 
//  delay(100);
//}

/** ===========================================================
 * \fn      displaySleep
 * \brief   puts the display in a sleep mode or wakes it up
 *
 * \param   (bool) true = sleep, false = wake up
 * \return  -
 ============================================================== */
void displaySleep(boolean y)
{
  if (y)
  {
    displayOff();
    digitalWrite(EN_BOOST, LOW); 
    delay(100);  // VCC power up delay
  }
  else
  {
    digitalWrite(EN_BOOST, HIGH); 
    delay(100);  // VCC power up delay
    displayOn();
  }
}

/** ===========================================================
 * \fn      displayInvert
 * \brief   inverts the display content
 ============================================================== */
void displayInvert()
{
  writeCommand(0xA7);
}
/** ===========================================================
 * \fn      displayNormal
 * \brief   reinverts the display content (set to normal)
 ============================================================== */
void displayNormal()
{
  writeCommand(0xA4);
}

/** ===========================================================
 * \fn      displayOn
 * \brief   turns the display on
 ============================================================== */
void displayOn()
{
  #ifdef DEBUGGING
  Serial.println("display on");
  #endif
  
//  digitalWrite(EN_BOOST, HIGH); 
//  delay(100);
  writeCommand(0xAF);
  delay(1);
}

/** ===========================================================
 * \fn      displayOff
 * \brief   turns the display off
 ============================================================== */
void displayOff()
{
  #ifdef DEBUGGING
  Serial.println("display off");
  #endif
  
  writeCommand(0xAE);
  delay(1);
//  digitalWrite(EN_BOOST, LOW);
//  delay(100);
} 

/** ===========================================================
 * \fn      displaySetContrast
 * \brief   set contrast of the display
 *
 * \param   (byte) contrast 0... 255 (127 = default)
 * \return  -
 ============================================================== */
void displaySetContrast(byte contrast)
{
  writeCommand(0x81);
  writeCommand(contrast);
}

/** ===========================================================
 * \fn      startHScroll
 * \brief   start a horizontal scrolling
 *
 * \param   (byte) start scroll address (0... 127)
 *          (byte) end scroll address (0... 127)
 *          CAUTION: startScrollAdr + endScrollAdr <= 128
 *          (byte) step width to scroll each 6 frames (0.. 127)
 * \return  -
 ============================================================== */
void startHScroll(byte startScrollAdr, byte endScrollAdr, byte stepWidth)
{
  writeCommand(0x26);             // continous scroll
  writeCommand(stepWidth);        // horizontal scroll by x column (0... 63)
  writeCommand(startScrollAdr);   // start row address (0... 127)
  writeCommand(0);                // scrolling time interval (0 = 6, 1 = 10, 2 = 100, 3 = 200 frames)
  writeCommand(endScrollAdr);     // end row address (0... 127)
  writeCommand(0);                // no vertical scroll
  writeCommand(0x2F);             // activate scrolling
}

/** ===========================================================
 * \fn      startVScroll
 * \brief   start a vertical scrolling
 *
 * \param   (byte) start scroll address (0... 127)
 *          (byte) end scroll address (0... 127)
 *          CAUTION: startScrollAdr + endScrollAdr <= 128
 *          (byte) step width to scroll each 6 frames (0.. 127)
 * \return  -
 ============================================================== */
void startVScroll(byte startScrollAdr, byte endScrollAdr, byte stepWidth)
{ // startScrollAdr + endScrollAdr <= 128
  writeCommand(0x26);             // continous scroll
  writeCommand(0);                // no horizontal scroll
  writeCommand(startScrollAdr);   // start row address (0... 127)
  writeCommand(0);                // scrolling time interval (0 = 6, 1 = 10, 2 = 100, 3 = 200 frames)
  writeCommand(endScrollAdr);     // end row address (0... 127)
  writeCommand(stepWidth);        // vertical scroll by x row (0... 63)
  writeCommand(0x2F);             // activate scrolling
}

/** ===========================================================
 * \fn      startDScroll
 * \brief   start a diagonal scrolling
 *
 * \param   (byte) start scroll address (0... 127)
 *          (byte) end scroll address (0... 127)
 *          CAUTION: startScrollAdr + endScrollAdr <= 128
 *          (byte) step width to scroll each 6 frames (0.. 127)
 * \return  -
 ============================================================== */
void startDScroll(byte startScrollAdr, byte endScrollAdr, byte stepWidth)
{ // startScrollAdr + endScrollAdr <= 128
  writeCommand(0x26);             // horizontal scroll
  writeCommand(stepWidth);        // horizontal scroll by x column (0... 63)
  writeCommand(startScrollAdr);   // start row address (0... 127)
  writeCommand(0);                // scrolling time interval (0 = 6, 1 = 10, 2 = 100, 3 = 200 frames)
  writeCommand(endScrollAdr);     // end row address (0... 127)
  writeCommand(stepWidth);        // vertical scroll by x row (0... 63)
  writeCommand(0x2F);             // activate scrolling
}

/** ===========================================================
 * \fn      stopScroll
 * \brief   stop scrolling
 *
 * \param   -
 * \return  -
 ============================================================== */
void stopScroll()
{
  writeCommand(0x2E);            // deactivate scrolling
}

/** ===========================================================
 * \fn      setDisplayAddress
 * \brief   sets the address of the columne (x) and the row (y)
 *          (the end address is always the maximum)
 *
 * \param   (int) x = column (0... 63)
 *          (int) y = row (0... 127)
 * \return  -
 ============================================================== */
static void setDisplayAddress(int x, int y)
{
// #ifdef DEBUGGING
//   Serial.println("set display address");
// #endif

  //  /* limit input paramters */
  //  if ((x >= NUMBER_OF_PIXELS_PER_COLUMN / 2) || (y >= NUMBER_OF_PIXELS_PER_ROW))
  //  {
  //    x = NUMBER_OF_PIXELS_PER_COLUMN / 2 - 1;
  //    y = NUMBER_OF_PIXELS_PER_ROW - 1;
  //  }

  /* set column address */
  writeCommand(0x15);
  writeCommand(x);                                   // start adr.
  writeCommand(NUMBER_OF_PIXELS_PER_COLUMN / 2 - 1);   // end adr.

  /* set row address */
  writeCommand(0x75);
  writeCommand(y);                                   // start adr.
  writeCommand(NUMBER_OF_PIXELS_PER_ROW - 1);          // end adr.
}

/** ===========================================================
 * \fn      setDisplayAddress
 * \brief   sets the start and the end address of the
 *          columne (x) and the row (y)
 *
 * \param   (int) x = start column (0... 63)
 *          (int) x_ = end column (0... 63)
 *          (int) y = start row (0... 127)
 *          (int) y_ = end row (0... 127)
 * \return  -
 ============================================================== */
/* overload function */
static void setDisplayAddress(int x, int x_, int y, int y_)
{
// #ifdef DEBUGGING
//   Serial.println("set display address");
// #endif

  /* set column address */
  writeCommand(0x15);
  writeCommand(x);                                   // start adr.
  writeCommand(x_);   // end adr.

  /* set row address */
  writeCommand(0x75);
  writeCommand(y);                                   // start adr.
  writeCommand(y_);          // end adr.
}

/** ===========================================================
 * \fn      writeCommand
 * \brief   writes a command byte to the OLED controller
 *
 * \param   (byte) command byte
 * \return  -
 ============================================================== */
static void writeCommand(byte command)
{
  //digitalWrite(D_C, LOW);    // D/C = 0 -> write command
  clearBit(D_C_PORT, D_C_PIN);

  writeByte(command);
}

/** ===========================================================
 * \fn      writeData
 * \brief   writes a data byte to the OLED controller
 *
 * \param   (byte) data byte
 * \return  -
 ============================================================== */
static void writeData(byte data)
{
//  digitalWrite(D_C, HIGH);    // D/C = 1 -> write data (automatic address increment)
  setBit(D_C_PORT, D_C_PIN);

  writeByte(data);
}

/** ===========================================================
 * \fn      writeByte
 * \brief   writes a non-specific byte via SPI to the OLED
 *          controller
 *
 * \param   (byte) byte value
 * \return  -
 ============================================================== */
static void writeByte(byte byteValue)
{
  /* HW SPI (~250ns) */
  clearBit(CS_PORT, CS_PIN);      // reset chip select pin PB0 (OLED display enabled)
  SPI.transfer(byteValue);
  setBit(CS_PORT, CS_PIN);        // set chip select pin PB0 (OLED display disabled)
}

/* function to handle the buttons ----------------------------- */
/** ===========================================================
 * \fn      getButtonState1
 * \brief   returns current button state (taking a given 
 *          debounce time into account)
 *
 * \param   -
 * \return  (bool) false = button not pushed
 *                 true = button pushed
 ============================================================== */
boolean getButtonState1()
{
  static unsigned long debounceTimeButton1 = 0;
  boolean state = false;
  
  if (!getBit(BUTTON1_PIN, BUTTON1_PIN_NR))
  {
    state = true;
    debounceTimeButton1 = millis();
  }
  else if ((millis() - debounceTimeButton1) <= DEBOUNCE_TIME) state = true;
  
  return state;
}

/** ===========================================================
 * \fn      getButtonState2
 * \brief   returns current button state (taking a given 
 *          debounce time into account)
 *
 * \param   -
 * \return  (bool) false = button not pushed
 *                 true = button pushed
 ============================================================== */
boolean getButtonState2()
{
  static unsigned long debounceTimeButton2 = 0;
  boolean state = false;
  
  if (!getBit(BUTTON2_PIN, BUTTON2_PIN_NR))
  {
    state = true;
    debounceTimeButton2 = millis();
  }
  else if ((millis() - debounceTimeButton2) <= DEBOUNCE_TIME) state = true;
  
  return state;
}

/** ===========================================================
 * \fn      getButtonState3
 * \brief   returns current button state (taking a given 
 *          debounce time into account)
 *
 * \param   -
 * \return  (bool) false = button not pushed
 *                 true = button pushed
 ============================================================== */
boolean getButtonState3()
{
  static unsigned long debounceTimeButton3 = 0;
  boolean state = false;
  
  if (!getBit(BUTTON3_PIN, BUTTON3_PIN_NR))
  {
    state = true;
    debounceTimeButton3 = millis();
  }
  else if ((millis() - debounceTimeButton3) <= DEBOUNCE_TIME) state = true;
  
  return state;
}

/* functions to handle ADCs ----------------------------------- */
/** ===========================================================
 * \fn      initADC
 * \brief   initializes the ADC
 *
 * \param   -
 * \return  -
 ============================================================== */
static void initADC()
{
  ADMUX = (1<<REFS1) | (1<<REFS0);     // define internal 2.56V voltage reference
  ADCSRA = (1<<ADPS1) | (1<<ADPS0);    // frequenzy prescaler = 8
  ADCSRA |= (1<<ADEN);                 // enable ADC

  /* "dummy-readout" to warm up */
//  for (byte n = 0; n < 5; n++)
//  {
  ADCSRA |= (1<<ADSC);                 // single conversion
  while (ADCSRA & (1<<ADSC));          // wait until ADC is completed
  (void) ADC;                          // read value to reset the result
//  }
}

/** ===========================================================
 * \fn      readADC
 * \brief   returns ADC value of choosen ADC
 *
 * \param   (byte) ADCx
 * \return  (uint) ADC value (0... 1023)
 ============================================================== */
unsigned int readADC(byte adc)
{ 
  /* set MUX5 and subtract an offset if ADC > 7 */
  if (adc > 7)
  {
    ADCSRB |= (1 << MUX5);      // set bit 5
    adc -= 8;                   // offset
  }
  else ADCSRB &= ~(1 << MUX5);  // reset bit 5

  /* choose channel without editing other bits */
  ADMUX = (ADMUX & ~(0x1F)) | (adc & 0x1F);
  ADCSRA |= (1<<ADSC);          // single conversion
  while (ADCSRA & (1<<ADSC));   // wait until ADC is completed
  return ADC;                   // return converted value
}

/** ===========================================================
 * \fn      readADCAvg
 * \brief   returns average ADC value of choosen ADC and
 *          given number of samples
 *
 * \param   (byte) ADCx
 *          (uint) number of samples
 * \return  (uint) ADC value (0... 1023)
 ============================================================== */
unsigned int readADCAvg(byte adc, unsigned int nSamples)
{
  unsigned int sum = 0;

  for (unsigned int i = 1; i <= nSamples; ++i)
  {
    sum += readADC(adc);
  }

  return (sum / nSamples);
}

/** ===========================================================
 * \fn      readBatteryVoltage
 * \brief   returns ADC value of current battery voltage
 *          (internal 2.56V voltage as reference)
 *
 * \param   -
 * \return  (uint) ADC value (0... 1023)
 ============================================================== */
unsigned int readBatteryVoltage()
{
  return readADC(V_SCAN_ADC);
}

/* functions to handle the RTC -------------------------------- */
/** ===========================================================
 * \fn      enableRTC
 * \brief   enables or disables the RTC
 *
 * \param   (bool) true = enable, false = disable
 * \return  -
 ============================================================== */
void enableRTC(boolean en)
{
  byte secRegister;

  /* get sec register */
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(SEC);
  Wire.endTransmission();
  
  Wire.requestFrom(RTC_ADDRESS, 1);
  secRegister = Wire.read();

  /* enable or disable the RTC */
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(RTC_);
  if (en) Wire.write(secRegister | (1 << 7));
  else Wire.write(secRegister & ~(1 << 7));
  Wire.endTransmission();
}

/** ===========================================================
 * \fn      getOscillatorState
 * \brief   gets the oscillator state
 *
 * \param   -
 * \return  (bool) true = oscillator's running, false = not 
 ============================================================== */
boolean getOscillatorState()
{
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(DAY);
  Wire.endTransmission();
  
  Wire.requestFrom(RTC_ADDRESS, 1);
  return (Wire.read() >> 5);
}

/** ===========================================================
 * \fn      initRTC
 * \brief   initializes the RTC
 *
 * \param   -
 * \return  - 
 ============================================================== */
static void initRTC()
{
  enableRTC(true);

  while (!getOscillatorState())
  {
    #ifdef DEBUGGING
    Serial.println("enabling RTC...");
    #endif
  }

  // if (!getOscillatorState())  // check if running (hang up otherwise)
  // {
  //   #ifdef DEBUGGING
  //   Serial.println("RTC initialization failed!");
  //   #endif

  //   while(1);
  // }

  setHourTimeFormat24(true);
  enableBatteryBackupSupply(true);
}

/** ===========================================================
 * \fn      bcd2bin
 * \brief   converts a BCD-code to a binary code 
 *
 * \param   (int) bcd code
 * \return  (int) binary code
 ============================================================== */
static int bcd2bin(int val)
{
  return val - 6 * (val >> 4);
}

/** ===========================================================
 * \fn      bin2bcd
 * \brief   converts a binary code to a BCD-code
 *
 * \param   (int) binary code
 * \return  (int) bcd code
 ============================================================== */
static int bin2bcd(int val)
{
  return val + 6 * (val / 10);
}

/** ===========================================================
 * \fn      setHourTimeFormat24
 * \brief   sets the hour time format bit
 *
 * \param   (bool) true = 24h, false = 12h format
 * \return  -
 ============================================================== */
static void setHourTimeFormat24(boolean b)
{
  byte hourRegister;

  /* get hour register */
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(HOUR);
  Wire.endTransmission();
  
  Wire.requestFrom(RTC_ADDRESS, 1);
  hourRegister = Wire.read();

  /* set hour time format */
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(HOUR);
  if (b) Wire.write(hourRegister & ~(1 << 6));  // 0 = 24-hour format
  else Wire.write(hourRegister | (1 << 6));     // 1 = 12-hour format
  Wire.endTransmission();
}

// /** ===========================================================
//  * \fn      setAMIndicator
//  * \brief   sets the AM/PM indicator
//  *
//  * \param   (bool) true = AM, false = PM
//  * \return  -
//  ============================================================== */
// void setAMIndicator(boolean b)
// {
//   byte hourRegister;

//   /* get hour register */
//   Wire.beginTransmission(RTC_ADDRESS);
//   Wire.write(HOUR);
//   Wire.endTransmission();
  
//   Wire.requestFrom(RTC_ADDRESS, 1);
//   hourRegister = Wire.read();

//   /* check if 12-hour format is enabled */
//   if ((hourRegister & (1 << 6)))
//   {
//     /* set AM/PM indicator */
//     Wire.beginTransmission(RTC_ADDRESS);
//     Wire.write(HOUR);
//     if (b) Wire.write(hourRegister & ~(1 << 5));  // 0 = AM
//     else Wire.write(hourRegister | (1 << 5));     // 1 = PM
//     Wire.endTransmission();
//   }
// }

/** ===========================================================
 * \fn      enableBatteryBackupSupply
 * \brief   enables or disables the external battery backup
 *          supply
 *
 * \param   (bool) true = enabled, false = disabled
 * \return  -
 ============================================================== */
void enableBatteryBackupSupply(boolean en)
{
  byte dayRegister;

  /* get day register */
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(DAY);
  Wire.endTransmission();
  
  Wire.requestFrom(RTC_ADDRESS, 1);
  dayRegister = Wire.read();

  /* enable battery backup supply */
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(DAY);
  if (en) Wire.write(dayRegister | (1 << 3));
  else Wire.write(dayRegister & ~(1 << 3));
  Wire.endTransmission();
}

/** ===========================================================
 * \fn      setCurrentTime
 * \brief   transmits current time (defined in the Time
 *          structure) to the RTC
 *
 * \param   -
 * \return  -
 ============================================================== */
void setCurrentTime()
{
  byte hourRegister, dayRegister;

  /* disable oscillator */
  enableRTC(false);
  while (getOscillatorState())
  {
    #ifdef DEBUGGING
    Serial.println("disabling RTC...");
    #endif
  }

  #ifdef DEBUGGING
  Serial.println("set current time");
  #endif

  /* get the registers with the status bits */
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(HOUR);
  Wire.endTransmission();
  
  Wire.requestFrom(RTC_ADDRESS, 2);
  hourRegister = Wire.read();
  dayRegister = Wire.read();

  /* set current time */
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(RTC_);
  Wire.write(bin2bcd(Time.sec));
  Wire.write(bin2bcd(Time.min));
  Wire.write(bin2bcd(Time.hour) | (hourRegister & ~REG_MASK_HOUR));
  Wire.write(bin2bcd(Time.day) | (dayRegister & ~REG_MASK_DAY));
  Wire.write(bin2bcd(Time.date));
  Wire.write(bin2bcd(Time.month));
  Wire.write(bin2bcd(Time.year - MILLENNIUM));
  Wire.endTransmission();

  /* reenable oscillator */
  enableRTC(true);

  while (!getOscillatorState())
  {
    #ifdef DEBUGGING
    Serial.println("enabling RTC...");
    #endif
  }
}

/** ===========================================================
 * \fn      setSecond
 * \brief   transmits given second value to the RTC
 *
 * \param   (byte) second
 * \return  -
 ============================================================== */
void setSecond(byte ss)
{
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(SEC);
  Wire.write(bin2bcd(ss) | (1 << 7)); // enable oscillator
  Wire.endTransmission();
}

/** ===========================================================
 * \fn      setMinute
 * \brief   transmits given minute value to the RTC
 *
 * \param   (byte) minute
 * \return  -
 ============================================================== */
void setMinute(byte mm)
{
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(MIN);
  Wire.write(bin2bcd(mm));
  Wire.endTransmission();
}

/** ===========================================================
 * \fn      setHour
 * \brief   transmits given houre value to the RTC
 *
 * \param   (byte) houre
 * \return  -
 ============================================================== */
void setHour(byte hh)
{
  byte hourRegister;

  /* get hour registers */
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(HOUR);
  Wire.endTransmission();
  
  Wire.requestFrom(RTC_ADDRESS, 1);
  hourRegister = Wire.read();

  /* set hour */
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(HOUR);
  Wire.write(bin2bcd(hh) | (hourRegister & ~REG_MASK_HOUR));
  Wire.endTransmission();
}

/** ===========================================================
 * \fn      setDay
 * \brief   transmits given day value to the RTC
 *
 * \param   (byte) day
 * \return  -
 ============================================================== */
void setDay(byte d)
{
  byte dayRegister;

  /* get day registers */
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(DAY);
  Wire.endTransmission();
  
  Wire.requestFrom(RTC_ADDRESS, 1);
  dayRegister = Wire.read();

  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(DAY);
  Wire.write(bin2bcd(d) | (dayRegister & ~REG_MASK_DAY));
  Wire.endTransmission();
}

/** ===========================================================
 * \fn      setDate
 * \brief   transmits given date value to the RTC
 *
 * \param   (byte) date
 * \return  -
 ============================================================== */
void setDate(byte date)
{
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(DATE);
  Wire.write(bin2bcd(date));
  Wire.endTransmission();
}

/** ===========================================================
 * \fn      setMonth
 * \brief   transmits given month value to the RTC
 *
 * \param   (byte) month
 * \return  -
 ============================================================== */
void setMonth(byte m)
{
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(MONTH);
  Wire.write(bin2bcd(m));
  Wire.endTransmission();
}

/** ===========================================================
 * \fn      setYear
 * \brief   transmits given year value to the RTC
 *
 * \param   (byte) year
 * \return  -
 ============================================================== */
void setYear(byte y)
{
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(YEAR);
  Wire.write(bin2bcd(y - MILLENNIUM));
  Wire.endTransmission();
}

/** ===========================================================
 * \fn      getCurrentTime
 * \brief   gets current time of the RTC and saves it into the
 *          Time structure
 *
 * \param   -
 * \return  -
 ============================================================== */
void getCurrentTime()
{
  #ifdef DEBUGGING
  Serial.println("get current time");
  #endif

  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(RTC_);
  Wire.endTransmission();
  
  Wire.requestFrom(RTC_ADDRESS, 7);
  Time.sec = bcd2bin(Wire.read() & REG_MASK_SEC);
  Time.min = bcd2bin(Wire.read() & REG_MASK_MIN);
  Time.hour = bcd2bin(Wire.read() & REG_MASK_HOUR);
  Time.day = bcd2bin(Wire.read() & REG_MASK_DAY);
  Time.date = bcd2bin(Wire.read() & REG_MASK_DATE);
  Time.month = bcd2bin(Wire.read() & REG_MASK_MONTH);
  Time.year = bcd2bin(Wire.read() & REG_MASK_YEAR) + MILLENNIUM;
}

/** ===========================================================
 * \fn      getSecond
 * \brief   gets current second value of the RTC
 *
 * \param   -
 * \return  (byte) second
 ============================================================== */
byte getSecond()
{
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(SEC);
  Wire.endTransmission();
  
  Wire.requestFrom(RTC_ADDRESS, 1);
  return bcd2bin(Wire.read() & REG_MASK_SEC);
}

/** ===========================================================
 * \fn      getMinute
 * \brief   gets current minute value of the RTC
 *
 * \param   -
 * \return  (byte) minute
 ============================================================== */
byte getMinute()
{
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(MIN);
  Wire.endTransmission();
  
  Wire.requestFrom(RTC_ADDRESS, 1);
  return bcd2bin(Wire.read() & REG_MASK_MIN);
}

/** ===========================================================
 * \fn      getHour
 * \brief   gets current hour value of the RTC
 *
 * \param   -
 * \return  (byte) hour
 ============================================================== */
byte getHour()
{
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(HOUR);
  Wire.endTransmission();
  
  Wire.requestFrom(RTC_ADDRESS, 1);
  return bcd2bin(Wire.read() & REG_MASK_HOUR);
}

/** ===========================================================
 * \fn      getDay
 * \brief   gets current day value of the RTC
 *
 * \param   -
 * \return  (byte) day
 ============================================================== */
byte getDay()
{
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(DAY);
  Wire.endTransmission();
  
  Wire.requestFrom(RTC_ADDRESS, 1);
  return bcd2bin(Wire.read() & REG_MASK_DAY);
}

/** ===========================================================
 * \fn      getDate
 * \brief   gets current date value of the RTC
 *
 * \param   -
 * \return  (byte) date
 ============================================================== */
byte getDate()
{
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(DATE);
  Wire.endTransmission();
  
  Wire.requestFrom(RTC_ADDRESS, 1);
  return bcd2bin(Wire.read() & REG_MASK_DATE);
}

/** ===========================================================
 * \fn      getMonth
 * \brief   gets current month value of the RTC
 *
 * \param   -
 * \return  (byte) month
 ============================================================== */
byte getMonth()
{
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(MONTH);
  Wire.endTransmission();
  
  Wire.requestFrom(RTC_ADDRESS, 1);
  return bcd2bin(Wire.read() & REG_MASK_MONTH);
}

/** ===========================================================
 * \fn      getYear
 * \brief   gets current year value of the RTC
 *
 * \param   -
 * \return  (int) year
 ============================================================== */
int getYear()
{  
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(YEAR);
  Wire.endTransmission();
  
  Wire.requestFrom(RTC_ADDRESS, 1);
  return (bcd2bin(Wire.read() & REG_MASK_YEAR) + MILLENNIUM);
}

//  /* receive */
//  byte nBytes = 1;
//  char messageCustomSymbol[MESSAGE_SIZE];
//  Wire.requestFrom((byte)RTC_ADDRESS, (byte)nBytes);
//  for (unsigned int cnt = 0; Wire.available() && cnt < sizeof(messageCustomSymbol); cnt++)
//  {
//    messageCustomSymbol[cnt] = Wire.read();
//  }

/** ===========================================================
 * \fn      showRTC
 * \brief   serial-prints the current time of the customSymbol
 *
 * \param   -
 * \return  -
 ============================================================== */
#ifdef DEBUGGING
void showRTC()
{
  Serial.print("ss = ");
  Serial.println(getSecond());
  Serial.print("mm = ");
  Serial.println(getMinute());
  Serial.print("hh = ");
  Serial.println(getHour());
  Serial.print("d = ");
  Serial.println(getDay());
  Serial.print("date = ");
  Serial.println(getDate());
  Serial.print("m = ");
  Serial.println(getMonth());
  Serial.print("y = ");
  Serial.println(getYear());
}
#endif

/**
 * @}
 */
 