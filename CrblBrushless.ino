
#define HARDWARE_NAME "CRBL_20250823"
// Hardware ESP8266 Weemos D1
// Arduino ide 1.8.15
// File/Preferences/Additionnal boards URLs + http://arduino.esp8266.com/stable/package_esp8266com_index.json
// CardType:NodeMCU 0.9 (ESP12) UpSpeed:115k CpuFreq:80Mhz FlashSz:4Mb 

// Arduino IDE 2.3.6
// File/Preferences/Additionnal boards URLs + ,https://dl.espressif.com/dl/package_esp32_index.json
// ESP32

// TODO a led ws2812 blink on command received
// TODO a color faint on command processed 

typedef uint32_t pr_uint32_t[3];
typedef int32_t pr_int32_t[3];

#if defined( ESP8266)
// D1...D11 def by os
 //#define WITH_ESPNOW // TODO_LATER
 //#define WITH_ESPNOW_MASTER
#elif defined( ESP32)
 // mapping is ... for FOC
 // ESP32 Dev Module
  #define D1 32
  #define D2 33
  #define D6 25
  //#define analogWrite( x, y) ledcWrite( x, y) // TODO_HERE
#else
  #define ICACHE_RAM_ATTR
  #define D1 1
  #define D2 2
  #define D3 3
  #define D4 4
  #define D5 5
  #define D6 6
  #define D7 7
  #define D8 8
  #define D9 9
  #define D10 10
  #define D11 11

  #define A0 36
#endif

int PinModes[20]={0};

#define RunOtherTasks( T) delay(T)

//#define TEST

#define KFACT 1000 // precision to compute intervals to express valuse 0..1 in integer world

// just to identify on command line
#define MACHINE_NAME "CRBL_AXEX"

// WITH_WS2812 activate RGB leds on the given line
#define WITH_WS2812 D3

// AXE_MINE : Axe followed
// 0 respond to X Y Z like M0 M1 M2
// 1 respond to X Y Z like M1 M2 M3
// k respond to X Y Z like Ak Ak+1 Ak+2

// AxeX
#define AXE_MINE 0
#define WITH_BRLESS {  D1, -1, D2, -1, D6, -1} // A+ A- B+ B- C+ C- esp8266
//if needed define WITH_BRLESS_DIR // mode direction // A ADir instead of A+ A-

//#define MACHINE_TYPE "CRBL_AXEY"
#define AXE_MINE 1
//#define WITH_STEPPER { D2, D1} // DIR PULSE

//#define MACHINE_TYPE "CRBL_AXEZ"
#define AXE_MINE 2
//#define WITH_STEPPER { D2, D1} // DIR PULSE

// 50.0 too low
#define STEP_PER_TURN 500.0

#ifdef WITH_WS2812
// NeoPixel strip led ws2812 https://github.com/adafruit/Adafruit_NeoPixel
#include <Adafruit_NeoPixel.h>
//#include <avr/power.h>
#endif /* WITH_WS2812 */

#ifdef WITH_ESPNOW
  #include <esp_now.h>
  #include <WiFi.h>
#endif /*WITH_ESPNOW */
int GcodeMotorId = 1; // X

uint32_t LastDispMs = 0;
uint32_t DbgLastActivityMillis = 0;

// -- variables declaration

// 0 - rien du tout sauf hello pwr on and GCode
// 1 - boot et infos systeme
// 2 - interessant pour le client (un script externe, par exemple liste des passagers)
// 3 - Dev
int Verbose = 2;


void dbgprintf( int Lev, const char *fmt, ... ) {
  char buf[128]; // resulting string limited to 128 chars

  if ( Lev <= Verbose) {
    va_list args;
    va_start (args, fmt );
    vsnprintf(buf, 128, fmt, args);
    va_end (args);
    Serial.print(buf);
  }
}

// cause time cycle regularly
// GL_TYPE DiffTime( GL_TYPE T0, GL_TYPE T1) ... .ino is not .c ...
uint32_t ICACHE_RAM_ATTR DiffTime( uint32_t T0, uint32_t T1) {
  // TODO_HERE : time cycle, test...
  //if ( T0 > T1) {
  //  return( 0xFFFFFFFF - T0 + T1 + 1);
  //}
  return ( abs((int32_t)(T1 - T0)));
}


// attempt to pass data between threads
void ICACHE_RAM_ATTR pr_uint32_write( pr_uint32_t& Id, uint32_t Val) {
  Id[0] = Val;
  Id[1] = Val;
  Id[2] = Val;
}

uint32_t pr_uint32_read (pr_uint32_t& Id) {
  uint32_t V1, V2, V3, Val = 0;

  V3 = Id[2];
  V2 = Id[1];
  V1 = Id[0];
  if (V1 == V2) {
    Val = V1;
  } else if (V2 == V3) {
    Val = V2;
  } else if (V3 == V1) {
    Val = V3;
  } else {
    Val = V1; // supposed currently writing V2, V1 is the new val
    //  (V3/3+V2/3+V1/3);
  }

  return ( Val);
}

void ICACHE_RAM_ATTR pr_int32_write( pr_int32_t& Id, int32_t Val) {
  Id[0] = Val;
  Id[1] = Val;
  Id[2] = Val;
}

int32_t pr_int32_read (pr_int32_t& Id) {
  uint32_t V1, V2, V3, Val = 0;

  V3 = Id[2];
  V2 = Id[1];
  V1 = Id[0];
  if (V1 == V2) {
    Val = V1;
  } else if (V2 == V3) {
    Val = V2;
  } else if (V3 == V1) {
    Val = V3;
  } else {
    Val = V1; // supposed currently writing V2, V1 is the new val 
    //  (V3/3+V2/3+V1/3);
  }

  return ( Val);
}

#define dbgReadCh Serial.read

  typedef struct {
    uint8_t mac[6];
    uint8_t AboNum;
    uint8_t AboLen;
    uint8_t NbFramesInAir; // Nb frames sent and no ack
    uint8_t State; // 0 Ok, 1 missed some frames, 2 not seen awhile
  } EspNowAboList_t;

#ifdef WITH_ESPNOW
// wifi says to go to T P[1..n]
  // Slv->Master IdMsg=0; uint8_t Mac[6]; AboNum; AboLen
  // Master->Slv IdMsg=1; uint32_t CuurentTime in ms
  // Master->Slv IdMsg=2; uint8_t AboNum; uint8_t AboLen; uint32_t T; uint32_t P[AboNum+i] i in 1 .. AboLen

void EspNowInit()
{
  #ifdef WITH_ESPNOW_MASTER
  #else
  #endif
}

void EspNowLoop()
{
}
#endif /*WITH_ESPNOW */

#ifdef WITH_BRLESS
  // brushless (3 coils) motor
  uint8_t BrushlessPins[] = WITH_BRLESS;
  #define BrushlessNb (sizeof( BrushlessPins) / (6*sizeof(uint8_t)))
  #define MOTOR_BRLESS_IDX 0
  #ifdef WITH_BRLESS_DIR
    uint8_t BrushlessMode = 1;
  #else /* WITH_BRLESS_DIR */
    uint8_t BrushlessMode = 0;
  #endif /* WITH_BRLESS_DIR */
  #define WITH_MOTOR
#else
  #define BrushlessNb 0
#endif /* WITH_BRLESS */

#ifdef WITH_STEPPER
  uint8_t StepperPins[] = WITH_STEPPER;
  #define StepperNb (sizeof( StepperPins) / (2*sizeof(uint8_t)))
  #define MOTOR_STEPPER_IDX 0
  #define WITH_MOTOR
#else /* WITH_STEPPER */
  #define StepperNb 0
#endif /* WITH_STEPPER */

#ifdef WITH_MOTOR

  #define MOTOR_NB ( 0 \
     +BrushlessNb \
     +StepperNb \
      )
  #define MOTOR_MAXTOSC_MS 1000 // supposed time to reach position between 2 osc commands
  #ifndef HOMING_MAX
    #define HOMING_MAX 14000
  #endif /* HOMING_MAX */
  #ifndef HOMING_TIME
    #define HOMING_TIME 6000
  #endif /* HOMING_TIME */


  typedef enum MotorMode_e {
    // motor Mode
    MOTOR_UNDEF = 0,
    MODOR_DC_LR   , //left right, grayIn
    //MOTOR_DC_DP, //Dir Pwm, NoGray
    MOTOR_PWM,   // Pwm 0..9999, NoGray
    MOTOR_PWM2,   // Pwm 0..9999, NoGray, no slice
    MOTOR_STEPPER,
    #ifdef WITH_SERVO
      MOTOR_SERVO,
    #endif
    MOTOR_BRLESS,
    MOTOR_MAXDEFS
  } MotorMode_t;

  typedef struct {
    uint32_t P1; // segment drive P1->P2 if drived per position
    uint32_t T1ms;
    uint32_t P2;
    uint32_t T2ms;

    int8_t Order;      // -1 at boot, 1..3 if WishP2 set externally, 5 to set everything
    uint8_t MotNum;    // to retrieve when we got just a pointer on the struct
    pr_int32_t WishP2; // the app part (out of the interruption) asks for a pos
    pr_uint32_t WishDTms;
    
    int16_t WishDec; // TODO_LATER probab pr_ too
    uint8_t HomingOn;
    uint32_t HomingMs;
    uint8_t DriverMode; // MotorMode_t MODOR_DC_LR & co.

    uint8_t Pin1; // stepper:pin dir
    uint8_t Pin2; // stepper:pin pulse

    #ifdef WITH_STOPPER
      uint8_t DMin; // stopper min
      uint8_t DMax; // stopper max
      uint8_t LastMin;
      uint8_t LastMax;
      uint16_t ReadMin;
      uint16_t ReadMax;
    #endif /* WITH_STOPPER */

    pr_uint32_t Pos; // the position of the motor for the external world

    uint32_t LastPulseUs; // last time we set a value to the motor

    // to compute inside each kind of motor
    uint32_t LastPos;
    uint32_t LastSlice;

    // 200 min microsec up (or down) for a pulse, 2us for a https://www.pololu.com/file/0J590/drv8825.pdf for example
    // min microsec up (or down) for a pulse, >=2us for a https://www.pololu.com/file/0J590/drv8825.pdf for example
    uint32_t MinCommandUs; // minimum time between two commands to the motor in microseconds

    uint16_t InstVal; // las val set in power pin(pwm)

    uint8_t StepperD; // current dir
    uint8_t StepperP; // current Pulse (up or down)

  } Motor_t;


  Motor_t MotorArray[MOTOR_NB];
#endif /* WITH_MOTOR */


void MyAnalogWrite( int Pin, int Duty /* 0 .. 255 */)
{

  #ifdef ESP32
  int LocalDuty = Duty; // *255/1000; for 0..1000
  switch(Pin) // for FOC card
  {
    case 32: ledcWrite( 0, LocalDuty); return;
    case 33: ledcWrite( 1, LocalDuty); return;
    case 25: ledcWrite( 2, LocalDuty); return;
    case 26: ledcWrite( 3, LocalDuty); return;
    case 27: ledcWrite( 4, LocalDuty); return;
    case 14: ledcWrite( 5, LocalDuty); return;
  }
  #endif /* ESP32 */

  analogWrite( Pin, Duty);
}

void MyPinmode( int Pin, int Mode) {
    // dbgprintf( 0, "#%s(%i, )\n", __func__, Pin);

#ifdef ESP32
  int Freq = 30000;
  int Reso = 8;

  switch(Pin)
  {
    case 32: ledcSetup( 0, Freq, Reso);ledcAttachPin( Pin, 0); return;
    case 33: ledcSetup( 1, Freq, Reso);ledcAttachPin( Pin, 1); return;
    case 25: ledcSetup( 2, Freq, Reso);ledcAttachPin( Pin, 2); return;
    case 26: ledcSetup( 3, Freq, Reso);ledcAttachPin( Pin, 3); return;
    case 27: ledcSetup( 4, Freq, Reso);ledcAttachPin( Pin, 4); return;
    case 14: ledcSetup( 5, Freq, Reso);ledcAttachPin( Pin, 5); return;
  }

#endif /* ESP32 */
#ifdef ESP8266
  if ((16 == Pin) && (INPUT_PULLUP == Mode)) {
    // rem : if RESTART_PIN is 16 on ESP8266 a physical pullup (10kOhm?) is required
    // rem : seems no interrupt either
    Mode = INPUT;
    dbgprintf( 1, "# Warn, Restart SwitchInit %i specific (pulldown)\n", Pin);
  }
#endif /* #ifdef ESP8266 */

  if ((Pin >= 0) && (Pin < 20))
  {
    dbgprintf( 0, "# MyPinmode %i - %i\n", Pin, Mode);
    if ((PinModes[Pin] != Mode)) {
      pinMode( Pin, Mode);
      PinModes[Pin] = Mode;
    } else {
      dbgprintf( 1, "# Warn, duplicated init %i\n", Pin);
      pinMode( Pin, Mode);
    }
  }
}

// ------ RGB leds called neopixels

#ifdef WITH_WS2812
#ifndef NEOPIXEL_NUMPIXELS
#define NEOPIXEL_NUMPIXELS 2
#endif
#define PIX_SUBS 8
#define PIX_MAX (256*PIX_SUBS-1)

#define RndMax 8
uint8_t RndTab[RndMax] = {255, 127, 223, 63, 191, 95, 159, 31 };
char RndCount=1;

Adafruit_NeoPixel pixels = Adafruit_NeoPixel( NEOPIXEL_NUMPIXELS, WITH_WS2812/*NEOPIXEL_PIN (default)*/, NEO_GRB + NEO_KHZ800);
uint32_t ws2812_LastDispMs = 0;

/* @brief change value of a neopixel
 * change pixels [PixSt..PixNd[ Color to RGB values, from 0,0,0 up to PIX_MAX-1,PIX_MAX,PIX_MAX
 * PixSt 0..NEOPIXEL_NUMPIXELS-1
*/
int NeopixelPrepare( uint16_t PixSt, uint16_t PixNd, uint16_t r, uint16_t g, uint16_t b) {
  uint16_t R,G,B;

  // dbgprintf( 0,"#%s(PixSt:%i, PixNd:%i", __func__, PixSt, PixNd);
  // dbgprintf( 0,", r:%i, g:%i, b:%i)\n", r, g, b);

   // sanity check
   if (PixSt >= NEOPIXEL_NUMPIXELS)
      return(-1);
   if (PixNd >= NEOPIXEL_NUMPIXELS)
      PixNd = NEOPIXEL_NUMPIXELS-1;

   R = r/PIX_SUBS;
   G = g/PIX_SUBS;
   B = b/PIX_SUBS;
  // low values are a little bit too 'step'
  if (  ((0!=R) && (0 !=G) &&(0 != B))
       &&((255 != R) && (255 != G) && (255 != B))) {
    R = R%255+1;
    if ( RndTab[RndCount]*PIX_SUBS/256 < R%256) {
      R = R+1;
      G = G+1;
      B = B+1;
    }
  }
  
  // some kind of bug with first neopixels?...
  if ((R==G) &&(G==B) && (0 != R)){
    if (R > 0) {
      R--;
    } else {
      R++;
    }
  }

  for(int i=PixSt;i<=PixNd;i++){
    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    // dbgprintf( 0,"#%s setPixelColor(i:%i", __func__, i);
    // dbgprintf( 0,", R:%i, G:%i, B:%i)\n", R, G, B);

//modif OH
//    pixels.setPixelColor(i, pixels.Color( (byte) intensiteLumiereCasier*r, (byte) intensiteLumiereCasier*g, (byte) intensiteLumiereCasier*b)); // Moderately bright green color.
    pixels.setPixelColor(i, pixels.Color( (byte) R, (byte) G, (byte) B)); // Moderately bright green color.
//fin modif OH
  }
  return(0);
}


void ws2812_setup()
{
  pixels.setPin(WITH_WS2812);
  pixels.begin();
  // dbgprintf( 0,"#%s()\n", __func__);
}

void ws2812_loop()
{
  if (DiffTime( ws2812_LastDispMs, millis()) > 4000) {

    // NeopixelPrepare( 0, 1, PIX_MAX, PIX_MAX, PIX_MAX);

    RndCount++;
    RndCount %= RndMax;
    // dbgprintf( 0,"#%s show\n", __func__);
    pixels.show(); // This sends the updated pixel color to the hardware.
    ws2812_LastDispMs = millis();
  }
}

#endif /* WITH_WS2812 */

// ------ Motor management

#ifdef WITH_MOTOR
// move from position 1 to position 0xFFFFFFFF is 2 in distance and -1 in direction
int32_t Vectorize( uint32_t CurrentPos, uint32_t ExpectedPos) {
  int Dir;
  uint32_t Dist;

  ExpectedPos -= CurrentPos;
  if ( ExpectedPos > 0) {
    Dir = 1;
    Dist = ExpectedPos;
  } else {
    Dir = -1;
    Dist = -ExpectedPos;
  }

  if (Dist > 0x7FFFFFFF) {
    Dist = - Dist;
    Dir = Dir * -1;
  }

  return (Dir * Dist);
}
#endif /* WITH_MOTOR */

#ifdef WITH_BRLESS
// brushless is isolated so if soft evolve if can be completely wiped after build ---------
int32_t PwrLast[3];

/**
 * @brief setup for brushless
 */
int BrushlessSetup() {
  int Res = 0;
  Motor_t* pMotor;
  int Idx;
  int PinNum;

  dbgprintf( 0,"#%s()\n", __func__);
  for (Idx = 0; Idx<3; Idx++) {
    PwrLast[Idx] = 0;
  }

  for( Idx = 0; Idx < BrushlessNb; Idx++){
    
    pMotor = &(MotorArray[Idx+MOTOR_BRLESS_IDX]);

    pMotor->StepperD = 0;
    pMotor->StepperP = 0;
    pMotor->Pin1 = Idx*6;
    
    for(PinNum = 0; PinNum<6; PinNum++) {
      if (BrushlessPins[pMotor->Pin1 + PinNum] >= 0) {
        MyPinmode( BrushlessPins[pMotor->Pin1 + PinNum], OUTPUT);
        SetFreq( BrushlessPins[pMotor->Pin1 + PinNum], 32);
        //digitalWrite( BrushlessPins[pMotor->Pin1 + PinNum], 0);
      }
    }

    // 15us for typical mofset
    //pMotor->MinCommandUs = 500;
    pMotor->MinCommandUs = 1500;

    if (MOTOR_UNDEF == pMotor->DriverMode) {
      pMotor->DriverMode = MOTOR_BRLESS;
    }
  }
  Res = 1; // init done
  // dbgprintf( 0,"#%s Res %i \n", __func__, Res);
  return( Res);
}


int32_t BrushlessLoop( void* pMotorVoid, int32_t ExpectedPos) {
  Motor_t* pMotor = (Motor_t*) pMotorVoid;  // wth builder cries about Motor_t not defined if declared in args ...
  uint32_t StepperPos = pMotor->LastPos;
  int32_t Pwr[3];
  int32_t PwrMin;
  int Idx = 0;
  int Torque = 127; // up to 255

  // dbgprintf( 2,"#%s( ... , %i)\n", __func__, ExpectedPos);
#ifdef TEST
  dbgprintf( 2,"Brless Test \n");
  Idx = 0;
        MyAnalogWrite(BrushlessPins[pMotor->Pin1 + 2*Idx], 254);
  Idx = 1;
        MyAnalogWrite(BrushlessPins[pMotor->Pin1 + 2*Idx], 128);
  Idx = 2;
        MyAnalogWrite(BrushlessPins[pMotor->Pin1 + 2*Idx], 0);
  return(0);
#endif

  //dbgprintf( 2,"#");
  //dbgprintf( 2,"Brless Pos:");
  //dbgprintf( 2,"%i ", ExpectedPos);
  // compute cyclic
  PwrMin = 2*Torque;
  for (Idx = 0; Idx<3; Idx++) {
    // TODO_LATER : integer compute
    // Pwr[Idx]=127+sin(ExpectedPos*2*PI/STEP_PER_TURN+Idx*2*PI/3.0)*127;
    Pwr[Idx]=Torque/2+sin(ExpectedPos*2*PI/STEP_PER_TURN+Idx*2*PI/3.0)*sin(ExpectedPos*2*PI/STEP_PER_TURN+Idx*2*PI/3.0)*Torque;
    if ( Pwr[Idx] < PwrMin)
      PwrMin = Pwr[Idx];
  }
  for (Idx = 0; Idx<3; Idx++) {
    Pwr[Idx]= Pwr[Idx] - PwrMin;
    if (Pwr[Idx] < 0)
      Pwr[Idx] = 0;
    if (Pwr[Idx] >= 254)
      Pwr[Idx] = 254;
  }
  
  // apply powers
  for (Idx = 0; Idx<3; Idx++) {

    // dump
    // dbgprintf( 2,"%4i", Pwr[Idx]);
    // if(Idx <2) {
    //   dbgprintf( 2,",");
    // }

    if(1==BrushlessMode) {
      // TODO_LATER : pwm brownian, don't write if already done by last
      if(Pwr[Idx] < 0){
        MyAnalogWrite(BrushlessPins[pMotor->Pin1 + 2*Idx+1], -Pwr[Idx]);
        digitalWrite(BrushlessPins[pMotor->Pin1 + 2*Idx], 1);
      } else if(Pwr[Idx] > 0){
        MyAnalogWrite(BrushlessPins[pMotor->Pin1 + 2*Idx+1], Pwr[Idx]);
        digitalWrite(BrushlessPins[pMotor->Pin1 + 2*Idx], 1);
      } else {
        digitalWrite(BrushlessPins[pMotor->Pin1 + 2*Idx], 0);
      }
    } else { // not BrushlessMode
      //dbgprintf( 2," Pwr %i p %i %i ", Pwr[Idx], BrushlessPins[pMotor->Pin1 + 2*Idx]);
      MyAnalogWrite(BrushlessPins[pMotor->Pin1 + 2*Idx], Pwr[Idx]);
    } // endif BrushlessMode
    //delay(1); // no use?
    PwrLast[Idx] = Pwr[Idx];
  }
  
  StepperPos = ExpectedPos;
  pMotor->LastPos = StepperPos;

  // dbgprintf( 2," %s ret %i\n", __func__, StepperPos);
  return ( StepperPos);
}

// end of brushless isolation -------------------------------------------------------------
#endif /* WITH_BRLESS */


#ifdef WITH_STEPPER

void StepperSetup( ) {

  Motor_t* pMotor;
  int Idx;
  int PinDir;
  int PinPulse;

  dbgprintf( 0, "#%s()\n", __func__);

  for( Idx = 0; Idx < StepperNb; Idx++){
    
    pMotor = &(MotorArray[Idx+MOTOR_STEPPER_IDX]);
    PinDir = StepperPins[0+2*Idx];
    PinPulse = StepperPins[1+2*Idx];

    pMotor->StepperD = 0;
    pMotor->StepperP = 0;
    pMotor->Pin1 = PinDir;
    pMotor->Pin2 = PinPulse;
    
    MyPinmode( PinDir, OUTPUT);
    #ifdef WITH_INVERT1
      if(1 == Idx) {
        dbgprintf( 3, " Idx %i inverted %i\n", Idx, PinDir);
        digitalWrite( PinDir, !pMotor->StepperD);
      } else {
        dbgprintf( 3, " Idx %i standard %i\n", Idx, PinDir);
        digitalWrite( PinDir, pMotor->StepperD);
      }
    #else /* WITH_INVERT1 */
      dbgprintf( 3, " Idx %i normal %i\n", Idx, PinDir);
      digitalWrite( PinDir, pMotor->StepperD);
    #endif /* WITH_INVERT1 */
    MyPinmode( PinPulse, OUTPUT);
    digitalWrite( PinPulse, pMotor->StepperP);

    // 200 min microsec up (or down) for a pulse, theoric 2us for a https://www.pololu.com/file/0J590/drv8825.pdf for example
    // 150 good for small motors, too short for Big without microstep
    //pMotor->MinCommandUs = 500;
    pMotor->MinCommandUs = 150;

    if (MOTOR_UNDEF == pMotor->DriverMode) {
      pMotor->DriverMode = MOTOR_STEPPER;
    }
  }
  dbgprintf( 0,"#%send\n", __func__);
}

uint32_t StepperLoop( void* pMotorVoid, uint32_t ExpectedPos) {
  Motor_t* pMotor = (Motor_t*) pMotorVoid;  // wth builder cries about Motor_t not defined if declared in args ...
  long CurTime;
  int32_t Dist;
  uint8_t DirPin = pMotor->Pin1;
  uint8_t PulsPin = pMotor->Pin2;
  uint32_t StepperPos = pMotor->LastPos;
  uint8_t TMin = 0;

  CurTime = millis();

  Dist = Vectorize( StepperPos, ExpectedPos);
  // dbgprintf( 2, " Pos:%5u, Expect:%5u,", StepperPos, ExpectedPos);
  // dbgprintf( 2, " D:%5i\n", Dist);

  #ifdef WITH_STOPPER
    if( PinNumAcceptable(pMotor->DMin))
      pMotor->ReadMin = (pMotor->ReadMin *3 + (STOPPER_UNTOUCH != digitalRead( pMotor->DMin))*100)/4;
    TMin = pMotor->ReadMin > 50;
    if (TMin != pMotor->LastMin) {
      pMotor->LastMin = TMin;
      dbgprintf( 2, "touch min %i %i\n", pMotor->MotNum, pMotor->LastMin);
    }

    if (TMin && pMotor->HomingOn) { // fin de homing
      dbgprintf( 2, "end homing %i %i\n", pMotor->MotNum, pMotor->LastMin);
      pMotor->HomingOn = 0;
      pMotor->LastPos = pMotor->P2 = pMotor->P1 = 0;
      pMotor->WishDec = 10; // move a little away from stopper
      pr_int32_write( pMotor->WishP2, 0);
      pMotor->Order = 0;
    }
  #endif /* WITH_STOPPER */
  #ifdef WITH_STOPPER
    // TODO_HERE
    //if (PinNumAcceptable( DMax) && STOPPER_UNTOUCH != digitalRead( DMax)) {
    //  dbgprintf( 2, "touch max %i %i\n", MotNum, DMax);
    //  Res = 1;
    //}
  #endif /* WITH_STOPPER */
  if (Dist < 0) {
    if (1 == pMotor->StepperD) {
      pMotor->StepperD = 0;
      #ifdef WITH_INVERT1
        if(1 == pMotor->MotNum) {
          digitalWrite( DirPin, !pMotor->StepperD);
        } else {
          digitalWrite( DirPin, pMotor->StepperD);
        }
      #else /* WITH_INVERT1 */
        digitalWrite( DirPin, pMotor->StepperD);
      #endif /* WITH_INVERT1 */
      // dir <- StepperD
    } else {
      StepperPos --;
      pMotor->StepperP = !pMotor->StepperP;
      // puls <- StepperP
      if (!TMin)
        digitalWrite( PulsPin, pMotor->StepperP);
    }
  } else if (Dist > 0) {
    if (0 == pMotor->StepperD) {
      pMotor->StepperD = 1;
      // dir <- StepperD
      #ifdef WITH_INVERT1
        if(1 == pMotor->MotNum) {
          digitalWrite( DirPin, !pMotor->StepperD);
        } else {
          digitalWrite( DirPin, pMotor->StepperD);
        }
      #else /* WITH_INVERT1 */
        digitalWrite( DirPin, pMotor->StepperD);
      #endif /* WITH_INVERT1 */
    } else { // 0 == dist
        StepperPos ++;
        pMotor->StepperP = !pMotor->StepperP;
      // puls <- StepperP
      digitalWrite( PulsPin, pMotor->StepperP);
    }
  }
  pMotor->LastPos = StepperPos;

  return ( StepperPos);
}

#endif /* WITH_STEPPER */

// WITH_MOTOR ----------------------------------------------------------
#ifdef WITH_MOTOR

void SetFreq( int pin, int divisor) {
#ifdef ESP32
// TODO_HERE
#elif defined( ESP8266)
// change the frequency for MotorDcPwm, https://github.com/esp8266/Arduino/issues/2592.
  analogWriteFreq( 200);
  //analogWriteFreq( 32768/4 );
#else
// https://arduino.blaisepascal.fr/modifier-la-frequence-pwm/
   byte mode;
   if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
      switch(divisor) {
         case 1: mode = 0x01; break;
         case 8: mode = 0x02; break;
         case 64: mode = 0x03; break;
         case 256: mode = 0x04; break;
         case 1024: mode = 0x05; break;
         default: return;
      }
      if(pin == 5 || pin == 6) {
         TCCR0B = TCCR0B & 0b11111000 | mode;
      } else {
         TCCR1B = TCCR1B & 0b11111000 | mode;
      }
   } else if(pin == 3 || pin == 11) {
      switch(divisor) {
         case 1: mode = 0x01; break;
         case 8: mode = 0x02; break;
         case 32: mode = 0x03; break;
         case 64: mode = 0x04; break;
         case 128: mode = 0x05; break;
         case 256: mode = 0x06; break;
         case 1024: mode = 0x7; break;
         default: return;
      }
      TCCR2B = TCCR2B & 0b11111000 | mode;
   }
#endif
}

void MotorSetup() {
  int Idx;
  int UserMode;
  Motor_t* pMotor;

  dbgprintf( 0, "#%s posA0 %i\n", __func__, Idx);

  memset( &(MotorArray[0]), 0, sizeof(MotorArray));
  
  for (Idx = 0; Idx < MOTOR_NB; Idx++) {

  dbgprintf( 0, "#%s posA1 %i\n", __func__, Idx);

    pMotor = &(MotorArray[Idx]);

    pMotor->MotNum = Idx; // useful to debug when we got just the pointer
    pr_uint32_write( pMotor->WishDTms, 50);// TODO_HERE : en param sauvegard
    pMotor->Order = -1;
    switch (Idx) {
      default : UserMode = 0;                 break;
    }
    if (0 != UserMode) {
      pMotor->DriverMode = UserMode;
    }
    #ifdef WITH_STOPPER
      pMotor->DMin = StopperPins[Idx*2]; // stopper min
      pMotor->DMax = StopperPins[1+Idx*2]; // stopper max

      if (PinNumAcceptable( pMotor->DMin))
        MyPinmode( pMotor->DMin, INPUT_PULLUP);
      if (PinNumAcceptable( pMotor->DMax))
        MyPinmode( pMotor->DMax, INPUT_PULLUP);
      pMotor->ReadMin = 0;
      pMotor->ReadMax = 0;
    #endif /* WITH_STOPPER */
  }

  pMotor->LastPulseUs = micros();

  #ifdef WITH_BRLESS
    BrushlessSetup( );
  #endif /* WITH_BRLESS */

  #ifdef WITH_STEPPER
    StepperSetup( );
  #endif /* WITH_STEPPER */

  #ifdef WITH_DCMOTOR
    DcMotorSetup();
  #endif /* WITH_DCMOTOR */

  #ifdef WITH_SERVO
    for ( uint8_t i = 0; i < ServoNb; i++) {
      ServoInit( i, ServoPins[i]);
    }
  #endif /* WITH_SERVO */

  #ifdef STEPPER_IN_PULSE_PIN
    StepperInSetup();
  #endif /* STEPPER_IN_PULSE_PIN */

  //MaxXGet();
  //SpeedXGet();
  //VectX = SpeedX;
  //MaxYGet();
  //SpeedYGet();
  //VectY = SpeedY;

}

/* Description: Set motor home(ing)
 * St- 1 start homing, 0 caller decided it is done
 */
void MotorHome( int MotNum, int St) {
  Motor_t* pMotor;
  
  dbgprintf( 2, "MotorHome( %i, %i)\n", MotNum, St);
  if ((MotNum >= 0) && (MotNum < MOTOR_NB)) {

    pMotor = &(MotorArray[MotNum]);
    if (St) {
      pMotor->HomingOn = 1;
      pMotor->HomingMs = millis();
      pMotor->WishDec = 0;
      pMotor->Order = 0;

      pMotor->LastPos = HOMING_MAX;
      pMotor->P2 = HOMING_MAX;
      pr_int32_write( pMotor->WishP2, 0);
      pr_uint32_write( pMotor->WishDTms, HOMING_TIME);
      pMotor->WishDec = 0;
      pMotor->Order = 1;

      pr_uint32_write( (pMotor->Pos), HOMING_MAX);

      // MotorSet( MotNum, 0);
      MotorSetTimed( MotNum, 0, HOMING_TIME);
    } // else quit homing mode done in loops
  }
}

// instruct motor X to go to pos Y in T milliseconds
void MotorSetTimed( int MotNum, int MotVal, int Dt) {
  
  // dbgprintf( 2, "MotorSet( %i, %i)\n", MotNum, MotVal);
  if ((MotNum >= 0) && (MotNum < MOTOR_NB)) {
    Motor_t* pMotor = &(MotorArray[MotNum]);

    if (Dt >0)
      pr_uint32_write( pMotor->WishDTms, Dt);
    //dbgprintf( 2, "MotorSet( %i, %i) %i\n", MotNum, MotVal, pMotor->DriverMode);
    switch( pMotor->DriverMode) {
      default :
      case MOTOR_PWM :
      case MOTOR_PWM2 :
      #ifdef WITH_SERVO
        case MOTOR_SERVO :
      #endif /* WITH_SERVO */
        pr_int32_write( pMotor->WishP2, MotVal);
        if (pMotor->Order < 0) // just booted
          pMotor->Order = 0;
        break;
      case MOTOR_STEPPER:
      case MODOR_DC_LR:
        {
                if (pMotor->Order < 0) { // don't know where we are after boot
                  pMotor->WishDec = MotVal;
                  pMotor->Order = 0;
                }
                pr_int32_write( pMotor->WishP2, MotVal);
        }
        break;
    }
    if(pMotor->Order < 3) { // suppose ++ atomic and no more than 2 collisions
      pMotor->Order++;
    }
  } else {
    dbgprintf( 2, "Warn - No Such Motor %i\n", MotNum);
  }
}

void MotorSet( int MotNum, int MotVal) {
  MotorSetTimed( MotNum, MotVal, 500);
}

// the released state of the sensors
/* Description: consider switches to stop motor if touched
 * returns: 0 no limit touched
 */
int MotorLimit( int MotNum, int DMin, int DMax) {
  int Res = 0;
  Motor_t* pMotor = &(MotorArray[MotNum]);
  if (pMotor->HomingOn && DiffTime( pMotor->HomingMs, millis()) > 20000) {
    // timeout homing procs
    pMotor->HomingOn = !pMotor->HomingOn;
    dbgprintf( 2, "timeout no switch no home for %i\n", MotNum);
  }

  return( Res);
}

void MotorLoop( int FromInterrupt) {
  int32_t k; // 0..KFACT
  uint32_t CurrPos;
  uint32_t ExpectedPos;
  uint32_t CurTime;
  uint32_t NextTime;
  uint32_t CurMicros;
  int Idx;
  Motor_t* pMotor;
#define KFACT 1000

  // T1M = k T1T2
  // P1M = k P1P2

  CurTime = millis();
  CurMicros = micros();

  for (Idx = 0; Idx < MOTOR_NB; Idx++) {
    pMotor = &(MotorArray[Idx]);

    CurrPos = pr_uint32_read( pMotor->Pos);
    // dbgprintf( 2, " CurrPos %u\n", CurrPos);

    switch( pMotor->DriverMode) {
      default : 
        break;
      case MODOR_DC_LR :
        #ifdef WITH_GRAYCODE_IN
          CurrPos = pr_uint32_read( PrGrayPos);
        #endif /* WITH_GRAYCODE_IN */
        break;
    }

    // return for the unsuitables from inside interrupts
    switch( pMotor->DriverMode) {
      default :
      case MOTOR_PWM2:
        // no use to cut the flow
        break;
      case MOTOR_PWM: // full of analog write probably interrupt driven ... unsuitable inside interrupts itself
      #ifdef WITH_SERVO
        case MOTOR_SERVO: // servo interrupt driven ... unsuitable to set inside interrupts itself
      #endif /* WITH_SERVO */
        if (FromInterrupt) {
          return;
        }
        break;
    }

    // returns if inside moratory time window
    if ( DiffTime( CurMicros, pMotor->LastPulseUs) < pMotor->MinCommandUs) {
      return;
    }

    //dbgprintf( 2, " Order:%5i\n", pMotor->Order);
    // transfert external orders in the vector
    if ( pMotor->Order == 5) { // order 5 set everything
      pMotor->LastPos = pMotor->P2 = pMotor->P1 = pr_int32_read( pMotor->WishP2) - pMotor->WishDec;
      pMotor->Order = 0;
    } else if ( pMotor->Order > 0) {
      pMotor->P1 = CurrPos;
      //dbgprintf( 2, " trorder P1 CP %u %u\n", pMotor->P1, CurrPos);
      pMotor->T1ms = CurTime;
      k = 50;
      while( (k > 0) && (pMotor->Order > 0)) { // some friend might flow with orders
        pMotor->Order = 0;
        k--;
        pMotor->P2 = pr_int32_read( pMotor->WishP2) - pMotor->WishDec;
        pMotor->T2ms = CurTime + pr_uint32_read( pMotor->WishDTms);
      }
      // dbgprintf( 2, " transfert order %i %u %u\n", pMotor->Order, pMotor->P1, pMotor->P2);
      //dbgprintf( 2, " transfert order %i %u\n", pMotor->Order, pMotor->P1);
    }

    // compute expected pos
    if ( CurTime > pMotor->T2ms) {
      //k = KFACT;
      k = KFACT + 1;
      ExpectedPos = pMotor->P2;
    } else if (CurTime < pMotor->T1ms) {
      //k = 0;
      k = -1;
      ExpectedPos = pMotor->P1;
    } else {
      int32_t DiffT;
      int32_t DiffK;
      DiffT = Vectorize( pMotor->T1ms, pMotor->T2ms);
      // dbgprintf( 2, " DiffT %i", DiffT);
      if (0 == DiffT) {
        k = KFACT;
      } else {
        DiffK = Vectorize( pMotor->T1ms, CurTime);
        //dbgprintf( 2, " DiffK %i", DiffK);
        k = (KFACT * DiffK) / DiffT;
      }
      //if (pMotor->P2 > pMotor->P1) {
      ExpectedPos = pMotor->P1 + k * Vectorize( pMotor->P1, pMotor->P2) / KFACT;
      //} else {
      //  ExpectedPos = pMotor->P1 - k*(pMotor->P1 - pMotor->P2)/KFACT;
      //}
    }
    // dbgprintf( 2, " k %i", k);
    // RunOtherTasks( 200); // for Dev

    // TODO_LATER : timed component and immediate component does not mix as expected, some work to do around
    if (0 == Idx) {
#ifdef WITH_STEPPER_IN
      ExpectedPos += pr_uint32_read( pr_StepperInPos);
#endif /* WITH_STEPPER_IN */
    }

    #ifdef WITH_STOPPER
      MotorLimit( Idx, StopperPins[2*Idx], StopperPins[2*Idx+1]);
    #endif /* WITH_STOPPER */

    CurrPos = ExpectedPos;
    switch( pMotor->DriverMode) {
      default : 
      case MOTOR_UNDEF:
        // hoops
        // dbgprintf( 2, "MotorLoop Hoops %i undef %i\n", Idx, pMotor->DriverMode); // miss in code, must init somewhere
        break;
      #ifdef WITH_BRLESS
        case MOTOR_BRLESS:
          CurrPos = BrushlessLoop( pMotor, ExpectedPos);
          break;
      #endif /* WITH_BRLESS */
      #ifdef WITH_STEPPER
        case MOTOR_STEPPER:
          CurrPos = StepperLoop( pMotor, ExpectedPos);
          break;
      #endif /* WITH_STEPPER */
      #ifdef WITH_DCMOTOR
        case MODOR_DC_LR :
          CurrPos = DcPosLoop( pMotor, CurrPos);
          break;
        case MOTOR_PWM:
          MotorDcPwm3( Idx, ExpectedPos);
          break;
        case MOTOR_PWM2:
          MotorDcPwm2( Idx, ExpectedPos);
          break;
      #endif /* WITH_DCMOTOR */
      #ifdef WITH_SERVO
        case MOTOR_SERVO:
          ServoPosLoop( Idx, ExpectedPos);
          break;
      #endif /* WITH_SERVO */
    }
    

    pMotor->LastPulseUs = CurMicros;
    pr_uint32_write( pMotor->Pos, CurrPos);
  } // end for each motor
}
#endif /* WITH_MOTOR */


void AutoActivated() {
  static int LastMs = 0;
  static int MyX = 0;
  static int32_t MyY = 0;
  static int VectX = 70;
  // orig rail 1000
  static int VectY = 80;
  int CurMs = millis();
  Motor_t* pMotor;
  
  if ( DiffTime(CurMs, LastMs) > 500) {
    LastMs = CurMs;
    
    pMotor = &(MotorArray[0]);
    MyX += VectX;
    pr_uint32_write( pMotor->WishDTms, 5000);
    pr_int32_write( pMotor->WishP2, MyX);
    pMotor->Order++;

    pMotor = &(MotorArray[1]);
    MyY += VectY;
    pr_uint32_write( pMotor->WishDTms, 5000);
    pr_int32_write( pMotor->WishP2, MyY);
    pMotor->Order++;

    if (MyX > 32000) {
      VectX = - VectX;
      MyX -= 1;
    }
    if (MyX < 0) {
      VectX = - VectX;
      MyX = 1;
    }
    // orig max rail 200000
    if (MyY > 2400) {
      VectY = - VectY;
      MyY -= 1;
    }
    if (MyY < 0) {
      VectY = - VectY;
      MyY = 1;
    }
  }
    
}

#define CmdLineDbgMax 40
unsigned char CmdLineDbgBuff[CmdLineDbgMax+1];
int CmdLineDbgLen = 0;
int CmdLineDbgInCmt = 0;

//------ Str string
#define StrDec 100000
#define StrNDec 5
/* @brief  get num (sign, integer, decimal in StrDecth part) from string
   @return number of char parsed
*/
int StrGetNum( unsigned char* szCommand, signed char* pSign, uint32_t* pNint, uint32_t* pDec) {
  int Found = 0;
  int Cnt = 0;
  int InVal = 0;
  int State = 0; // 0 first num, 1 Decimal
  unsigned char Ch;

  // dbgprintf( 2,"#%s( \"%s\")\n", __func__, szCommand);

  *pSign = 1;
  *pNint = 0;
  *pDec = 0;

  while( !Found)
  {
    Ch = szCommand[Cnt++];
    if( Ch >= '0' && Ch <= '9') {
      InVal = 1;
      if (0 == State)
        *pNint = *pNint*10+ Ch-'0';
      else if(State < StrNDec) {
        *pDec = *pDec*10+ Ch-'0';
        State++;
      }
    } else {
      switch(Ch) {
        case '.': State = 1; break;
        case 0: Cnt--; Found = 1; break;
        case ' ': // Space
          if (InVal)
            Found = 1;
        break;
        default: Cnt--; Found = 1; break;
      }
    }
  }
  while(State < StrNDec) {
    *pDec = *pDec*10;
    State++;
  }

  dbgprintf( 2,"#%s ret %i\n", __func__, Cnt);
  return(Cnt);
}

// @brief get first letter found
int StrGetLetter( unsigned char* szCommand, unsigned char* pCh) {
  int Found = 0;
  int Cnt = 0;
  int InVal = 0;
  unsigned char Ch;

  // dbgprintf( 2,"#%s( \"%s\")\n", __func__, szCommand);

  while( !Found)
  {
    Ch = szCommand[Cnt++];
    if( Ch >= 'A' && Ch <= 'Z') {
      Found = 1;
      InVal = 1;
      *pCh = Ch;
    } else if( Ch >= 'a' && Ch <= 'z') {
      Found = 1;
      InVal = 1;
      *pCh = Ch;
    } else {
      switch(Ch) {
        case 0: Cnt--; Found = 1; break;
        case ' ': // Space
          if (InVal)
            Found = 1;
          // else next
        break;
        default: Cnt--; Found = 1; break;
      }
    }
  }

  // dbgprintf( 2,"#%s ret %i\n", __func__, Cnt);

  return( Cnt);
}

// ----- Gcode like interg

#define Gcode_NBVALS 6
#define Gcode_VALX 1
#define Gcode_VALY 2
#define Gcode_VALZ 3
#define Gcode_VALT 4
#define Gcode_VALt 5

int32_t  GcodeVal[Gcode_NBVALS];
unsigned GcodeGot[Gcode_NBVALS] = {0};

// @brief get coord like after X in "G1 X30.3"
int GcodeGetCoordN(  unsigned char* szCommand, int CoordId) {
  uint32_t Cmd;
  uint32_t Dum;
  signed char Sign;
  int Len;

  // dbgprintf( 0,"%s( \"%s\", %i) PosB \n", __func__, szCommand, CoordId);

  Len = StrGetNum( szCommand, &Sign, &Cmd, &Dum);
  if (Len >= 0 && CoordId>=0 && CoordId<Gcode_NBVALS)
  {
    GcodeVal[ CoordId] = Sign*Cmd;
    GcodeGot[ CoordId] = 1;
  }
  return( Len);
}

// @brief parse a Gcode line G type ... after the G
void GcodeG( unsigned char* szCommand) {

  int Found = 0;
  int Len;
  uint32_t Cmd;
  uint32_t Dum;
  signed char Sign;
  unsigned char Ch;
  int Cnt = 0;

  // dbgprintf( 2,"#%s( \"%s\")\n", __func__, szCommand);
  //xx
  Len = StrGetNum( szCommand+Cnt, &Sign, &Cmd, &Dum);
  if (Len >= 0)
    Cnt += Len;

  // TODO_LATER Cmd 0 .. 1 ...

  // X Y Z (later U V W Mxxx)
  // Fxxx
  memset( &GcodeGot, 0, sizeof(GcodeGot));
  while(!Found) {
    Ch = 0;
    Len = StrGetLetter( szCommand+Cnt, &Ch);
    if (Len <= 0)
      Found = 1;
    else
    {
      Cnt += Len;
      switch( Ch){
        case 'F':
          Len = GcodeGetCoordN( szCommand+Cnt, 0);
          if (Len > 0)
             Cnt += Len;
          break;
        case 'X':
          Len = GcodeGetCoordN( szCommand+Cnt, Gcode_VALX);
          if (Len > 0)
             Cnt += Len;
          break;
        case 'Y':
          Len = GcodeGetCoordN( szCommand+Cnt, Gcode_VALY-AXE_MINE);
          if (Len > 0)
             Cnt += Len;
          break;
        case 'Z':
          Len = GcodeGetCoordN( szCommand+Cnt, Gcode_VALZ-AXE_MINE);
          if (Len > 0)
             Cnt += Len;
          break;
        case 'T': // time you got to reach position
          Len = GcodeGetCoordN( szCommand+Cnt, Gcode_VALT);
          if (Len > 0)
             Cnt += Len;
          break;
        case 't': // global time
          Len = GcodeGetCoordN( szCommand+Cnt, Gcode_VALt);
          if (Len > 0)
             Cnt += Len;
          break;
        case 'M' : // Mx yyyy # motor x goto yyy
          Len = StrGetNum( szCommand+Cnt, &Sign, &Cmd, &Dum);
          Dum = -1;
          if (Len >= 0) {
            Cnt += Len;
            Dum = Cmd;
          }
          if (AXE_MINE == Dum || AXE_MINE+1 == Dum || AXE_MINE+2 == Dum)
            Len = GcodeGetCoordN( szCommand+Cnt, Dum-AXE_MINE);
          
          if (Len > 0)
             Cnt += Len;
          break;
        default :
          Len = GcodeGetCoordN( szCommand+Cnt, -1); // eat me that number
          if (Len > 0)
             Cnt += Len;
      }
    }
  }

  // TODO_LATER to be more Gcode compliant, like speed and other axes synchro using others GcodeMotorId than 1 (loop X Y Z)
  
  if (GcodeMotorId >= 0 && GcodeMotorId<Gcode_NBVALS && GcodeGot[GcodeMotorId]) {
    Motor_t* pMotor;

    pMotor = &(MotorArray[0]);
    if (GcodeGot[Gcode_VALT]) {
      pr_uint32_write( pMotor->WishDTms, GcodeVal[Gcode_VALT]);
    } else {
      pr_uint32_write( pMotor->WishDTms, 5000);
    }
    pr_int32_write( pMotor->WishP2, GcodeVal[GcodeMotorId]);
    if (-1 == pMotor->Order) // TODO_LATER at boot it is -1 to relink the position on hot restart if no homing availlable
      pMotor->Order = 0;    
    pMotor->Order++;
    dbgprintf( 0, "# let's %i G to %i\n", 0, GcodeVal[Gcode_VALX]);
  }
  // TODO_HERE timing to be more Gcode compliant
  dbgprintf( 0, "OK\n");
}

// ----- command line
void CmdLineProcess( unsigned char* szCommandLine) {
  // dbgprintf( 2,"#%s( \"%s\")\n", __func__, szCommandLine);
  switch( szCommandLine[0])
  {
    case 'G':
      GcodeG( szCommandLine+1);
      break;
    case 'M': // 115?
      if (AXE_MINE < 3)
        dbgprintf( 1, "ok MACHINE:%s:%c:A%i\n", MACHINE_NAME, 'X'+AXE_MINE, AXE_MINE);
      else
        dbgprintf( 1, "ok MACHINE:%s:A%i\n", MACHINE_NAME, AXE_MINE);
    break;
  }
}

void CmdLineParse( unsigned char Ch) {
  if('#'== Ch) {
     CmdLineDbgInCmt = 1;
  } else if ('\n' == Ch) {
    CmdLineDbgBuff[CmdLineDbgLen] = 0;
    if (CmdLineDbgLen)
      CmdLineProcess( CmdLineDbgBuff);
      CmdLineDbgLen = 0;
      CmdLineDbgInCmt = 0;
  } else {
    if (!CmdLineDbgInCmt) {
      if (CmdLineDbgLen < CmdLineDbgMax)
        CmdLineDbgBuff[CmdLineDbgLen++] = Ch;
      else
        dbgprintf( 0, "#overflow in dbg command line - missing carriage return - ignored char\n");
    }
  }
}

// @brief drive command line periodics
void CmdLineLoop()
{
  int iTmp;
  unsigned char Ch;

  iTmp = 20; // maybe we were waiting ( wifi, deep sleep ...), treat full command in one loop
  do {
    Ch = dbgReadCh();
    if ((0 != Ch) && (255 != Ch)) {
      DbgLastActivityMillis = millis();
      CmdLineParse( Ch);
      RunOtherTasks( 20); // or get bad chars
      iTmp--;
    } else {
      iTmp = 0;
    }
  } while (iTmp > 0);
}

// ----- lowlevel link

/* @brief once in the lifecycle init some lines */
void setup() {
  // setup code here run once

  // DbgSetup
  Serial.begin(115200); // for debug
  Serial.setTimeout( 0);// ms, default 1000
  dbgprintf( 0, "# Hello %s\n", HARDWARE_NAME);
  
  #ifdef WITH_MOTOR
    MotorSetup();
  #endif /* WITH_MOTOR */

#ifdef WITH_WS2812
  ws2812_setup();
#endif /* WITH_WS2812 */

  dbgprintf( 0, "#Setup ends\n");
}

void loop() {
  // main code here, run repeatedly
  static int LoopCnt = 0;

  if (DiffTime( LastDispMs, millis()) > 4000) {
    int Idx;
        
    dbgprintf( 0, "#EspUsb=A%i", AXE_MINE);
    dbgprintf( 0, ",Loop=%i", LoopCnt);
    for( Idx = 0; Idx < MOTOR_NB; Idx++){
      Motor_t* pMotor;
      pMotor = &(MotorArray[Idx]);
      dbgprintf( 0, ",M%i=%i", Idx, pMotor->LastPos);      
    }
    dbgprintf( 0, "\n");
    LastDispMs = millis();
  }
  //dbgprintf( 0, "#Loop %i\n", LoopCnt);
  //AutoActivated();
  CmdLineLoop();
  
  #ifdef WITH_MOTOR
    MotorLoop(0);
  #endif /* WITH_MOTOR */


#ifdef WITH_WS2812
  NeopixelPrepare( 0, 0, PIX_MAX, 0, 0);
  ws2812_loop();
#endif /* WITH_WS2812 */

  LoopCnt++;
}
