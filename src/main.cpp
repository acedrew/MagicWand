#include <Arduino.h>
#include <main.h>
#include <FastLED.h>

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#define BUFFER_LENGTH 64

#define INTERRUPT_PIN 19
#define MPU_PWR_PIN 23

MPU6050 mpu;
bool dmpReady = false;
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus;    // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;  // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;   // count of all bytes currently in FIFO
uint8_t fifoBuffer[BUFFER_LENGTH];

// orientation/motion vars
Quaternion qt;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

#define LED_PIN     13
#define COLOR_ORDER GRB
#define CHIPSET     WS2812
#define NUM_LEDS    15

#define BRIGHTNESS  64
#define FRAMES_PER_SECOND 60

// COOLING: How much does the air cool as it rises?
// Less cooling = taller flames.  More cooling = shorter flames.
// Default 55, suggested range 20-100 
#define COOLING  55

// SPARKING: What chance (out of 255) is there that a new spark will be lit?
// Higher chance = more roaring fire.  Lower chance = more flickery fire.
// Default 120, suggested range 50-200.
#define SPARKING 120
CRGBPalette256 gPal;
bool gReverseDirection = false;
uint8_t currentMode = 1;
uint8_t touchStatus = 0;
int touchTime = 0;
uint8_t touchCounter = 1;

#define TOUCH_THRESHOLD 40

CRGB leds[NUM_LEDS];
void Fire2012WithPalette(uint8_t modulus)
{
// Array of temperature readings at each simulation cell
  static byte heat[NUM_LEDS];

  // Step 1.  Cool down every cell a little
    for( int i = 0; i < NUM_LEDS; i++) {
      heat[i] = qsub8( heat[i],  random8(0, (((COOLING + modulus) * 10) / NUM_LEDS) + 2));
    }
  
    // Step 2.  Heat from each cell drifts 'up' and diffuses a little
    for( int k= NUM_LEDS - 1; k >= 2; k--) {
      heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
    }
    
    // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
    if( random8() < SPARKING ) {
      int y = random8(7);
      heat[y] = qadd8( heat[y], (random8(160,255)) );
    }

    // Step 4.  Map from heat cells to LED colors
    for( int j = 0; j < NUM_LEDS; j++) {
      // Scale the heat value from 0-255 down to 0-240
      // for best results with color palettes.
      byte colorindex = scale8( heat[j], 240);
      CRGB color = ColorFromPalette( gPal, colorindex);
      int pixelnumber;
      if( gReverseDirection ) {
        pixelnumber = (NUM_LEDS-1) - j;
      } else {
        pixelnumber = j;
      }
      leds[pixelnumber] = color;
    }
}

void IRAM_ATTR touch14() {
  noInterrupts();
  if (touchStatus & 2){
    touchCounter++;
    touchStatus = 0;
  } else {
    touchTime = millis();
    touchStatus = touchStatus | 1; 
  }
  interrupts();
}
void IRAM_ATTR touch13() {
  noInterrupts();
    touchTime = millis();
    touchStatus = touchStatus & 2; 
  interrupts();
}
void IRAM_ATTR touch12() {
  noInterrupts();
  if (touchStatus & 1){
    touchCounter--;
    touchStatus = 0;
  } else {
    touchTime = millis();
    touchStatus = touchStatus | 2; 
  }
  interrupts();
}

void setup() {
  pinMode(0, INPUT_PULLUP);
  Serial.begin(115200);
    pinMode(MPU_PWR_PIN, OUTPUT);
  digitalWrite(MPU_PWR_PIN, LOW);
  delay(10);
  digitalWrite(MPU_PWR_PIN, HIGH);
  delay(10);

  Wire.begin();
  Wire.setClock(400000);

  mpu.initialize();
  // pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  if (devStatus == 0)
  {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.setFullScaleAccelRange(16);
    mpu.setRate(5);
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); 
    // mpu.CalibrateAccel(6);
    // mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    // Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness( BRIGHTNESS );
  gPal = RainbowColors_p;
  touchAttachInterrupt(T9, touch14, TOUCH_THRESHOLD);
  // touchAttachInterrupt(T4, touch13, 255);
  touchAttachInterrupt(T5, touch12, TOUCH_THRESHOLD + 10);

  // put your setup code here, to run once:
}

void loop() {
  // Serial.println(currentMode);
  if (touchTime + 300 < millis() & touchStatus > 0) {
    touchStatus = 0;
  } else if (touchStatus > 0) {
    Serial.print("touch detected ");
    Serial.print(touchStatus);
    Serial.print(" ");
    Serial.println(touchCounter);
  }
    // random16_add_entropy( random());

  // Fourth, the most sophisticated: this one sets up a new palette every
  // time through the loop, based on a hue that changes every time.
  // The palette is a gradient from black, to a dark color based on the hue,
  // to a light color based on the hue, to white.
  //
  //   static uint8_t hue = 0;
  //   hue++;
  //   CRGB darkcolor  = CHSV(hue,255,192); // pure hue, three-quarters brightness
  //   CRGB lightcolor = CHSV(hue,128,255); // half 'whitened', full brightness
  //   gPal = CRGBPalette16( CRGB::Black, darkcolor, lightcolor, CRGB::White);


    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
    mpu.dmpGetQuaternion(&qt, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &qt);
    // mpu.dmpGetYawPitchRoll(ypr, &qt, &gravity);
    // Serial.print("\n");
    // Serial.print(qt.w);
    // Serial.print(" ");
    // Serial.print(qt.x);
    // Serial.print(" ");
    // Serial.print(qt.y);
    // Serial.print(" ");
    // Serial.print(qt.z);
    // Serial.print("\t");
    // Serial.print(ypr[1] * 180 / PI);
    // Serial.print("\t");
    // Serial.print(ypr[2] * 180 / PI);
    // Serial.println("");
    }
  // Fire2012WithPalette((uint8_t)(qt.w * 127)); // run simulation frame, using palette colors
  
  if(digitalRead(0)) { 
    switch(currentMode) {
      case 0:
        for(int i=0;i<NUM_LEDS;i++){
          leds[i] = gPal[i+((uint8_t)(abs(qt.w*255)))];
        }
        break;
      case 1:
        for(int i=0;i<NUM_LEDS;i++){
          leds[i] = gPal[(i*touchCounter)+((uint8_t)(abs(qt.w*127)))];
        }
        break;
      case 2:
       Fire2012WithPalette((uint8_t)(abs(qt.w*255)));
       break;
      default:
        for(int i=0;i<NUM_LEDS;i++){
          leds[i] = gPal[i+((uint8_t)(abs(qt.w*127)))];
        }
        break;
    }
        

  } else {
    currentMode = (uint8_t)(abs(qt.w*15));
    for(int i=0;i<NUM_LEDS;i++){
      if (i < currentMode) {
        leds[i] = gPal[i+((uint8_t)(abs(qt.w*127)))];
      } else {
        leds[i] = 0;
      }
    }
  }
    FastLED.show(); // display this frame
    FastLED.delay(1000 / FRAMES_PER_SECOND);

  // put your main code here, to run repeatedly:
}