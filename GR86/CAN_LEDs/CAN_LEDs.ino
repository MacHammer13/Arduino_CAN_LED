/* ===============================================================================
                                 Initialize
   =============================================================================*/
// Include libraries for LEDs and CAN
#include <FastLED.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>

// Define constants
#define CANint 2
#define NUM_LEDS 288
#define LED_PIN 3
#define BRIGHTNESS 60

#define ENGINE_MAX 7400
#define ENGINE_MIN 1000
#define STOP_TIMER 2000

CRGB led_array[NUM_LEDS]; // Create LED Array
MCP_CAN CAN0(9);          // Set CS to pin 9

/* ===============================================================================
                                 Variables
   =============================================================================*/
// Initialize all variables

// create variables for CAN Message and signal calculation
unsigned char len = 0;
unsigned char buf[8];
unsigned long ID = 0;
unsigned int A = 0, B = 1, C = 2, D = 3, E = 4, F = 5, G = 6, H = 7;

// create variables for brightness calculation and time
unsigned int scal = 100, t = 0, t_buf = 0, t_stop = 0;

// create RGB color triplets
unsigned int Green[3] = {0, 255, 0}, Red[3] = {255, 0, 0}, Blue[3] = {0, 0, 255};
unsigned int Yellow[3] = {255, 255, 0}, Pink[3] = {255, 0, 255}, Cyan[3] = {0, 255, 255};
unsigned int Orange[3] = {255, 80, 0}, Purple[3] = {110, 0, 255}, White[3] = {150, 150, 150}, Off[3] = {0, 0, 0};

// CAN signals to be calculated
unsigned int Gear, Gear_Buf, Accel_Pos, Engine_Speed, Vehicle_Speed, Brake_Pos;
bool Clutch_Switch, Brake_Switch, Accel_Switch, Sport_Switch, Reset_Switch;

/* ===============================================================================
                                 Setup
   =============================================================================*/
// Main setup function
void setup() {
  
  // Initialize Serial
  Serial.begin(9600);
  while (!Serial) {
    Serial.print("I will wait here forever...");
    delay(1000);
  };

  // Initialize LEDs
  FastLED.addLeds<NEOPIXEL, LED_PIN>(led_array, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  color_led(Off);

  // Initialize CAN and blink red if it fails
  if (CAN0.begin(CAN_500KBPS) != CAN_OK) {
    scal = 100;
    while (1) {
      color_led(Red);
      delay(1000);
      color_led(Off);
      delay(1000);
    }
  }
  pinMode(CANint, INPUT);       // CAN interupt pin
  CAN0.init_Mask(0, 0, 0xFFF);  // Initialize CAN Mask

  // Signal good to go!
  for (int i = 0; i < NUM_LEDS; i++) {
    led_array[i].setRGB(255, 255, 255);
    FastLED.show();
    delay(1 / NUM_LEDS * 1000);
  }
  for (int i = 0; i < NUM_LEDS; i++) {
    led_array[i].setRGB(0, 0, 0);
    FastLED.show();
    delay(1 / NUM_LEDS * 1000);
  }
}

/* ===============================================================================
                                 Loop
   =============================================================================*/
// Main function loop
void loop() {

  // get time and keep last time
  t_buf = t;
  t = millis();

  if (CAN_MSGAVAIL == CAN0.checkReceive()) {  // Check to see whether data is read
    CAN0.readMsgBufID(&ID, &len, buf);        // Read data
    calc_signals();                           // calculate new signals
  }

  if (Sport_Switch)                           // only power LEDs in sport mode
    power_led();                              // determine color and brightness then power LEDs
  else
    color_led(Off);                           // turn off if not in sport mode
}

/* ===============================================================================
                                 Functions
   =============================================================================*/
// Calculate known signals based on ID and equation
void calc_signals() {
  switch (ID) {
    case 0x40:
      Engine_Speed = (buf[D] << 8 | buf[C]) & 0x3FFF;
      Accel_Pos = buf[E] / 2.55;
      Accel_Switch = (buf[H] & 0xC0) != 0xC0;
      CAN0.init_Filt(0, 0, 0x139);
      break;
    case 0x139:
      Vehicle_Speed = ((buf[D] << 8 | buf[C]) & 0x1FFF) * 0.015694;
      Brake_Switch = (buf[E] & 0x4) == 4;
      Brake_Pos = min(buf[F] / 0.7, 100);
      CAN0.init_Filt(0, 0, 0x241);
      break;
    case 0x241:
      Gear_Buf = buf[E] >> 3 & 0x7;
      Clutch_Switch = ((buf[F] & 0x80) / 1.28) == 100;
      if ((Gear_Buf != 0) || (Engine_Speed < ENGINE_MIN))
        Gear = Gear_Buf;
      if (!(t % 1000))
        CAN0.init_Filt(0, 0, 0x328);
      else
        CAN0.init_Filt(0, 0, 0x040);
      break;
    case 0x328:
      Sport_Switch = buf[E] & 0x1;
      CAN0.init_Filt(0, 0, 0x040);
  }
}

/* =============================================================================*/
// Logic for determining how to color leds
void power_led() {
  if (Brake_Switch) {                 // ON BRAKES
    if (Vehicle_Speed == 0) {           // VEHICLE STOPPED
      t_stop = t_stop + (t - t_buf);    // Start counting
      if (t_stop > STOP_TIMER) {          // STOPPED FOR COUNTER
        scal = 100;                       // Run stop dance
        stop_dance(Red);                  //
      }
      else {                              // TIMER NOT ELAPSED
        scal = Brake_Pos;                 // Solid brake
        color_led(Red);                   //
      }
    }
    else {                            // VEHICLE MOVING
      t_stop = 0;                     // Reset timer
      scal = Brake_Pos;               // Solid brake
      color_led(Red);                 //
    }
  }
  else if (!Brake_Switch) {         // OFF BRAKES
    t_stop = 0;                     // Reset timer
    if (Clutch_Switch) {              // ON CLUTCH
      if (Accel_Switch) {               // ON ACCEL
        color_eng_gear();               // Color based on gear and engine speed
      }
      else if (!Accel_Switch) {         // OFF ACCEL
        if (Engine_Speed > ENGINE_MIN)    // HIGH ENGINE SPEED
          color_eng_gear();               // Color based on gear and engine speed
        else                              // LOW ENGINE SPEED
          color_led(Off);                 // Turn lights off
      }
    }
    else if (!Clutch_Switch) {        // OFF CLUTCH
      if (Engine_Speed > ENGINE_MAX - 400) {
        if (!(t % 2))                   // CLOSE TO REV LIMIT
          color_led(Off);               // Flash
        else                            //
          color_eng_gear();             //
      }
      else                              // NOT CLOSE TO REV LIMIT
        color_eng_gear();               // Color based on gear and engine speed
    }
  }
}

/* =============================================================================*/
// Change light color and brightness based on engine speed and gear
void color_eng_gear() {
  scal = map(Engine_Speed, ENGINE_MIN, ENGINE_MAX, 0, 100);
  switch (Gear) {
    case 0:
      color_led(Off);
      break;
    case 1:
      color_led(Green);
      break;
    case 2:
      color_led(Blue);
      break;
    case 3:
      color_led(Purple);
      break;
    case 4:
      color_led(Pink);
      break;
    case 5:
      color_led(Orange);
      break;
    case 6:
      color_led(White);
      break;
  }
}

/* =============================================================================*/
// Color the LEDs based on desired color and brightness
void color_led(unsigned int color[3]) {
  fill_solid(led_array, NUM_LEDS, CRGB(color[0]*scal / 100, color[1]*scal / 100, color[2]*scal / 100));
  FastLED.show();
}

/* =============================================================================*/
// Make the lights dance when stopped for more than 2 seconds
void stop_dance(unsigned int color[3]) {
  fadeToBlackBy(led_array, NUM_LEDS, 20);
  int pos = beatsin16(13, 0, NUM_LEDS-1);
  led_array[pos] += CRGB(color[0] * scal / 100, color[1] * scal / 100, color[2] * scal / 100);
  FastLED.show();
}
