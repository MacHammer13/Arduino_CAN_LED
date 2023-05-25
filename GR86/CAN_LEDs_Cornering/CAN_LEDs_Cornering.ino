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
#define BRIGHT_MAX 50

#define ENG_MAX 7400
#define ENG_MIN 1000
#define STOP_TIMER 5000
#define SPD_MAX 140
#define SPD_MIN 1
#define LAT_MAX 1

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
const unsigned int A = 0, B = 1, C = 2, D = 3, E = 4, F = 5, G = 6, H = 7, g = 9.81;

// create variables for brightness calculation and time
unsigned int brightness = 0, led, t = 0, t_buf = 0, t_stop = 0, lat_scale;

// create RGB color triplets
const unsigned int Green[3] = {0, 255, 0}, Red[3] = {255, 0, 0}, Blue[3] = {0, 0, 255};
const unsigned int Yellow[3] = {255, 255, 0}, Pink[3] = {255, 0, 255}, Cyan[3] = {0, 255, 255};
const unsigned int Orange[3] = {255, 80, 0}, Purple[3] = {110, 0, 255}, White[3] = {150, 150, 150}, Off[3] = {0, 0, 0};

// CAN signals to be calculated
unsigned int Gear, Gear_Buf, Accel_Pos, Eng_Spd, Eng_Spd_Buf, Veh_Spd, Brake_Pos, Dash_Bright, mode;
bool F_Clutch, F_Brake, F_Accel, F_DrivDoor, F_PassDoor;
float Steer_Ang, Tire_Ang, Yaw_Rate, Lng_Acc, Lat_Acc;

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
  color_led(Off);

  // Initialize CAN and blink red if it fails
  if (CAN0.begin(CAN_500KBPS) != CAN_OK) {
    brightness = BRIGHT_MAX;
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
  brightness = BRIGHT_MAX;
  for (int i = 0; i < 3; i++) {
    color_led(Green);
    delay(500);
    color_led(Off);
    delay(500);
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
    calc_signals();
    if (F_DrivDoor || F_PassDoor) {
      brightness = BRIGHT_MAX;
      if (F_PassDoor) {
        for (int i = 0; i < NUM_LEDS / 2; i++) {
          led_array[i].setRGB(White[0], White[1], White[2]);
        }
      }
      if (F_DrivDoor) {
        for (int i = NUM_LEDS / 2; i < NUM_LEDS; i++) {
          led_array[i].setRGB(White[0], White[1], White[2]);
        }
      }
      FastLED.setBrightness(brightness);
      FastLED.show();
    } else {
      switch (Dash_Bright) {                       // Determine mode based on dash brightness switch
        case 0x12:
          color_led(Off);
          mode = 1;
          break;
        case 0x1E:
          color_led(Off);
          mode = 2;
          break;
        case 0x35:
          color_led(Off);
          mode = 3;
          break;
        case 0x5A:
          power_led_corner();                     // steering angle, lateral acceleration
          mode = 4;
          break;
        case 0x96:
          power_led_eng();                        // engine speed, brakes, and gear
          mode = 5;
          break;
        case 0xFA:
          color_led(Off);
          mode = 6;
          break;
        default:
          color_led(Off);
          break;
      }
    }
    /*Serial.print(mode,DEC);
    Serial.print("\t");
    Serial.print(Steer_Ang,DEC);
    Serial.print("\t");
    Serial.print(Lat_Acc,DEC);
    Serial.print("\t");
    Serial.print(Lng_Acc,DEC);
    Serial.println("");*/
  }
}

/* ===============================================================================
                                 Functions
   =============================================================================*/
// Calculate signals from CAN message and set new filters
void calc_signals() {
  switch (ID) {
    case 0x40:
      Eng_Spd_Buf = uint16_t(buf[D] << 8 | buf[C]) & 0x3FFF;
      if (Eng_Spd_Buf <= ENG_MAX)
        Eng_Spd = Eng_Spd_Buf;
      Accel_Pos = buf[E] / 2.55;
      F_Accel = (buf[H] & 0xC0) != 0xC0;
      if (mode == 4)
        CAN0.init_Filt(0, 0, 0x138);
      else if (mode == 5)
        CAN0.init_Filt(0, 0, 0x139);
      break;
    case 0x138:
      Steer_Ang = int16_t(buf[D] << 8 | buf[C]) * -0.1;
      Yaw_Rate = int16_t(buf[F] << 8 | buf[E]) * -0.2725;
      if (mode == 4)
        CAN0.init_Filt(0, 0, 0x139);
      else if (mode == 5)
        CAN0.init_Filt(0, 0, 0x139);
      break;
    case 0x139:
      Veh_Spd = (uint16_t(buf[D] << 8 | buf[C]) & 0x1FFF) * 0.015694;
      F_Brake = (buf[E] & 0x4) == 4;
      Brake_Pos = min(buf[F] / 0.7, 100);
      if (mode == 4)
        CAN0.init_Filt(0, 0, 0x13B);
      else if (mode == 5)
        CAN0.init_Filt(0, 0, 0x241);
      break;
    case 0x13B:
      Lat_Acc = (int8_t(buf[G]) * 0.2 + 0.2) / g;
      Lng_Acc = (int8_t(buf[H]) * -0.1 - 0.1) / g;
      if (mode == 4)
        CAN0.init_Filt(0, 0, 0x390);
      else if (mode == 5)
        CAN0.init_Filt(0, 0, 0x241);
      break;
    case 0x241:
      Gear_Buf = buf[E] >> 3 & 0x7;
      F_Clutch = ((buf[F] & 0x80) / 1.28) == 100;
      if ((Gear_Buf != 0) || (Eng_Spd < ENG_MIN))
        Gear = Gear_Buf;
      CAN0.init_Filt(0, 0, 0x390);
      break;
    case 0x390:
      Dash_Bright = buf[F];
      CAN0.init_Filt(0, 0, 0x3AC);
      break;
    case 0x3AC:
      F_DrivDoor = buf[E] & 0x1;
      F_PassDoor = buf[E] & 0x2;
      if (mode == 4)
        CAN0.init_Filt(0, 0, 0x138);
      else if (mode == 5)
        CAN0.init_Filt(0, 0, 0x040);
      break;
  }
}

/* =============================================================================*/
// Logic for determining how to color leds in engine mode
void power_led_eng() {
  if (F_Brake) {                 // ON BRAKES
    if (Veh_Spd == 0) {           // VEHICLE STOPPED
      t_stop = t_stop + (t - t_buf);    // Start counting
      if (t_stop > STOP_TIMER) {          // STOPPED FOR COUNTER
        brightness = constrain(map(Brake_Pos, 0, 100, 0, BRIGHT_MAX), 0, BRIGHT_MAX);
        stop_dance(Red);                  // Run stop dance
      }
      else {                              // TIMER NOT ELAPSED
        brightness = constrain(map(Brake_Pos, 0, 100, 0, BRIGHT_MAX), 0, BRIGHT_MAX);           // Solid brake
        color_led(Red);                   //
      }
    }
    else {                            // VEHICLE MOVING
      t_stop = 0;                     // Reset timer
      brightness = constrain(map(Brake_Pos, 0, 100, 0, BRIGHT_MAX), 0, BRIGHT_MAX);         // Solid brake
      color_led(Red);                 //
    }
  }
  else if (!F_Brake) {         // OFF BRAKES
    t_stop = 0;                     // Reset timer
    if (F_Clutch) {              // ON CLUTCH
      if (F_Accel) {               // ON ACCEL
        color_eng_gear();               // Color based on gear and engine speed
      }
      else if (!F_Accel) {         // OFF ACCEL
        if (Eng_Spd > ENG_MIN)    // HIGH ENGINE SPEED
          color_eng_gear();               // Color based on gear and engine speed
        else                              // LOW ENGINE SPEED
          color_led(Off);                 // Turn lights off
      }
    }
    else if (!F_Clutch) {        // OFF CLUTCH
      if (Eng_Spd > ENG_MAX - 400) {
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
// Logic for determining how to color leds in cornering mode
void power_led_corner() {
  if (abs(Lat_Acc) < 0.1) {
    if (Veh_Spd == 0) {
      stop_dance(Red);
    } else {
      brightness = constrain(map(Veh_Spd, SPD_MIN, SPD_MAX, 0, BRIGHT_MAX), 0, BRIGHT_MAX);
      color_led(Red);
    }
  } else {
    brightness = constrain(map(Veh_Spd, SPD_MIN, SPD_MAX, 0, BRIGHT_MAX), 0, BRIGHT_MAX);
    lat_scale = constrain(map(abs(Lat_Acc), 0, 1, 0, NUM_LEDS - 1), 0, NUM_LEDS - 1);
  }
}

/* =============================================================================*/
// Change light color and brightness based on engine speed and gear
void color_eng_gear() {
  brightness = constrain(map(Eng_Spd, ENG_MIN, ENG_MAX, 0, BRIGHT_MAX), 0, BRIGHT_MAX);
  switch (Gear) {
    case 0:
      color_led(Off);
      break;
    case 1:
      color_led(Green);
      break;
    case 2:
      color_led(Purple);
      break;
    case 3:
      color_led(Orange);
      break;
    case 4:
      color_led(Pink);
      break;
    case 5:
      color_led(Blue);
      break;
    case 6:
      color_led(White);
      break;
  }
}

/* =============================================================================*/
// Color the LEDs based on desired color and brightness
void color_led(unsigned int color[3]) {
  FastLED.setBrightness(brightness);
  fill_solid(led_array, NUM_LEDS, CRGB(color[0], color[1], color[2]));
  FastLED.show();
}

/* =============================================================================*/
// Color the LEDs based on desired color and brightness
void color_led_corner(unsigned int color[3]) {
  FastLED.setBrightness(brightness);              // Set brightness
  for (int i = 0; i < NUM_LEDS; i++) {            // Loop LED
    if (Lat_Acc < 0) {                            // Left to right
      if (i < (NUM_LEDS / 2)) {                   // Determine LED position based on left to right
        led = NUM_LEDS - i - 1;
      }
      else {
        led = i - (NUM_LEDS / 2);
      }
      if (led >= lat_scale)                       // Turn off led if less than scale
        led_array[led] = CRGB(color[0], color[1], color[2]);
      else
        led_array[led] = CRGB(Off[0], Off[1], Off[2]);
    }
    else   {
      if (i < (NUM_LEDS / 2)) {
        led = (NUM_LEDS / 2) - i - 1;
      }
      else {
        led = i;
      }
      if (led <= lat_scale)
        led_array[led] = CRGB(color[0], color[1], color[2]);
      else
        led_array[led] = CRGB(Off[0], Off[1], Off[2]);
    }
  }
  FastLED.show();
}

/* =============================================================================*/
// Make the lights dance when stopped for more than x seconds
void stop_dance(unsigned int color[3]) {
  fadeToBlackBy(led_array, NUM_LEDS, 20);
  int pos = beatsin16(13, 0, NUM_LEDS - 1);
  led_array[pos] += CRGB(color[0], color[1], color[2]);
  FastLED.show();
}
