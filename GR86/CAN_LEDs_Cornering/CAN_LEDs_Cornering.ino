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
#define BRIGHT_MIN 1
#define BRIGHT_MAX 50
#define START_TIMER 5000
#define STOP_TIMER 3000

#define ENG_MAX 7500
#define ENG_MIN 1000
#define ENG_OFF 700
#define SPD_MAX 140
#define SPD_MIN 1
#define LAT_MAX 1
#define RATIO_MIN 10
#define RATIO_MAX 150

CRGB led_array[NUM_LEDS]; // Create LED Array
MCP_CAN CAN0(9);          // Set CS to pin 9

/* ===============================================================================
                                 Variables
   =============================================================================*/
// Initialize all variables

// create variables for CAN Message and signal calculation
unsigned char len = 0, buf[8];
unsigned long ID = 0;
const unsigned int A = 0, B = 1, C = 2, D = 3, E = 4, F = 5, G = 6, H = 7, g = 9.81;

// create variables for brightness calculation and time
unsigned int brightness = 0, t = 0, t_buf = 0, t_stop = 0, t_start = 0, lat_scale = 0, hue = 0;

// create RGB color triplets
const unsigned int Green[3] = {0, 255, 0}, Red[3] = {255, 0, 0}, Blue[3] = {0, 0, 255};
const unsigned int Yellow[3] = {255, 255, 0}, Pink[3] = {255, 0, 255}, Cyan[3] = {0, 255, 255};
const unsigned int Orange[3] = {255, 80, 0}, Purple[3] = {110, 0, 255}, White[3] = {150, 150, 150}, Off[3] = {0, 0, 0};
int Gears[8][2] = {{27, 360},
  {35, 300},
  {42, 240},
  {53, 180},
  {76, 120},
  {125, 60},
  {500, 30},
  {2000, 1}
};
int ids[5], id = 0;

// CAN signals to be calculated
unsigned int Gear, Gear_Buf, Accel_Pos, Eng_Spd, Eng_Spd_Buf, Brake_Pos, Dash_Bright, mode;
bool F_Clutch, F_Brake, F_Accel, F_DrivDoor, F_PassDoor, F_Light;
float Steer_Ang, Tire_Ang, Yaw_Rate, Lng_Acc, Lat_Acc, Gear_Ratio, Veh_Spd, ratio;

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
  while (t_start < START_TIMER) {
    FastLED.setBrightness(brightness);
    juggle();
    t_buf = t;
    t = millis();
    t_start += t - t_buf;
  }
  color_led(Off);
  brightness = 0;
  t = 0;
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
      ids[0] = 0x390, ids[1] = 0x3AC, ids[2] = 0x0, ids[3] = 0x0, ids[4] = 0x0;
    }
    else if (!F_Light) {                            // Only activate when its dark
      switch (Dash_Bright) {                       // Determine mode based on dash brightness switch
        case 0x12:
          color_led(Off);
          mode = 1;
          ids[0] = 0x390, ids[1] = 0x3AC, ids[2] = 0x0, ids[3] = 0x0, ids[4] = 0x0;
          break;
        case 0x1E:
          color_led(Off);
          mode = 2;
          ids[0] = 0x390, ids[1] = 0x3AC, ids[2] = 0x0, ids[3] = 0x0, ids[4] = 0x0;
          break;
        case 0x35:
          color_led(Off);
          mode = 3;
          ids[0] = 0x390, ids[1] = 0x3AC, ids[2] = 0x0, ids[3] = 0x0, ids[4] = 0x0;
          break;
        case 0x5A:
          power_led_throttle();
          mode = 4;
          ids[0] = 0x40, ids[1] = 0x139, ids[2] = 0x390, ids[3] = 0x3AC, ids[4] = 0x0;
          break;
        case 0x96:
          power_led_corner();                     // steering angle, lateral acceleration, and vehicle speed
          mode = 5;
          ids[0] = 0x139, ids[1] = 0x13B, ids[2] = 0x390, ids[3] = 0x3AC, ids[4] = 0x0;
          break;
        case 0xFA:
          power_led_eng();                        // engine speed, brakes, and gear
          mode = 6;
          ids[0] = 0x40, ids[1] = 0x139, ids[2] = 0x390, ids[3] = 0x3AC, ids[4] = 0x0;
          break;
        default:
          color_led(Off);
          ids[0] = 0x390, ids[1] = 0x3AC, ids[2] = 0x0, ids[3] = 0x0, ids[4] = 0x0;
          break;
      }
    }
    else {
      brightness = 10;
      color_led(Red);
    }
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
      /*if (mode == 4)
        CAN0.init_Filt(0, 0, 0x139);
      else if (mode == 5)
        CAN0.init_Filt(0, 0, 0x138);
      else if (mode == 6)
        CAN0.init_Filt(0, 0, 0x139);
      else
        CAN0.init_Filt(0, 0, 0x390);*/
      break;
    case 0x138:
      Steer_Ang = int16_t(buf[D] << 8 | buf[C]) * -0.1;
      Yaw_Rate = int16_t(buf[F] << 8 | buf[E]) * -0.2725;
      /*if (mode == 4)
        CAN0.init_Filt(0, 0, 0x139);
      else if (mode == 5)
        CAN0.init_Filt(0, 0, 0x139);
      else if (mode == 6)
        CAN0.init_Filt(0, 0, 0x139);
      else
        CAN0.init_Filt(0, 0, 0x390);*/
      break;
    case 0x139:
      Veh_Spd = (uint16_t(buf[D] << 8 | buf[C]) & 0x1FFF) * 0.05625; // 0.015694;
      F_Brake = (buf[E] & 0x4) == 4;
      Brake_Pos = min(buf[F] / 0.7, 100);
      /*if (mode == 4)
        CAN0.init_Filt(0, 0, 0x390);
      else if (mode == 5)
        CAN0.init_Filt(0, 0, 0x13B);
      else if (mode == 6)
        CAN0.init_Filt(0, 0, 0x390);
      else
        CAN0.init_Filt(0, 0, 0x390);*/
      break;
    case 0x13B:
      Lat_Acc = (int8_t(buf[G]) * -0.1) / g;
      Lng_Acc = (int8_t(buf[H]) * -0.1) / g;
      /*if (mode == 4)
        CAN0.init_Filt(0, 0, 0x390);
      else if (mode == 5)
        CAN0.init_Filt(0, 0, 0x390);
      else if (mode == 6)
        CAN0.init_Filt(0, 0, 0x390);
      else
        CAN0.init_Filt(0, 0, 0x390);*/
      break;
    case 0x241:
      Gear_Buf = buf[E] >> 3 & 0x7;
      F_Clutch = ((buf[F] & 0x80) / 1.28) == 100;
      if ((Gear_Buf != 0) || (Eng_Spd < ENG_MIN))
        Gear = Gear_Buf;
      /*CAN0.init_Filt(0, 0, 0x390);*/
      break;
    case 0x390:
      Dash_Bright = buf[F];
      F_Light = buf[G] & 0x10;
      /*CAN0.init_Filt(0, 0, 0x3AC);*/
      break;
    case 0x3AC:
      F_DrivDoor = buf[E] & 0x1;
      F_PassDoor = buf[E] & 0x2;
      /*if (mode == 4)
        CAN0.init_Filt(0, 0, 0x040);
      else if (mode == 5)
        CAN0.init_Filt(0, 0, 0x138);
      else if (mode == 6)
        CAN0.init_Filt(0, 0, 0x040);
      else
        CAN0.init_Filt(0, 0, 0x390);*/
      break;
  }
  if (ID == ids[id]) {
    id = id + 1;
    if (ids[id] == 0x0)
      id = 0;
    else
      CAN0.init_Filt(0, 0, ids[id]);
  }
}

/* =============================================================================*/
// Logic for engine mode
void power_led_eng() {
  if (F_Brake) {                 // ON BRAKES
    if (Veh_Spd == 0) {           // VEHICLE STOPPED
      t_stop = t_stop + (t - t_buf);    // Start counting
      if (t_stop > STOP_TIMER) {          // STOPPED FOR COUNTER
        brightness = BRIGHT_MAX;
        color_led(Off);
        FastLED.setBrightness(brightness);
        stop_dance(Red);                  // Run stop dance
      }
      else {                              // TIMER NOT ELAPSED
        brightness = constrain(map(Brake_Pos, 0, 100, BRIGHT_MIN, BRIGHT_MAX), BRIGHT_MIN, BRIGHT_MAX);           // Solid brake
        color_led(Red);                   //
      }
    }
    else {                            // VEHICLE MOVING
      t_stop = 0;                     // Reset timer
      brightness = constrain(map(Brake_Pos, 0, 100, BRIGHT_MIN, BRIGHT_MAX), BRIGHT_MIN, BRIGHT_MAX);         // Solid brake
      color_led(Red);                 //
    }
  }
  else if (!F_Brake) {         // OFF BRAKES
    t_stop = 0;                     // Reset timer
    if (Eng_Spd > ENG_MAX - ENG_OFF) {
      if (!(t % 2))                   // CLOSE TO REV LIMIT
        color_led(Off);               // Flash
      else                            //
        color_eng_gear();             //
    }
    else                              // NOT CLOSE TO REV LIMIT
      color_eng_gear();               // Color based on gear and engine speed
  }
}

/* =============================================================================*/
// Logic for cornering mode
void power_led_corner() {

  // not cornering
  if (abs(Lat_Acc) < 0.1) {
    if (Veh_Spd == 0) {
      t_stop = t_stop + (t - t_buf);    // Start counting
      if (t_stop > STOP_TIMER) {          // STOPPED FOR COUNTER
        brightness = BRIGHT_MAX;
        FastLED.setBrightness(brightness);
        stop_dance(Red);                  // Run stop dance
      } else {                              // TIMER NOT ELAPSED
        t_stop = 0;
        brightness = BRIGHT_MAX / 2;
        color_led(Red);                   //
      }
    }

    // cornering
  } else {
    t_stop = 0;
    brightness = BRIGHT_MAX;
    FastLED.setBrightness(brightness);              // Set brightness
    color_led_corner(6);
  }
}

/* =============================================================================*/
// Locic for throttle mode
void power_led_throttle() {
  if (F_Accel) {
    brightness = constrain(map(Accel_Pos, 0, 100, BRIGHT_MIN, BRIGHT_MAX), BRIGHT_MIN, BRIGHT_MAX);
    color_led(Green);
  }
  else if (F_Brake) {
    brightness = constrain(map(Brake_Pos, 0, 100, BRIGHT_MIN, BRIGHT_MAX), BRIGHT_MIN, BRIGHT_MAX);
    color_led(Red);
  }
  else {
    brightness = BRIGHT_MIN;
    if (Veh_Spd > 0)
      color_led(Green);
    else
      color_led(Red);
  }
}

/* =============================================================================*/
// Change light color and brightness based on engine speed and gear
void color_eng_gear() {
  //brightness = BRIGHT_MAX;
  brightness = constrain(map(Eng_Spd, ENG_MIN, ENG_MAX, BRIGHT_MIN, BRIGHT_MAX), BRIGHT_MIN, BRIGHT_MAX);
  FastLED.setBrightness(brightness);

  ratio = constrain(Eng_Spd / max(Veh_Spd, 1), 27, 2000);
  hue = lin_interp(ratio, Gears);                         // calculate hue from 0 to 360
  hue = constrain(map(hue, 1, 360, 0, 255), 0, 255);      // convert to single byte

  for (int i = 0; i < NUM_LEDS; i++) {                    // set LEDs to hue
    led_array[i].setHue(hue);
  }
  FastLED.show();
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
void color_led_corner(int h) {

  // determine brightness offset (127 = full brightness on one side, nothing on the other)
  lat_scale = constrain(map(abs(Lat_Acc) * BRIGHT_MAX / 2, 0, 25, 0, 127), 0, 127);

  // left turn
  if (Lat_Acc > 0) {
    for (int i = 0; i < NUM_LEDS; i++) {
      if (i < NUM_LEDS / 2)
        led_array[i] = CHSV(h, 255, 127 + lat_scale); // make left side brighter
      else
        led_array[i] = CHSV(h, 255, 127 - lat_scale); // make right side dimmer
    }
  }

  // right turn
  else {
    for (int i = 0; i < NUM_LEDS; i++) {
      if (i < NUM_LEDS / 2)
        led_array[i] = CHSV(h, 255, 127 - lat_scale); // make left side dimmer
      else
        led_array[i] = CHSV(h, 255, 127 + lat_scale); // make right side brighter
    }
  }

  FastLED.show();
}

/* =============================================================================*/
// Make the lights dance when stopped for more than x seconds
void stop_dance(unsigned int color[3]) {
  /*fadeToBlackBy(led_array, NUM_LEDS, 20);
    int pos = beatsin16(13, 0, NUM_LEDS - 1);
    led_array[pos] += CRGB(color[0], color[1], color[2]);*/

  fadeToBlackBy(led_array, NUM_LEDS, 20);
  uint8_t dothue = 0;
  for (int i = 0; i < 8; i++) {
    led_array[beatsin16(i + 7, 0, NUM_LEDS - 1)] |= CHSV(dothue, 200, 255);
  }

  FastLED.show();
}

/* =============================================================================*/
// Startup sequence
void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy(led_array, NUM_LEDS, 20);
  uint8_t dothue = 0;
  for (int i = 0; i < 8; i++) {
    led_array[beatsin16(i + 7, 0, NUM_LEDS - 1)] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
  FastLED.show();
}

/* =============================================================================*/
// Function to linearly interpolate
float lin_interp(float x, int data[][2]) {
  float x0, x1, y0, y1, y;
  for (int i = 0; i < 8; i++) {
    if (x >= data[i][0] && x < data[i + 1][0]) {
      x0 = data[i][0];
      x1 = data[i + 1][0];
      y0 = data[i][1];
      y1 = data[i + 1][1];
      y  = y0 + (x - x0) * ((y1 - y0) / (x1 - x0));
      break;
    }
  }
}
