/* ===============================================================================
                                 Initialize
   =============================================================================*/

// Include libraries for LEDs and CAN
#include <FastLED.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>

// Define constants
#define NUM_LEDS 288
#define LED_PIN 3

#define BRIGHT_MIN 1
#define BRIGHT_MAX 50

#define START_TIMER 5000
#define STOP_TIMER 3000

#define ENG_MAX 7500
#define ENG_OFF 500

// Create LED Array
CRGB led_array[NUM_LEDS];

// Set CS to pin 9
MCP_CAN CAN0(9);

/* ===============================================================================
                                 Variables
   =============================================================================*/
// Initialize all variables

// create variables for CAN Message and signal calculation
unsigned char len = 0, buf[8];
unsigned long ID = 0;
const uint8_t A = 0, B = 1, C = 2, D = 3, E = 4, F = 5, G = 6, H = 7, g = 98; // g * 10

// create variables for brightness calculation and time
uint8_t brightness = 0, lat_scale = 0;
uint32_t t = 0, t_buf = 0, t_stop = 0;

// create RGB color triplets
uint16_t Red = 0, Orange = 30, Yellow = 60, Green = 120, Cyan = 180, Blue = 240, Purple = 270, Pink = 300, hue_c = 0;
uint16_t Gears[9][2] = {{1, 359},
  {27, Pink},     // 6th gear
  {35, Purple},   // 5th gear
  {42, Blue},     // 4th gear
  {53, Cyan},     // 3rd gear
  {76, Green},    // 2nd gear
  {125, Yellow},  // 1st gear
  {200, Orange},
  {2000, Red}
};

uint16_t ids[6] = {0x390, 0x3AC, 0x0, 0x0, 0x0, 0x0}, id = 0;

// CAN signals to be calculated
uint8_t Gear, Accel_Pos, Brake_Pos, Dash_Bright, Air_Temp, Oil_Temp, Cool_Temp, mode, mode_buf;
uint16_t Eng_Spd, Eng_Spd_Buf, Brake_Pres, Eng_Min;
bool F_Clutch, F_Brake, F_Accel, F_AC, F_DrivDoor, F_PassDoor, F_Light, F_Park, F_Headlights, F_DRL, F_Highbeams, F_Sport;
float Steer_Ang, Tire_Ang, Yaw_Rate, Lng_Acc, Lat_Acc, Gear_Ratio, Veh_Spd;

/* ===============================================================================
                                 Setup
   =============================================================================*/
// Main setup function
void setup() {

  // Initialize Serial
  Serial.begin(9600);

  // Initialize LEDs
  FastLED.addLeds<NEOPIXEL, LED_PIN>(led_array, NUM_LEDS);

  // Initialize CAN connection
  if (CAN0.begin(CAN_500KBPS) != CAN_OK) {

    // set brightness
    brightness = BRIGHT_MAX;

    // blink red if it failed
    while (1) {
      color_led(Red);
      FastLED.show();
      delay(1000);
      fill_solid(led_array, NUM_LEDS, CRGB(0, 0, 0));
      FastLED.show();
      delay(1000);
    }

  }

  // Initialize CAN Mask
  CAN0.init_Mask(0, 0, 0xFFF);  // Initialize CAN Mask
  CAN0.init_Filt(0, 0, 0x390);
  pinMode(2, INPUT);                       // Setting pin 2 for /INT input

  // Good to go!

}

/* ===============================================================================
                                 Loop
   =============================================================================*/
// Main function loop
void loop() {

  // get time and keep last time
  t_buf = t;
  t = millis();

  // set mode buf
  mode_buf = mode;

  // look for CAN message
  if (CAN_MSGAVAIL == CAN0.checkReceive()) {

    // read CAN message
    CAN0.readMsgBufID(&ID, &len, buf);

    // calculate signals
    calc_signals();

    // there is a door open
    if (F_DrivDoor || F_PassDoor) {
      mode = 7;

      // set brightness
      brightness = BRIGHT_MAX;

      // passenger door open
      if (F_PassDoor) {

        // turn on passenger LEDs
        for (int i = 0; i < NUM_LEDS / 2; i++) {
          led_array[i].setRGB(255, 255, 255);
        }

      }

      // driver door open
      if (F_DrivDoor) {

        // turn on driver LEDs
        for (int i = NUM_LEDS / 2; i < NUM_LEDS; i++) {
          led_array[i].setRGB(255, 255, 255);
        }

      }

      // set list of IDs to check
      ids[0] = 0x390, ids[1] = 0x3AC, ids[2] = 0x0, ids[3] = 0x0, ids[4] = 0x0, ids[5] = 0x0;

    }

    // parking brake is on
    else if (F_Park) {
      mode = 9;

      // set brightness
      brightness = BRIGHT_MAX;

      // apply brightness
      FastLED.setBrightness(brightness);

      // run rainbow function
      juggle();

      // set list of IDs to check
      ids[0] = 0x390, ids[1] = 0x3AC, ids[2] = 0x0, ids[3] = 0x0, ids[4] = 0x0, ids[5] = 0x0;

    }

    // low brightness or headlights are on
    else if (!F_Light || F_Headlights) {

      // dashboard brightness switch value
      switch (Dash_Bright) {

        // mode 1, lowest brightness
        case 0x12:
          mode = 1;

          // set list of IDs
          ids[0] = 0x390, ids[1] = 0x3AC, ids[2] = 0x0, ids[3] = 0x0, ids[4] = 0x0, ids[5] = 0x0;

          // turn off LEDs
          fill_solid(led_array, NUM_LEDS, CRGB(0, 0, 0));

          break;

        // mode 2
        case 0x1E:
          mode = 2;

          // set list of IDs
          ids[0] = 0x390, ids[1] = 0x3AC, ids[2] = 0x0, ids[3] = 0x0, ids[4] = 0x0, ids[5] = 0x0;

          // turn off LEDs
          fill_solid(led_array, NUM_LEDS, CRGB(0, 0, 0));

          break;

        // mode 3
        case 0x35:
          mode = 3;

          // set list of IDs
          ids[0] = 0x390, ids[1] = 0x3AC, ids[2] = 0x0, ids[3] = 0x0, ids[4] = 0x0, ids[5] = 0x0;

          // turn off LEDs
          fill_solid(led_array, NUM_LEDS, CRGB(0, 0, 0));

          break;

        // mode 4, throttle mode
        case 0x5A:
          mode = 4;

          // set IDs
          ids[0] = 0x40, ids[1] = 0x139, ids[2] = 0x390, ids[3] = 0x3AC, ids[4] = 0x0, ids[5] = 0x0;

          // run function for throttle/brake control
          power_led_throttle();

          break;

        // mdoe 5, cornering mode
        case 0x96:
          mode = 5;

          // set IDs
          ids[0] = 0x139, ids[1] = 0x13B, ids[2] = 0x328, ids[3] = 0x390, ids[4] = 0x3AC, ids[5] = 0x0;

          // run function for cornering control
          power_led_corner();

          break;

        // mode 6, engine/gear mode
        case 0xFA:
          mode = 6;

          // set IDs
          ids[0] = 0x40, ids[1] = 0x139, ids[2] = 0x328, ids[3] = 0x390, ids[4] = 0x3AC, ids[5] = 0x0;

          // run function for engine/gear control
          power_led_eng();

          break;

        // default mode
        default:
          mode = 0;

          // set IDs
          ids[0] = 0x390, ids[1] = 0x3AC, ids[2] = 0x0, ids[3] = 0x0, ids[4] = 0x0, ids[5] = 0x0;

          // turn off LEDs
          fill_solid(led_array, NUM_LEDS, CRGB(0, 0, 0));

          break;

      }

    }

    // high brightness
    else {

      // set brightness
      brightness = 10;

      // color solid red
      color_led(Red);

    }

  }

  // deploy LEDs
  FastLED.show();

  if (mode != mode_buf) {
    id = 0;
    CAN0.init_Filt(0, 0, ids[id]);
  }

  /*Serial.print(t,DEC);
    Serial.print("\t");
    Serial.print(Lat_Acc,DEC);
    Serial.print("\t");
    Serial.print(Lng_Acc,DEC);
    Serial.print("\t");
    Serial.print(lat_scale,DEC);
    Serial.println("");*/

  // update hue for cornering mode
  EVERY_N_SECONDS( 1 ) {
    hue_c++;
  }

  if (hue_c > 359)
    hue_c = 0;
  
}

/* ===============================================================================
                                 Functions
   =============================================================================*/
// Calculate signals from CAN message and set new filters
void calc_signals() {

  // calculate signals based on can ID
  switch (ID) {

    // throttle pedal
    case 0x40:
      Eng_Spd_Buf = uint16_t(buf[D] << 8 | buf[C]) & 0x3FFF;          // rpm
      if (Eng_Spd_Buf <= ENG_MAX)
        Eng_Spd = Eng_Spd_Buf;                                        // rpm
      Accel_Pos = uint8_t(buf[E] / 2.55);                             // %
      F_Accel = (buf[H] & 0xC0) != 0xC0;                              // true = on
      break;

    // air con
    case 0x41:
      F_AC = (buf[H] & 0x2) == 2;                                     // true = on
      break;

    // steering
    case 0x138:
      Steer_Ang = int16_t(buf[D] << 8 | buf[C]) * -0.1;               // degrees
      Yaw_Rate = int16_t(buf[F] << 8 | buf[E]) * -0.2725;             // deg/s? rad/s?
      break;

    // brakes
    case 0x139:
      Veh_Spd = (uint16_t(buf[D] << 8 | buf[C]) & 0x1FFF) * 0.05625;  // kph
      F_Brake = (buf[E] & 0x4) == 4;                                  // true = on
      Brake_Pos = min(buf[F] / 0.7, 100);                             // %
      Brake_Pres = uint16_t(buf[F] * 128);                            // pa? kpa? psi?
      break;

    // accelerometers
    case 0x13B:
      Lat_Acc = (int8_t(buf[G]) * -0.1);                              // m/s^2
      Lng_Acc = (int8_t(buf[H]) * -0.1);                              // m/s^2
      break;

    // transmission
    case 0x241:
      Gear = buf[E] >> 3 & 0x7;                                       // -
      F_Clutch = ((buf[F] & 0x80) / 1.28) == 100;                     // true = depressed                                            // -
      break;

    // sport mode
    case 0x328:
      F_Sport = buf[E] & 0x1;                                         // true = sport mode
      break;

    // engine temps
    case 0x345:
      Oil_Temp = int8_t(buf[D]) - 40;                                 // C
      Cool_Temp = int8_t(buf[E]) - 40;                                // C
      break;

    // brightness
    case 0x390:
      Air_Temp = int8_t(buf[E]) / 2 - 40;                             // C
      Dash_Bright = buf[F];                                           // ?
      F_Light = buf[G] & 0x10;                                        // true = bright, false = dark enough for headlights
      break;

    // doors
    case 0x3AC:
      F_DrivDoor = buf[E] & 0x1;                                      // true = open
      F_PassDoor = buf[E] & 0x2;                                      // true = open
      F_Park = buf[G] & 0x20;                                         // true = parking brake activated
      F_DRL = buf[H] & 0x1;                                           // true = on
      F_Headlights = buf[H] & 0x2;                                    // true = on
      F_Highbeams = buf[H] & 0x4;                                     // true = on
      break;
  }

  // rotate IDs

  // id matches requested ID
  if (ID == ids[id]) {

    // go to next id
    id = id + 1;

    // reset if at the end of list
    if (ids[id] == 0x0)
      id = 0;

    // apply new filter
    CAN0.init_Filt(0, 0, ids[id]);

  }

}

/* =============================================================================*/
// Logic for engine mode
void power_led_eng() {

  // on brakes
  if (F_Brake) {

    // vehicle stopped
    if (Veh_Spd == 0) {

      // timer
      t_stop = t_stop + (t - t_buf);

      // timer elapsed
      if (t_stop > STOP_TIMER) {

        // set brightness
        brightness = BRIGHT_MAX;

        // run stop dance
        stop_dance(Red);
      }

      // timer not elapsed
      else {

        // set brightnee based on pedal position
        brightness = constrain(map(Brake_Pos, 0, 100, BRIGHT_MIN, BRIGHT_MAX), BRIGHT_MIN, BRIGHT_MAX);

        // color LEDs red
        color_led(Red);

      }

    }

    // vehicle moving
    else {

      // reset timer
      t_stop = 0;

      // set brightness based on brake position
      brightness = constrain(map(Brake_Pos, 0, 100, BRIGHT_MIN, BRIGHT_MAX), BRIGHT_MIN, BRIGHT_MAX);

      // color red
      color_led(Red);

    }

  }

  // off brakes
  else {

    // reset timer
    t_stop = 0;

    // engine speed close to redline
    if (Eng_Spd > ENG_MAX - ENG_OFF) {

      // flash LEDs
      if (!(t % 2))
        fill_solid(led_array, NUM_LEDS, CRGB(0, 0, 0));
      else
        color_eng_gear();
    }

    // not close to redline
    else
      color_eng_gear();

  }

}

/* =============================================================================*/
// Logic for cornering mode
void power_led_corner() {

  // stopped
  if (Veh_Spd == 0) {

    // count timer
    t_stop = t_stop + (t - t_buf);

    // timer elapsed
    if (t_stop > STOP_TIMER) {

      // set brightness
      brightness = BRIGHT_MAX;

      // run the stop dance
      stop_dance(hue_c);

    }

    // timer not elapsed
    else {

      // set brightness to half
      brightness = BRIGHT_MIN;

      // set color to red
      color_led(Red);

    }

  }

  // vehicle moving
  else {

    // reset timer
    t_stop = 0;

    // set brightness to half
    brightness = BRIGHT_MAX;

    // set color to red
    color_led_corner(hue_c);
  }

}

/* =============================================================================*/
// Locic for throttle mode
void power_led_throttle() {

  // On accel pedal
  if (F_Accel) {

    // set brightness based on accel pedal position
    brightness = constrain(map(Accel_Pos, 0, 100, BRIGHT_MIN, BRIGHT_MAX), BRIGHT_MIN, BRIGHT_MAX);

    // color LEDs green
    color_led(Green);

  }

  // On brake peda;
  else if (F_Brake) {

    // set brightness based on brake pedal position
    brightness = constrain(map(Brake_Pos, 0, 100, BRIGHT_MIN, BRIGHT_MAX), BRIGHT_MIN, BRIGHT_MAX);

    // color LEDs red
    color_led(Red);

  }

  // On neither pedal
  else {

    // low brightness
    brightness = BRIGHT_MIN;

    // green if moving, red if stopped
    if (Veh_Spd > 0)
      color_led(Green);
    else
      color_led(Red);

  }

}

/* =============================================================================*/
// Change light color and brightness based on engine speed and gear
void color_eng_gear() {

  // set engine speed min
  if (F_Sport)
    Eng_Min = 3000;
  else
    Eng_Min = 1000;
    
  // scale brightness based on engine speed
  brightness = constrain(map(Eng_Spd, Eng_Min, ENG_MAX, BRIGHT_MIN, BRIGHT_MAX), BRIGHT_MIN, BRIGHT_MAX);

  // calculate gear ratio
  float ratio = constrain(Eng_Spd / max(Veh_Spd, 1), 27, 2000);

  // scale hue based on gear ratio
  float hue_360 = lin_interp(ratio, Gears);

  // set the LED color and brightness
  color_led(hue_360);

}

/* =============================================================================*/
// Color the LEDs based on desired color and brightness
void color_led(uint16_t color) {

  // set LED brightness
  FastLED.setBrightness(brightness);

  // convert color from 0-360 to single byte
  uint8_t hue = constrain(map(color, 0, 359, 0, 255), 0, 255);

  // use built in function to color all LEDs
  fill_solid(led_array, NUM_LEDS, CHSV(hue, 255, 255));

}

/* =============================================================================*/
// Color the LEDs based on desired color and brightness
void color_led_corner(uint16_t color) {

  // set brightness
  FastLED.setBrightness(brightness);

  // convert color from 0-360 to single byte
  uint8_t hue = constrain(map(color, 0, 359, 0, 255), 0, 255);

  // determine brightness offset (127 = full brightness on one side, nothing on the other)
  if (F_Sport)
    lat_scale = constrain(map(abs(Lat_Acc) * 10, 0, g, 0, NUM_LEDS / 2), 0, NUM_LEDS / 2);
  else
    lat_scale = constrain(map(abs(Lat_Acc) * 10, 0, g / 2, 0, NUM_LEDS / 2), 0, NUM_LEDS / 2);


  // control LEDs
  for (int i = 0; i < NUM_LEDS; i++) {

    // left turn
    if (Lat_Acc < 0) {
      if (i < lat_scale)
        led_array[i].setHue(hue);
      else
        led_array[i] = CHSV(0, 0, 0);
    }

    // right turn
    else if (Lat_Acc > 0) {
      if (i > NUM_LEDS / 2 && i < NUM_LEDS / 2 + lat_scale)
        led_array[i].setHue(hue);
      else
        led_array[i] = CHSV(0, 0, 0);
    }

    // no turn
    else {
      fill_solid(led_array, NUM_LEDS, CHSV(0, 0, 0));
    }
  }
}

/* =============================================================================*/
// Make the lights dance when stopped for more than x seconds
void stop_dance(uint16_t color) {

  // set brightness
  FastLED.setBrightness(brightness);

  uint8_t hue = constrain(map(color, 1, 360, 0, 255), 0, 255);      // convert to single byte

  fadeToBlackBy(led_array, NUM_LEDS, 20);
  for (int i = 0; i < 8; i++) {
    led_array[beatsin16(i + 7, 0, NUM_LEDS - 1)] |= CHSV(hue, 200, 255);
  }
}

/* =============================================================================*/
// Startup sequence
void juggle() {

  FastLED.setBrightness(brightness);

  fadeToBlackBy(led_array, NUM_LEDS, 20);
  uint8_t dothue = 0;
  for (int i = 0; i < 8; i++) {
    led_array[beatsin16(i + 7, 0, NUM_LEDS - 1)] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}

/* =============================================================================*/
// Function to linearly interpolate
float lin_interp(float x, uint16_t data[][2]) {

  float x0, x1, y0, y1;

  for (int i = 0; i < 9; i++) {

    if (x >= data[i][0] && x < data[i + 1][0]) {
      x0 = data[i][0];
      x1 = data[i + 1][0];
      y0 = data[i][1];
      y1 = data[i + 1][1];
      return y0 + (x - x0) * ((y1 - y0) / (x1 - x0));
      break;
    }

  }

}
