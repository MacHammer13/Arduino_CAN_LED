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

#define ENGINE_MAX 7400
#define ENGINE_MIN 1000
#define STOP_TIMER 2000
#define SPEED_MAX 140
#define SPEED_MIN 1
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
const unsigned int A = 0, B = 1, C = 2, D = 3, E = 4, F = 5, G = 6, H = 7;

// create variables for brightness calculation and time
unsigned int brightness = 0, t = 0, t_buf = 0, t_stop = 0, lat_scale;

// create RGB color triplets
const unsigned int Green[3] = {0, 255, 0}, Red[3] = {255, 0, 0}, Blue[3] = {0, 0, 255};
const unsigned int Yellow[3] = {255, 255, 0}, Pink[3] = {255, 0, 255}, Cyan[3] = {0, 255, 255};
const unsigned int Orange[3] = {255, 80, 0}, Purple[3] = {110, 0, 255}, White[3] = {150, 150, 150}, Off[3] = {0, 0, 0};
unsigned int L_to_R[NUM_LEDS], R_to_L[NUM_LEDS];

// CAN signals to be calculated
unsigned int Gear, Gear_Buf, Accel_Pos, Engine_Speed, Engine_Speed_Buf, Vehicle_Speed, Brake_Pos;
bool Clutch_Switch, Brake_Switch, Accel_Switch, Sport_Switch;
float Steering_Angle, Tire_Angle, Yaw_Rate, Longitudinal_Acceleration, Lateral_Acceleration;

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
  FastLED.setBrightness(BRIGHT_MAX);
  for (int i = 0; i < NUM_LEDS + 8; i++) {
    if (i < NUM_LEDS)
      led_array[i].setRGB(White[0], White[1], White[2]);
    if (i > 8)
      led_array[i - 9].setRGB(Off[0], Off[1], Off[2]);
    FastLED.show();
    if (i < (NUM_LEDS / 2)) {
      L_to_R[i] = NUM_LEDS - i - 1;
      R_to_L[i] = (NUM_LEDS / 2) - i - 1;
    }
    else {
      L_to_R[i] = i - (NUM_LEDS / 2) - 1;
      R_to_L[i] = i + (NUM_LEDS / 2) + i - 1;
    }
    delay((1 / NUM_LEDS) * 1000);
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
    if (Sport_Switch) {                       // Determine mode
      calc_signals_eng();                     // calculate new signals for engine and pedals
      power_led_eng();                        // engine speed, brakes, and gear
    }                       
    else if (1) {
      calc_signals_corner();                  // calculate new signals for cornering
      power_led_corner();                     // steering angle, lateral acceleration
    }                      
    else
      color_led(Off);                         // turn off if not in sport mode
  }


}

/* ===============================================================================
                                 Functions
   =============================================================================*/
// Calculate known signals based on ID and equation for enigne and pedals
void calc_signals_eng() {
  switch (ID) {
    case 0x40:
      Engine_Speed_Buf = (buf[D] << 8 | buf[C]) & 0x3FFF;
      if (Engine_Speed_Buf <= ENGINE_MAX)
        Engine_Speed = Engine_Speed_Buf;
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
      if ((t % 100) < 5)
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
// Calculate known signals based on ID and equation for cornering
void calc_signals_corner() {
  switch (ID) {
    case 0x138:
      Steering_Angle = (buf[D] << 8 | buf[C]) * -0.1;
      Yaw_Rate = (buf[E] << 8 | buf[C]) * -0.2725;
      CAN0.init_Filt(0, 0, 0x13B);
      break;
    case 0x13B:
      Lateral_Acceleration = buf[G] * 0.2;
      Longitudinal_Acceleration = buf[H] * -0.1;
      CAN0.init_Filt(0, 0, 0x143);
      break;
    case 0x143:
      Vehicle_Speed = ((buf[E] << 8 | buf[D]) & 0x3FFF) * 0.015694;
      if ((t % 100) < 5)
        CAN0.init_Filt(0, 0, 0x328);
      else
        CAN0.init_Filt(0, 0, 0x138);
      break;
    case 0x328:
      Sport_Switch = buf[E] & 0x1;
      CAN0.init_Filt(0, 0, 0x138);
  }
}

/* =============================================================================*/
// Logic for determining how to color leds
void power_led_eng() {
  if (Brake_Switch) {                 // ON BRAKES
    if (Vehicle_Speed == 0) {           // VEHICLE STOPPED
      t_stop = t_stop + (t - t_buf);    // Start counting
      if (t_stop > STOP_TIMER) {          // STOPPED FOR COUNTER
        brightness = 100;                 // Run stop dance
        stop_dance(Red);                  //
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
// Logic for determining how to color leds
void power_led_corner() {
  if (abs(Lateral_Acceleration) < 0.1) {
    if (Vehicle_Speed == 0) {
      stop_dance(Red);
    } else {
      brightness = constrain(map(Vehicle_Speed, SPEED_MIN, SPEED_MAX, 0, 100), 0, 100);
      color_led(Red);
    }
  } else {
    brightness = constrain(map(Vehicle_Speed, SPEED_MIN, SPEED_MAX, 0, BRIGHT_MAX), 0, BRIGHT_MAX);
    lat_scale = constrain(map(Lateral_Acceleration, -1, 1, 0, NUM_LEDS), 0, NUM_LEDS);
  }
}
/* =============================================================================*/
// Change light color and brightness based on engine speed and gear
void color_eng_gear() {
  brightness = map(Engine_Speed, ENGINE_MIN, ENGINE_MAX, 0, BRIGHT_MAX);
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
  FastLED.setBrightness(brightness);
  for (int i = 0; i < NUM_LEDS - 1; i++) {
    if (Longitudinal_Acceleration < 0) {
      if (i < lat_scale)
        led_array[L_to_R[i]] = CRGB(color[0], color[1], color[2]);
      else
        led_array[L_to_R[i]] = CRGB(Off[0], Off[1], Off[2]);
    }
    else   {
      if (i < lat_scale)
        led_array[R_to_L[i]] = CRGB(color[0], color[1], color[2]);
      else
        led_array[R_to_L[i]] = CRGB(Off[0], Off[1], Off[2]);
    }
  }
  FastLED.show();
}

/* =============================================================================*/
// Make the lights dance when stopped for more than 2 seconds
void stop_dance(unsigned int color[3]) {
  FastLED.setBrightness(BRIGHT_MAX);
  fadeToBlackBy(led_array, NUM_LEDS, 20);
  int pos = beatsin16(13, 0, NUM_LEDS - 1);
  led_array[pos] += CRGB(color[0], color[1], color[2]);
  FastLED.show();
}
