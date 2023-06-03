/* ===============================================================================
                                 Initialize
   =============================================================================*/
// Include libraries for CAN
#include <mcp_can.h>
#include <mcp_can_dfs.h>

// Define constants
#define CANint 2

MCP_CAN CAN0(9);  // Set CS to pin 9

/* ===============================================================================
                                 Variables
   =============================================================================*/

// CAN Message
unsigned char len = 0;
unsigned char buf[8];
unsigned long ID = 0;
const unsigned int A = 0, B = 1, C = 2, D = 3, E = 4, F = 5, G = 6, H = 7;
int Time, Time_Buf;

int id = 0x7C2;

/* ===============================================================================
                                 Setup
   =============================================================================*/

void setup() {
  Serial.begin(9600);

  while (!Serial) {
    Serial.print("I will wait here forever...");
    delay(1000);
  };

  //Serial.println("Initializing CAN:");

  if (CAN0.begin(CAN_500KBPS) == CAN_OK) {
    //Serial.println("Can Initialization Success :)");
  } else {
    Serial.println("Can Initialization Failed :(");
    while (1) {
      Serial.print("I will wait here forever...");
      delay(1000);
    }
  }

  pinMode(CANint, INPUT);                       // Setting pin 2 for INT input

  //Serial.println("Good to go!");

  //Serial.print("Raster\tID\tA\tB\tC\tD\tE\tF\tG\tH\n");

  // Filters available
  CAN0.init_Mask(0, 0, 0xFFF);
  CAN0.init_Filt(0, 0, id);
}

/* ===============================================================================
                                 Loop
   =============================================================================*/
void loop() {
  Time_Buf = Time;
  Time = millis();

  /*ID = random(0x100,0x11F);
  for (int i=0; i < 8; i++) {
    buf[i] = random(0xFF);
  }
  len = 8;
  display_message();*/

  if (CAN_MSGAVAIL == CAN0.checkReceive()) {  // Check to see whether data is read
    CAN0.readMsgBufID(&ID, &len, buf);        // Read datab

    if (ID == id)
      display_message();                        // Display Time, ID and Message
  }
}

/* ===============================================================================
                                 Functions
   =============================================================================*/

// Display Raw CAN Message
void display_message() {
  //Serial.print(Time,DEC);
  //Serial.print(" ");
  if (ID < 0x100)
    Serial.print(0,DEC);
  if (ID < 0x10)
    Serial.print(0,DEC);
  Serial.print(ID, HEX);  // Output HEX Header
  Serial.print(" ");
  Serial.print(len,DEC); // Output Length
  for (int i = 0; i < len; i++) {  // Output 8 Bytes of data in HEX
    Serial.print(" ");
    if (buf[i] < 0x10)
      Serial.print(0, DEC);
    Serial.print(buf[i], HEX);
  }
  Serial.println("");
}
