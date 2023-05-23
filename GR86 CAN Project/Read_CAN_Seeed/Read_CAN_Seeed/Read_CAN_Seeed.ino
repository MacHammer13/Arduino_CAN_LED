#include <mcp_can.h>
#include <mcp_can_dfs.h>

#define CANint 2

MCP_CAN CAN0(9);  // Set CS to pin 9

// CAN Message
unsigned char len = 0;
unsigned char buf[8];
unsigned long ID = 0;
unsigned int A = 0, B = 1, C = 2, D = 3, E = 4, F = 5, G = 6, H = 7;
float Time;

/* ===============================================================================
                                 Setup
   =============================================================================*/

void setup() {
  Serial.begin(9600);

  while (!Serial) {
    Serial.print("I will wait here forever...");
    delay(1000);
  };

  Serial.println("Initializing CAN:");

  if (CAN0.begin(CAN_500KBPS) == CAN_OK) {
    Serial.println("Can Initialization Success :)");
  } else {
    Serial.println("Can Initialization Failed :(");
    while (1) {
      Serial.print("I will wait here forever...");
      delay(1000);
    }
  }

  pinMode(CANint, INPUT);                       // Setting pin 2 for /INT input

  Serial.println("Good to go!");

  Serial.print("Time\tID\tA\tB\tC\tD\tE\tF\tG\tH");

}

/* ===============================================================================
                                 Loop
   =============================================================================*/
void loop() {
  Time = millis();

  if (CAN_MSGAVAIL == CAN0.checkReceive()) {  // Check to see whether data is read
    CAN0.readMsgBufID(&ID, &len, buf);        // Read data

    display_message();

  }
}

/* ===============================================================================
                                 Functions
   =============================================================================*/
// Display Raw CAN Message
void display_message() {
  Serial.print(Time / 1000, DEC);
  Serial.print("\t");
  Serial.print(ID, HEX);  // Output HEX Header
  Serial.print("\t");

  for (int i = 0; i < len; i++) {  // Output 8 Bytes of data in HEX
    if (buf[i] < 0x10)
      Serial.print(0, DEC);
    Serial.print(buf[i], HEX);
    Serial.print("\t");
  }
  Serial.println("");
}
