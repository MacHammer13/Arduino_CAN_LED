/* ===============================================================================
                                 Initialize
   =============================================================================*/

// Include libraries for CAN
#include <mcp_can.h>
#include <mcp_can_dfs.h>

MCP_CAN CAN0(9);  // Set CS to pin 9

/* ===============================================================================
                                 Variables
   =============================================================================*/

// CAN Message
unsigned char len = 0;
unsigned char buf[8];
unsigned long ID = 0;
const uint8_t A = 0, B = 1, C = 2, D = 3, E = 4, F = 5, G = 6, H = 7;
uint16_t IDS[100];

/* ===============================================================================
                                 Setup
   =============================================================================*/

void setup() {
  Serial.begin(9600);

  if (CAN0.begin(CAN_500KBPS) != CAN_OK) {
    while (1) {
      Serial.print("I will wait here forever...");
      delay(1000);
    }
  }

  // Filters available
  CAN0.init_Mask(0, 0, 0xFFF);

  for (int i = 0; i < 100; i++) {
    IDS[i] = 0x0;
  }
}

/* ===============================================================================
                                 Loop
   =============================================================================*/
void loop() {

  if (CAN_MSGAVAIL == CAN0.checkReceive()) {  // Check to see whether data is read
    CAN0.readMsgBufID(&ID, &len, buf);        // Read data

    for (int i = 0; i < 100; i++) {
      if (ID == IDS[i]) {
        break;
      }
      else if (IDS[i] == 0) {
        IDS[i] = ID;
        if (ID < 0x10)
          Serial.print(0, DEC);
        Serial.println(ID, HEX);
        break;
      }
    }
  }
}
