#include <U8x8lib.h>
#include <LoRa.h>

String receivedT;
String receivedRSSI;

//GPIO19 - LoRa_MISO
//GPIO18 - LoRa_CS
//GPIO5 - LoRa_SCK
//GPIO27 - LoRa_MOSI
//GPIO14 - LoRa_RST

#define SS  18 //Slave Select
#define RST 14
#define DI0 26

void setup() {

  int counter = 0;
  int conTimer = 0;
  bool conOK = 1;
  Serial.begin(115200);
  while(!Serial);
  LoRa.setPins(SS, RST, DI0);

  while (!LoRa.begin(868E6) && conOK == 1)
  {
    Serial.println(".");
    delay(1000);
    conTimer++;

    if (conTimer > 20)
    {
      conOK = 0;
      Serial.println("No connection to rocket.");
    }
    
  }
  

  LoRa.setSyncWord(0xF3);
  LoRa.beginPacket();
  LoRa.print("OK");
  LoRa.print(counter);
  LoRa.endPacket();

}

void loop() {
  

}
