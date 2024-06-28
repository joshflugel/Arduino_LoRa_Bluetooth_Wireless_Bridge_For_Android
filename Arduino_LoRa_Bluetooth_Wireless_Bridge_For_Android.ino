/* 
Arduino Wireless Bluetooth-LoRa bridge by Josué Galindo

                         |      LoRa| <- Low Power Area Network -> |LoRa     |
                         | Arduino 1|                              |Arduino 2|
|Android Device "A"| <-> | Bluetooth|                              |Bluetooth| <-> |Android Device "B"| 

In the schematic above, this code runs in Arduino Microcontrollers 1 and 2, 
which are connected to Android devices A and B respectively via Low Eenrgy Bluettooth (BLE).
The Arduinos acts as a bridge that allows both Android devices to communicate with each other
via LoRa WAN (Low Power Wide Area Network), in use cases where cellular network 
or WiFi connectivity is not available.
*/
                     
#include "Arduino.h"
#include "images.h"
#include "LoRaWan_APP.h"
#include <Wire.h>  
#include <HT_Display.h>  
#include "HT_SSD1306Wire.h"

// ==========================================   BLUETOOTH BLE code block 
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID        "5914b79c-0184-11ee-be56-0242ac120002"
#define CHARACTERISTIC_UUID "ff9afc85-0dfd-4e22-9a50-832b9a7dac9e"

BLECharacteristic *pCharacteristic;
String defaultReadMessage = "You read this from LoRa";
uint8_t* stringToUnsignedByteArray(const String& str) {
    char* charArray = new char[str.length() + 1];
    str.toCharArray(charArray, str.length() + 1);
    return reinterpret_cast<uint8_t*>(charArray);
}
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^   BLUETOOTH BLE code block 


// GLCD VARS
SSD1306Wire  factory_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst
// Show HELTEC Manufacturer Logo on GLCD
void logo(){
	factory_display.clear();
	factory_display.drawXbm(0,5,logo_width,logo_height,(const unsigned char *)logo_bits);
	factory_display.display();
}


// HELPER FUNCTIONS: Onboard Activity LED
void turnActivityLED_ON() {
   digitalWrite(LED, HIGH);  
}
void turnActivityLED_OFF() {
   digitalWrite(LED, LOW);  
}



/********************************* LORA VARS and Init  *********************************************/
#define RF_FREQUENCY                                868000000 // Hz

#define TX_OUTPUT_POWER                             16        // dBm, max in EU Region is 16dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            3000
#define TX_TIMEOUT_VALUE                            3000
#define BUFFER_SIZE                                 255       // Define the payload size here

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void OnRxTimeout( void );
void OnRxError( void );

typedef enum {
    LOWPOWER,
    STATE_RX,
    STATE_TX
} States_t;


const int ZERO = 0;
const int ONCE = 1; //TX
const int TWICE = 2; //RX
int blink = ZERO;
int16_t i_mainLoop;
int16_t rxNumber;
States_t state;
bool sleepMode = false;
int16_t Rssi,rxSize;

String rssi = "RSSI --";
String rxLoRaSignalStrength = "--";
String rxLoraMessage;
String send_num;
String show_lora = "lora data show";

unsigned int counter = 0;
bool receiveflag = false; // software flag for LoRa receiver, received data makes it true.
uint64_t chipid;
int16_t RssiDetection = 0;

/////  LORA INIT
void lora_init(void){
  Mcu.begin();
  Rssi=0;
  rxNumber = 0;
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;

  Radio.Init( &RadioEvents );
  Radio.SetChannel( RF_FREQUENCY );
  Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                 LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                 LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                 true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE );

  Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                 LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                 LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                 0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
	 state = STATE_RX;
} // end lora_init

String uint64ToString(uint64_t input) {
  String result = "";
  uint8_t base = 16;
  do {
    char c = input % base;
    input /= base;
    if (c < 10)
      c +='0';
    else
      c += 'A' - 10;
    result = c + result;
  } while (input);
  return result;
}
/********************************* LORA VARS and Init  end here ***************************************/
// Actual implementation of this method happens below around Line 303 aprox



// ==========================================   BLUETOOTH BLE code block  
bool hasConnectedAfterSetup = false;
bool isBluetoothConnected = false;
class BluetoothServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      isBluetoothConnected = true;
      hasConnectedAfterSetup = true;
      Serial.println("BLE Device Connected");
    }

    void onDisconnect(BLEServer* pServer) {
      isBluetoothConnected = false;
      Serial.println("BLE Device Disconnected");
    }
};

class androidBluetoothCallbacks: public BLECharacteristicCallbacks {
  // When Android READS from BLE Chiplet
  void onRead(BLECharacteristic* pCharacteristic, esp_ble_gatts_cb_param_t* param){
    pCharacteristic->setValue(stringToUnsignedByteArray(defaultReadMessage), defaultReadMessage.length()); 
    pCharacteristic->notify();
    refreshGLCD_BT_Tx(defaultReadMessage);
    Serial.println();
    Serial.print("*** BLE.onRead Tx: ");
    Serial.println(pCharacteristic->getValue().c_str());
    Serial.println();
  }
  // When Android WRITES to BLE Chiplet. ANDROID BLE -> LoRa
  void onWrite(BLECharacteristic *pCharacteristic) {
    String stringFromAndroid = String(pCharacteristic->getValue().c_str());
    stringFromAndroid.toCharArray(txpacket, sizeof(txpacket));
    dispatchLoRaTransmissionBuffer(txpacket);
    refreshGLCD_BT_Rx(stringFromAndroid);
    Serial.print("\n*** BLE.onWrite Rx: ");
    Serial.println(pCharacteristic->getValue().c_str());
    Serial.println();
  }
  void onNotify(BLECharacteristic  *pCharacteristic) { }
};
BLEService *pService;
BLEServer *pServer;
void initializeBluetooth(String deviceName) {
  BLEDevice::init(deviceName.c_str());
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new BluetoothServerCallbacks());
  pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
                                       );
  /* The Client characteristic configuration descriptor (CCCD) is a specific type of characteristic descriptor
  that is necessary when the characteristic supports server-initiated operations (i.e Notify and Indicate).  */
  BLEDescriptor cccdDescriptor(BLEUUID((uint16_t)0x2902));
  pCharacteristic->addDescriptor(&cccdDescriptor);
  pCharacteristic->setCallbacks(new androidBluetoothCallbacks());
  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
}
void mockMessageFromLora(){  // Mock method only for development
  String mockMessage = "I'm LoRa";
  Serial.println("MOCK message - BT Tx: " + mockMessage);
  pCharacteristic->setValue(stringToUnsignedByteArray(mockMessage), mockMessage.length());
  pCharacteristic->notify();
}
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^   BLUETOOTH BLE code block 



// ARDUINO CHIP_ID self identification Utilities: serial/mac/name/
String chipidString;
const String CHIP_ID_DARKSTAR = "CC3766FA12F4";
const String CHIP_NAME_DARKSTAR = "DARKSTAR";
const String CHIP_SERIAL_DARKSTAR = "2301";
const String CHIP_ID_WHITETIGER = "7C466AFA12F4";
const String CHIP_NAME_WHITETIGER = "WHITE TIGER";
const String CHIP_SERIAL_WHITETIGER = "2252";
const String easyChipNamePrefix = "LORA-WAN Arduino ";
String easyChipName;
String easyChipSerial;
// Obtains the name of this Arduino Chip
void setupEasyChipNameAndSerial() {
  uint64_t chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
	Serial.printf("ESP32ChipID=%04X",(uint16_t)(chipid>>32)); //print High 2 bytes
	Serial.printf("%08X\n",(uint32_t)chipid);                 //print Low 4bytes.
  String chipidString;
  chipidString = uint64ToString(chipid);
  if(chipidString == CHIP_ID_DARKSTAR) {
    easyChipName = CHIP_NAME_DARKSTAR;
    easyChipSerial = CHIP_SERIAL_DARKSTAR;
  }
  else
  if(chipidString == CHIP_ID_WHITETIGER) {
    easyChipName = CHIP_NAME_WHITETIGER;
    easyChipSerial = CHIP_SERIAL_WHITETIGER;
  }
}
void displayEasyChipName (uint row = 0){
  factory_display.drawString(0, 10 * row, easyChipName + " - " +easyChipSerial);
  factory_display.display();
}



///// MAIN LOOP VARS
bool resendflag=false;
bool deepsleepflag=false;
bool interrupt_flag = false;
void interrupt_GPIO0() {// Called in Setup
	interrupt_flag = true;
}
void interrupt_handle(void) {// Called once, every MainLoop
	if(interrupt_flag) {
		interrupt_flag = false;
		if(digitalRead(0)==0) {
			if(rxNumber <=2) {
				resendflag=true;
        Serial.println("resendFlag = TRUE");
			} else {
				deepsleepflag=true;
        Serial.println("deepsleepflag = TRUE");
			}
		}
	}

}
void VextON(void) {   // runs in Setup
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
  
}
void VextOFF(void) {   // runs in Main Loop
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, HIGH);
}



// LORA TX/RX CALLBACKS
void OnTxDone( void ) {
  String status = getLoraRadioStatus();
  Serial.println(status);
  Serial.printf("%s: TX completed", easyChipName);
  Serial.println();
  Radio.Rx(0);
  Radio.IrqProcess();
	refreshAndLogLoraRadioStatus();
}
void OnTxTimeout( void ) {
  String status = getLoraRadioStatus();
  Serial.println(status);
  //Radio.Sleep( );
  Serial.printf("%s: TX Timeout", easyChipName);
  Radio.Rx(0);
  Radio.IrqProcess();
	refreshAndLogLoraRadioStatus();
}
// When a payload from LoRa is received, method onRxDone, sends it to Android over BLE
// by updating the characteristic value and notifyiung the change
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr ) {
  Serial.println("\n\nRxDone");
  
	//rxNumber++;
  // RSSI Signal Strength Indicator = -30dBm: signal is strong. RSSI=-120dBm: signal is weak.
  Rssi=rssi;
  rxSize=size;

  size = min(size, static_cast<uint16_t>(BUFFER_SIZE - 1));
  memcpy(rxpacket, payload, size );
  rxpacket[size]='\0';
  //Radio.Sleep( );
  Serial.printf("\r%s received a packet: \"%s\" with Rssi %d , length %d\r\n",easyChipName,rxpacket,Rssi,rxSize);


  const String stringFromLora = String(rxpacket);
  Serial.println("String from LORA Rx: " + stringFromLora);

  pCharacteristic->setValue(stringFromLora.c_str());
  pCharacteristic->notify();
  refreshGLCD_BT_Tx(stringFromLora);


  rxLoraMessage ="";
  int i = 0;
  while(i < rxSize){
    rxLoraMessage += rxpacket[i];
    i++;
  }

  rxLoRaSignalStrength = "";
  rxLoRaSignalStrength += "Signal: ";
  rxLoRaSignalStrength += String(Rssi,DEC);
  rxLoRaSignalStrength += " dB";
  String signalStrength = "Weak";
  if(Rssi > -120)
    signalStrength = "Medium";
  if(Rssi > -30)
    signalStrength = "Strong";
  rxLoRaSignalStrength += ", " + signalStrength;

  refreshGLCD_Lora_Rx(rxLoraMessage);
  refreshGLCD_LoraSignalStrength(rxLoRaSignalStrength);
  blink = TWICE;
  String status = getLoraRadioStatus();
  Serial.println("\n\nRxDone");
  Serial.println(status);
}
void OnRxTimeout() {
  Serial.println("RX Timeout");
}
void OnRxError() {
  Serial.println();
  Serial.println("RX ERROR");
  Serial.println();
  pCharacteristic->setValue("Incoming Message could not be opened.");
  pCharacteristic->notify();
  refreshGLCD_BT_Tx("LoRa RX ERROR");
}



void setup() {
	Serial.begin(115200);

  setupEasyChipNameAndSerial();
  initializeBluetooth(easyChipNamePrefix + easyChipName);

	VextON();
	delay(100);
	factory_display.init();
	factory_display.clear();
	factory_display.display();
  
	attachInterrupt(0,interrupt_GPIO0,FALLING);
	
	rxLoraMessage ="...waiting LoRa data";
  factory_display.drawString(0, 10, rxLoraMessage);
  factory_display.display();
  lora_init();  // LORA Init
  delay(800);
  factory_display.clear();

  drawGLCD_Fixed_Elements();

	pinMode(LED ,OUTPUT);
	digitalWrite(LED, LOW);  
  randomSeed(analogRead(0));

  String empty="";
  empty.toCharArray(txpacket, sizeof(txpacket));

  Radio.Rx(0);
  Radio.IrqProcess();
  Serial.println();
  Serial.println("\nSETUP COMPLETE: ");
  Serial.println(easyChipName.c_str());
  Serial.println();
  hasConnectedAfterSetup = false;
}



// ======== EFFICIENT GLCD UTILITIES ======
void refreshGLCD_Lora_Rx(String text) {
  wipeRow(8,1);
  drawString(8,1,text);
  wipeRow(8,2);
}
void refreshGLCD_Lora_Tx(String text) {
  wipeRow(8,2);
  drawString(8,2,text);
  wipeRow(8,1);
}
void refreshGLCD_LoraSignalStrength(String signalStrength) {
  wipeRow(0,3);
  drawString(0,3,signalStrength);
}
void refreshGLCD_BT_Rx(String text) {
  wipeRow(11,4);
  drawString(11,4,text);
  wipeRow(11,5);
}  
void refreshGLCD_BT_Tx(String text){ 
  wipeRow(11,5);
  drawString(11,5,text);
  wipeRow(11,4);
}


void drawString(int16_t horizontalOffsetInCharacters, int16_t rowNumber, String text){
  factory_display.drawString(horizontalOffsetInCharacters*6, rowNumber*10, text);
  factory_display.display();
}
void wipeRow(int16_t horizontalOffsetInCharacters, int16_t rowNumber) {
    // Define the area to clear
  int16_t xOffsetInPixels = horizontalOffsetInCharacters * 6;  // X-coordinate of the top-left corner of the area to clear
  int16_t yOffsetInPixels = rowNumber * 10;     // Y-coordinate of the top-left corner of the area to clear
  int16_t clearWidthInPixels = 128-xOffsetInPixels; // Width of the area to clear
  int16_t clearHeightInPixels = 12; // Height of the area to clear
  factory_display.setColor(BLACK);
  factory_display.fillRect(xOffsetInPixels, yOffsetInPixels+1, clearWidthInPixels, clearHeightInPixels);
  factory_display.setColor(WHITE);
  factory_display.display();
}
void drawGLCD_Fixed_Elements() {
    // Draws GLCD elements that are rarely painted
    displayEasyChipName(); // Draws Arduino Device details in 1st row: Name and Serial Number
    drawString(0, 1, "LoRa Rx:");
    drawString(0, 2, "LoRa Tx:");
    drawString(0, 4, "BT<-AND Rx:");
    drawString(0, 5, "BT->AND Tx:"); 
}
// ^^^^^^^^^ EFFICIENT GLCD UTILITIES ^^^^^^^^^^



void dispatchLoRaTransmissionBuffer(char* txt) {

  
  char txPacket[BUFFER_SIZE];
  if (txt[0] != '\0') {
    sprintf(txPacket, "%s,user=%s", txt, easyChipName);
    Serial.println();
    Serial.println();
    Serial.printf("%s sending packet \"%s\" , length %d", easyChipName, txPacket, strlen(String(txPacket).c_str()));
    Serial.println(); 
    refreshGLCD_Lora_Tx(txPacket);

    // TRANSMIT packet over LoRa radio
    Radio.Send( (uint8_t *)txPacket, strlen(txPacket) );
    refreshAndLogLoraRadioStatus();

    // POST Transmission:
    blink = ONCE;
  }
  strcpy(txPacket, "");
  // Ensure null-termination
  txPacket[0] = '\0';
}

String mockWantsToTransmitMessage(int16_t syncIndex) {
  int randomNumber = random(50);
  if(randomNumber == 7) {
    String txMessage = "Hi chiplet " + String(syncIndex);
    Serial.println();
    Serial.printf("%s generated MOCK LoRa Tx Message", easyChipName);
    Serial.println();
    delay(100);
    return txMessage;
  }
  return "";
}



String loraRadioStatus = "";
String getLoraRadioStatus() {
  RadioState_t status = Radio.GetStatus();
  switch (status) {
    case RF_IDLE:
      return "LORA IDLE";
      break;
    case RF_RX_RUNNING:
      return "LORA RX";
      break;
    case RF_TX_RUNNING:
      return "LORA TX";
      break;
    case RF_CAD:
      return "LORA CAD";
      break;
    default:
      return "LORA Unknown";
      break;
  }
}
void refreshAndLogLoraRadioStatus() {
	if(loraRadioStatus != getLoraRadioStatus()) {
    loraRadioStatus = getLoraRadioStatus();
    Serial.printf("Lora Radio Status: %s",getLoraRadioStatus());
  }
}


void loop() {
  refreshAndLogLoraRadioStatus();
  Radio.IrqProcess();

  switch(blink) {
    case TWICE:  // Rx
      turnActivityLED_ON();  delay(40);
      turnActivityLED_OFF(); delay(70);
      turnActivityLED_ON();  delay(40);
      break;
    case ONCE:  // Tx
      // delay(100); in mock mode
      turnActivityLED_ON();  delay(100);
      turnActivityLED_OFF(); delay(50);
      break;
    default:
      delay(150);
      break;
  }

  turnActivityLED_OFF();
  blink = ZERO;

  // TICK THE CLOCK, increment the i iteration number of the Main Loop
  //Increment up to 99, reset to 0 when number passes 99
  if (i_mainLoop + 1 > 99) {
    i_mainLoop = 0;
  } else {
    i_mainLoop++;
  }

  if(hasConnectedAfterSetup == true && isBluetoothConnected == false) {
     Serial.println("BLE rethrottling device connection");
     ESP.restart();
  }
} 





