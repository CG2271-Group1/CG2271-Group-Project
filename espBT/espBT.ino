#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;

#define RXD2 16
#define TXD2 17

char buf[9];

void setup() {
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
    SerialBT.begin("2271ESP32_t1"); //Bluetooth device name
    Serial.println("The device started, now you can pair it with bluetooth!");
}

/* motion: 
 first 4 bits: 0001
 last 4 bits:
    0001 forward
    0010 left
    0011 back
    0100 right
    0101 stop
    0110 accelerate
    0111 slow down
*/
void loop() {
    if(SerialBT.available()) {
        String command = SerialBT.readStringUntil('\n');
        Serial.println(command);
        char cc = command[0];
        switch(cc) {
            case 'w':
                Serial.println(0x11);
                break;
            case 'a':
                Serial.println(0x12);
                break;
            case 's':
                Serial.println(0x13);
                break;
            case 'd':
                Serial.println(0x14);
                break;
            case 'e':
                Serial.println(0x15);
                break;
            case 'i':
                Serial.println(0x16);
                break;
            case 'k':
                Serial.println(0x15);
                break;
        }
    }
}
