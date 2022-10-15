#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
 
// REPLACE WITH RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xBC, 0xFF, 0x4D, 0x18, 0xBB, 0x31};
 
// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int16_t gear;
} struct_message;
struct_message Gear;
 
// variable declaration
int8 prev_gear = 0;

// pin declaration
const int8_t analog_pin = A0;
const int8_t control_x_pin = D7;
const int8_t control_y_pin = D8;
const int8_t status_led_pin = D5;
const int8_t reverse_pin = D2;
const int8_t CS = D1;
 

// Interrupt Service Routine
volatile bool spi_active = false;
volatile int reverse_state = 0;

IRAM_ATTR void ISR() {
  // not ideal but in this case sufficient. quickly reads the state of the reverse pin if CS is pulled high.
  reverse_state = digitalRead(reverse_pin);
  spi_active = true;
}


// Callback when data is sent
// void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
//   Serial.print("Last Packet Send Status: ");
//   if (sendStatus == 0){
//     Serial.println("Delivery success");
//   }
//   else{
//     Serial.println("Delivery fail");
//   }
// }
  
// maps analog values to their corresponding gears (-1 to 6)
int8_t map_gear(int x,int y) {
  int y_low = 180;
  int y_high = 750;
  int x_low = 310;
  int x_mid = 330;
  int x_high = 600;

  if (y>y_high) {
    if (x<x_low) {
      return 1; 
    } else if (x<x_high and x>x_mid) {
      return 3;
    }else {
      return 5;
    }
  }
  if (y<y_low) {
    if (x<x_low) {
      return 2;
    }else if (x<x_high and x>x_mid) {
      return 4;
    }else {
      // check if shifter is pushed
      if (!spi_active) {
        reverse_state = digitalRead(reverse_pin);
      }
      // if SPI is used, use volatile variable from interrupt
      if (reverse_state == 1) {
        return -1;
      } else {
        return 6;
      }
    }
  }
  //neutral
  return 0; 
}

// reads analog value from first channel
int16_t readX(void) {
  int sensorValue = 0;
  digitalWrite(control_y_pin, HIGH);
  digitalWrite(control_x_pin, LOW);
  delay(5);
  sensorValue = analogRead(analog_pin);
  digitalWrite(control_x_pin, HIGH);
  return sensorValue;
}

// reads analog value from second channel
int16_t readY(void) {
  int sensorValue = 0;
  digitalWrite(control_x_pin, HIGH);
  digitalWrite(control_y_pin, LOW);
  delay(5);
  sensorValue = analogRead(analog_pin);
  digitalWrite(control_y_pin, HIGH);
  return sensorValue;
}

// print x and y coordinates in excel readable format
void printXY(int x, int y ){
  Serial.print(x);
  Serial.print("\t");
  Serial.println(y);
}

void setup() {

  // configure MUX pins
  pinMode(control_x_pin, OUTPUT);
  pinMode(control_y_pin, OUTPUT);
  // deactivate MUX ASAP
  digitalWrite(control_x_pin, LOW);
  digitalWrite(control_y_pin, LOW);
  // configure pins
  pinMode(reverse_pin, INPUT);
  pinMode(status_led_pin, OUTPUT);

  // Init Serial Monitor
  Serial.begin(115200);

  // create interrupt
  attachInterrupt(digitalPinToInterrupt(CS), ISR, RISING);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
 
  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
 
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  // esp_now_register_send_cb(OnDataSent);
   
  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
}

 
void loop() {
  // read from adc
  uint16 x = readX();
  uint16 y = readY();
  // map to gear
  int8 gear = map_gear(x,y);

  // prints ADC values to the terminal
  // printXY(x,y);

  // only send if gear has changed
  if (prev_gear != gear) {
    Gear.gear = gear;

    // resend packet until packet is received: 0 -> delivery successful
    int8_t rcv_confirm = 0;
    do {
      rcv_confirm = esp_now_send(broadcastAddress, (uint8_t *) &Gear, sizeof(Gear));
      digitalWrite(status_led_pin, HIGH);
      // Serial.print("Value: ");
      // Serial.println(Gear.gear);
    } while(rcv_confirm!=0);
    prev_gear = gear;
  }
  // wait for 5ms -> polling rate = 200Hz
  delay(5);
  digitalWrite(status_led_pin, LOW);
}