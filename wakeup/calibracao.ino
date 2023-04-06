#include <hardware/flash.h> //for flash_get_unique_id
#include "mcp2515.h"


//for can bus communication
MCP2515 can0 {spi0, 17, 19, 16, 18, 10000000};
uint8_t this_pico_flash_id[8], node_address;

struct can_frame canMsgTx, canMsgRx;

MCP2515::ERROR err;

uint8_t pico_flash_id[8];
unsigned long time_to_write, time_wait;
unsigned long write_delay {10};

const byte interruptPin {20};
volatile byte data_available {false};

//variables to read the commands passed by serial monitor
String strs[4]; 
String inputString = "";         // a String to hold incoming data
int StringCount {0};

//the interrupt service routine
void read_interrupt(uint gpio, uint32_t events) {
  can0.readMessage(&canMsgRx);
  data_available = true;
}


//LED and LDR
const int ldrPin = 26;    // select the input pin for the LDR
const int ledPin = 15;      // select the pin for the LED
float b; //value is defined in the setup
const float m{-0.8}; //from datasheet
const float R{10}; //resistance in kOhms
float ks[3]; //gains for luminaires
float y00;

//median filter size
const int filter_size{7};

//reference and current value read
float r {0.0}; //reference
float y {0.0}; //current read of sensor


uint8_t ad1, ad2;

int node_order;

void setup(){
  flash_get_unique_id(pico_flash_id);
  node_address = pico_flash_id[7];
  //order the addresses
  if(     node_address == 0x29){ad1 = 0x32; ad2 = 0x34; node_order=0; b = 2.9306;}
  else if(node_address == 0x32){ad1 = 0x29; ad2 = 0x34; node_order=1; b = 3.1694;} 
  else if(node_address == 0x34){ad1 = 0x29; ad2 = 0x32; node_order=2; b = 3.2067;}
  pinMode(ledPin, OUTPUT_12MA); //makes the output limited to 12mA
  Serial.begin(9600); //initialze Serial
  //add_repeating_timer_ms( -10, my_repeating_timer_callback, NULL, &timer); //100 Hz sample frequency of 100 Hz
  analogReadResolution(12); //default is 10
  analogWriteFreq(30000); //30KHz
  analogWriteRange(4095); //Max PWM
  can0.reset();
  can0.setBitrate(CAN_1000KBPS);
  can0.setNormalMode();
  gpio_set_irq_enabled_with_callback(interruptPin, GPIO_IRQ_EDGE_FALL, true, &read_interrupt );
  time_to_write = millis() + write_delay;
  if(node_order == 0) send_wakeup_order();
}

void loop(){
  if (Serial.available() > 0) {
    // read the incoming string
    inputString = Serial.readString();
    split_array_string(inputString, strs);
    basic_commands(strs, StringCount);
    // clear the string:
    inputString = "";   
  }

  if( data_available ){
   // noInterrupts();
    can_frame frm {canMsgRx}; //local copy
   // interrupts();
    data_available = false;
    Serial.print("Received message from node ");
    Serial.print( canMsgRx.can_id , HEX);
    Serial.print(" : ");
    Serial.println(" ");
    //Serial.println();

    if(frm.data[0] == 0b11111100){ 
      Serial.println("Comando consensus recebido por can bus");
      Serial.println();
    //  duty = consensus();
    }

    else if((frm.data[0] & 0b00011111) == 0b00011111){
      Serial.println("Comando de wakeup recebido");
      int idx = frm.data[0] >> 6;
      Serial.print("Idx = "); Serial.println(idx);
      if(idx == 3){ //all leds turned off
        Serial.println("lendo valor y00");
        analogWrite(ledPin, 0); //turn off led
        delay(6000); //delay de 6s
        y00 = read_value_y();
        Serial.println("leu valor y00");
        Serial.println();
      }
      else if(idx == node_order){//LED turn on at max
        Serial.print("lendo valor "); Serial.println(idx);
        analogWrite(ledPin, 4095);
        delay(6000);
        ks[node_order] = (read_value_y()-y00)/4095;
        analogWrite(ledPin, 0);
        Serial.print("leu valor proprio ");Serial.println(idx);Serial.println();
      }
      else{ //other nodes turn their led on
        Serial.print("lendo valor "); Serial.println(idx);
        analogWrite(ledPin, 0);
        delay(6000);
        ks[idx] = (read_value_y()-y00)/4095.0;
        Serial.print("leu valor "); Serial.println(idx);Serial.println();
      }
    }
  }
}

void send_wakeup_order(){
  canMsgTx.can_dlc = 1;
  canMsgTx.can_id = node_address;
  canMsgTx.data[0] = 0b11111111;
  noInterrupts();
  err = can0.sendMessage(&canMsgTx);
  interrupts();
  //turn off the LED and read the value
  Serial.println("lendo valor y00");
  analogWrite(ledPin, 0); //turn off led
  delay(6000); //delay de 6s
  y00 = read_value_y();
  Serial.println("leu valor y00");
  int i = 0;
  while(i<3){
    if(millis() >= time_to_write){
      if(i == 0){
        Serial.println("Mandando a primeira mensagem"); Serial.println();
        canMsgTx.can_dlc = 1;
        canMsgTx.can_id = node_address;
        canMsgTx.data[0] = 0b00111111;
        //sends the message
        noInterrupts();
        err = can0.sendMessage(&canMsgTx);
        interrupts();
      }
      else if(i == 1){
        Serial.println("Mandando a segunda mensagem"); Serial.println();
        canMsgTx.can_dlc = 1;
        canMsgTx.can_id = node_address;
        canMsgTx.data[0] = 0b01111111;
        //sends the message
        noInterrupts();
        err = can0.sendMessage(&canMsgTx);
        interrupts();
      }
      else if(i == 2){
        Serial.println("Mandando a terceira mensagem"); Serial.println();
        canMsgTx.can_dlc = 1;
        canMsgTx.can_id = node_address;
        canMsgTx.data[0] = 0b10111111;
        //sends the message
        noInterrupts();
        err = can0.sendMessage(&canMsgTx);
        interrupts();
      }

      if(i == node_order){
        Serial.print("lendo valor proprio "); Serial.println(i);
        analogWrite(ledPin, 4095);
        delay(6000);
        ks[node_order] = (read_value_y()-y00)/4095;
        analogWrite(ledPin, 0);
        Serial.print("leu valor proprio"); Serial.println(i);Serial.println();
      }

      else{
        Serial.print("lendo valor "); Serial.println(i);
        analogWrite(ledPin, 0);
        delay(6000);
        ks[i] = (read_value_y()-y00)/4095;
        Serial.print("leu valor "); Serial.println(i);Serial.println();
      }
    time_to_write = millis() + write_delay;
    i++;
    }
  }
}


void basic_commands(String array[], int count){
  if(array[0] == "r" && count == 1){
    send_wakeup_order();
  }
}

void split_array_string(String input, String array[]){
  StringCount = 0;
    //Split the string into substrings
  while (input.length() > 0)
  {
    int index = input.indexOf(' ');
    if (index == -1) // No space found
    {
      array[StringCount++] = input;
      break;
    }
    else
    {
      array[StringCount++] = input.substring(0, index);
      input = input.substring(index+1);
    }
  }
}

//read the value of LDR
float read_value_y() {
  //Median Filter
  int filter_values[filter_size];
  for(int i =0; i<filter_size; i++){
        int rawMeasure = analogRead(ldrPin); // variable to store the value coming from the sensor
        filter_values[i] = rawMeasure;
  }
  int ldrValue = compute_median(filter_values, filter_size);
  float Vadc = ldrValue*3.3/4096; //voltage in the LDR
  float Rldr = 3.3*R/(Vadc) - R; //resistance LDR kohms
  return pow(10,(log10(Rldr)-b)/m); //convert from resistance to LUX (y)
}

//compute the median
int compute_median(int array[], int filter_size){
  //sorting
  for(int i=0; i<filter_size-1;i++){
    for(int j = i+1; j<filter_size; j++){
      if(array[j] < array[i]){
        int temp = array[i];
        array[i] = array[j];
        array[j] =  temp;
      }
    }
  }
  //median
  int median_index = filter_size/2;
  if(filter_size%2 != 0) return array[median_index];
  return (array[median_index - 1] + array[median_index])/2;
}


