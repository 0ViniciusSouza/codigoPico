#include "node.h"
#include "mcp2515.h"
#include "consensus.h"
#include <hardware/flash.h> //for flash_get_unique_id

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

//consensus algorithm
int maxiter = 50;
struct Node this_node{0,{0,0,0},{0,0,0},{0,0,0},{0,0,0},0,0,{0,0,0},0,0};
float rho = 0.07;
int states[3] {0,0,0};

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
float y00;

//median filter size
const int filter_size{7};

//reference and current value read
float r {0.0}; //reference
float y {0.0}; //current read of sensor


uint8_t ad1, ad2;



void setup(){
  flash_get_unique_id(pico_flash_id);
  node_address = pico_flash_id[7];
  //order the addresses
  if(     node_address == 0x29){ad1 = 0x32; ad2 = 0x34; this_node.index=0; b = 2.9306;}
  else if(node_address == 0x32){ad1 = 0x29; ad2 = 0x34; this_node.index=1; b = 3.1694;} 
  else if(node_address == 0x34){ad1 = 0x29; ad2 = 0x32; this_node.index=2; b = 3.2067;}
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
  if(this_node.index == 0) send_wakeup_order();
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
      else if(idx == this_node.index){//LED turn on at max
        Serial.print("lendo valor "); Serial.println(idx);
        analogWrite(ledPin, 4095);
        delay(6000);
        this_node.k[this_node.index] = (read_value_y()-y00)/4095;
        analogWrite(ledPin, 0);
        Serial.print("leu valor proprio ");Serial.println(idx);Serial.println();
      }
      else{ //other nodes turn their led on
        Serial.print("lendo valor "); Serial.println(idx);
        analogWrite(ledPin, 0);
        delay(6000);
        this_node.k[idx] = (read_value_y()-y00)/4095.0;
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

      if(i == this_node.index){
        Serial.print("lendo valor proprio "); Serial.println(i);
        analogWrite(ledPin, 4095);
        delay(6000);
        this_node.k[this_node.index] = (read_value_y()-y00)/4095;
        analogWrite(ledPin, 0);
        Serial.print("leu valor proprio"); Serial.println(i);Serial.println();
      }

      else{
        Serial.print("lendo valor "); Serial.println(i);
        analogWrite(ledPin, 0);
        delay(6000);
        this_node.k[i] = (read_value_y()-y00)/4095;
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
  else if(array[0] == "k" && count == 1){
    Serial.println("this_node.k = ");
    for(int i =0; i<3; i++) Serial.println(this_node.k[i]);
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

float consensus(){
  float d1[3]{0,0,0};
  float d2[3]{0,0,0};
  float d3[3]{0,0,0};
  for(int i =0; i<maxiter; i++){
    //minimize augmented lagragian with respect to di for the current values of yi and d_av
    float cost1 = consensus_iterate(this_node, rho, 3, d1);
    for(int j=0; j<3; j++){
      this_node.d[j] = d1[j];
    }
    //exchanges solution 

    for(int track_state = 0; track_state<3;track_state++){
      int msg_sent{0};
      int msg_received1{0};
      int msg_received2{0};
      while(msg_sent < 3 && msg_received1 <3 && msg_received2 <3){
        if(millis() >= time_to_write && msg_sent < 3 && states[track_state] == 1 ) {// && (send_ack1 == false || send_ack2 == false) && msg_sent < 3 ){
            //creates the message
            unsigned long message = this_node.d[msg_sent];
            canMsgTx.can_id = node_address;
            canMsgTx.can_dlc = 5;
            uint8_t command = 0b11100;
            uint8_t node_pos;
            if(msg_sent == 0) node_pos= 0x0;
            else if(msg_sent == 1) node_pos = 0x1;
            else if (msg_sent == 2) node_pos = 0x2;
            uint8_t response = 0x0;
            uint8_t iteration = (byte)i;
            int int_b_value = this_node.d[msg_sent]*1000;
            //divide into the proper bytes to transmit
            uint8_t b_value_1 = int_b_value >> 9; 
            uint8_t b_value_2 = int_b_value >> 1;
            uint8_t b_value_3 = (int_b_value & 0x1) << 7;
            canMsgTx.data[0] = ( (node_pos << 6))|((response << 5)) |(command);
            canMsgTx.data[1] = iteration;
            canMsgTx.data[2] = b_value_1;
            canMsgTx.data[3] = b_value_2;
            canMsgTx.data[4] = b_value_3;
            //sends the message
            noInterrupts();
            err = can0.sendMessage(&canMsgTx);
            interrupts();
            // Serial.print("Sending message ");
            // Serial.print( msg_sent );
            // Serial.print(" from node ");
            // Serial.print( node_address, HEX );
            // Serial.print(" for iteration ");
            // Serial.println(i);
            // for (int k=0 ; k < canMsgTx.can_dlc ; k++)
            //   Serial.print( canMsgTx.data[ k ], BIN), Serial.print(" ");
            // Serial.println();
            msg_sent++;
            time_to_write = millis() + write_delay;
        }

        if(data_available && states[track_state] == 0){ //recebeu mensagem e esta no estado de receber
          noInterrupts();
          can_frame frm {canMsgRx}; //local copy
          interrupts();
          //receives data from different node
          if((frm.data[0] & 0b00111111) == 0b00011100 && frm.can_dlc == 5){
            if((int)frm.data[1] == i){//same iteration
                float value = ((uint32_t) (frm.data[2] <<  9) | (uint32_t) (frm.data[3]  << 1) | (uint32_t) (frm.data[4]  >> 7))/1000.0; 
                int idx = frm.data[0] >> 6;
                            //    Serial.println();Serial.print("Iteracao = "); Serial.println(idx); Serial.println();
                if(frm.can_id == ad1){
              //    Serial.println("Recebeu dado no 1");
                  d2[idx] = value;
                  msg_received1++;
                }
                else if(frm.can_id == ad2){
                //  Serial.println("Recebeu dado no 2");
                  d3[idx] = value;
                  msg_received2++;
                }
            }
          }
        data_available = false;
        }
      }
    }
  //compute the average of each Node's solution
  for(int j=0; j<3; j++){
      this_node.d_av[j] = (this_node.d[j]+d2[j]+d3[j])/3;
    }
  //update the Lagrange multipliers
    for(int j=0; j<3; j++){
      this_node.y[j] = this_node.y[j] + rho*(this_node.d[j]-this_node.d_av[j]);
    }
  }
  //solucao dos duty cycles
  Serial.println("Solucao final dos duty cycles: ");
    for(int j=0;j<3;j++){
      Serial.println(this_node.d_av[j]);
    }

  //solucao que indica a referencia que cada no precisa seguir
  float reference = 0;
  //mulitiplicar K*d+o
  reference = this_node.k[0]*this_node.d_av[0] + this_node.k[1]*this_node.d_av[1] + this_node.k[2]*this_node.d_av[2] + this_node.o; 
  Serial.println();
  Serial.print("Referencia final do no:  ");
  Serial.println(reference);
  return reference;  
}
