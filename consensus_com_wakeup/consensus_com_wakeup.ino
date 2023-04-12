#include "node.h"
#include "mcp2515.h"
#include "consensus.h"
#include "command_handler.h"
#include "circular_buffer.h"
#include <hardware/flash.h> //for flash_get_unique_id

//for can bus communication
MCP2515 can0 {spi0, 17, 19, 16, 18, 10000000};
uint8_t this_pico_flash_id[8], node_address;

struct can_frame canMsgTx, canMsgRx;

MCP2515::ERROR err;

uint8_t pico_flash_id[8];


const byte interruptPin {20};
volatile byte data_available {false};

//variables to read the commands passed by serial monitor
String strs[4]; 
String inputString = "";         // a String to hold incoming data
int StringCount {0};

//consensus algorithm
int maxiter = 100;
struct Node this_node{
  0, //ID
  {0,0,0},  //DUTY
  {0,0,0},  //AV_DUTY
  {0,0,0},  //Y
  {0,0,0},  //K
  0,  //N
  0,  //M
  {0,0,0},  //C
  20,  //O
  100,  //L
  0   //r
};


float rho = 1;
int states[3] {0,0,0};

//the interrupt service routine
void read_interrupt(uint gpio, uint32_t events) {
  can0.readMessage(&canMsgRx);
  data_available = true;
}


//LED and LDR
const int ldrPin = 26;    // select the input pin for the LDR
const int ledPin = 15;      // select the pin for the LED
float b = 2.9306;    //value is defined in the setup
//
const float m{-0.8}; //from datasheet
const float R{10}; //resistance in kOhms
float y00;

//median filter size
const int filter_size{7};

//reference and current value read
float r {0.0}; //reference
float y {0.0}; //current read of sensor


uint8_t ad1, ad2;

unsigned long time_wakeup, time_order, time_wait;
unsigned long write_delay {10};
unsigned long order_delay {1000};
unsigned long time_consensus;
unsigned long consensus_delay {15};
float ref;

uint8_t node_ids[3][8];
bool firstloop = 1;
bool sleeping = 1;
bool consensus_flag = 0;
int N_nodes = 1;

void setup(){
  flash_get_unique_id(pico_flash_id);
  for(int i = 0; i < 8; i++) node_ids[0][i] = pico_flash_id[i];
  
  pinMode(ledPin, OUTPUT_12MA); //makes the output limited to 12mA
  Serial.begin(9600); //initialze Serial
  
  analogReadResolution(12); //default is 10
  analogWriteFreq(30000); //30KHz
  analogWriteRange(4095); //Max PWM
  
  can0.reset();
  can0.setBitrate(CAN_1000KBPS);
  can0.setNormalMode();
  gpio_set_irq_enabled_with_callback(interruptPin, 
                                     GPIO_IRQ_EDGE_FALL, 
                                     true, 
                                     &read_interrupt );

  time_order = millis() + order_delay;
  time_wakeup = millis() + write_delay;
  time_consensus = millis() + consensus_delay;
}

void loop(){
  //order nodes
  if(firstloop){
    if( millis() >= time_order) {
      canMsgTx.can_id = 0;
      canMsgTx.can_dlc = 8;
      Serial.print("This node ID: ");
      for(int i = 0; i < 8; i++){
        canMsgTx.data[i] = pico_flash_id[i];
        Serial.print(pico_flash_id[i], HEX); Serial.print(" ");
      }
      Serial.println();
        
      noInterrupts();
      err = can0.sendMessage(&canMsgTx);
      interrupts();

      if(N_nodes == 3){
        delay(1000);
        
        int larger_than = 0;

        for(int i = 1; i < 3; i++){
          for(int j = 0; j < 8; j++){
            if(node_ids[0][j] > node_ids[i][j]){
              larger_than++;
              break;
            }
            else if(node_ids[0][j] < node_ids[i][j]){
              break;
            }
          } 
        }
        
        this_node.index = larger_than;

        states[this_node.index] = 1;

        this_node.c[this_node.index] = 1;

        Serial.print("Node_ID: ");
        Serial.println(this_node.index);

        firstloop = 0;
      }

      time_order = millis() + order_delay;
    }

    if( data_available ) {
        noInterrupts();
        can_frame frm {canMsgRx}; //local copy
        interrupts();
        
        Serial.print("ID recebido: ");
        for(int i = 0; i < 8; i++) {
            Serial.print(frm.data[i], HEX); Serial.print(" ");
          }
        Serial.println();

        if(N_nodes == 1){ 
          for(int i = 0; i < 8; i++) {
            node_ids[N_nodes][i] = frm.data[i];
            Serial.print(frm.data[i], HEX); Serial.print(" ");
          }
          Serial.println();
          N_nodes++;
        }
        else if (N_nodes == 2) {
          bool different = 0;
          for(int i = 7; i >= 0; i--) {
            different |= (node_ids[1][i] != frm.data[i]);
            if (different) break;
          }
          if (different){
            for(int i = 0; i < 8; i++) {
              node_ids[N_nodes][i] = frm.data[i];
              Serial.print(frm.data[i], HEX); Serial.print(" ");
            }
            Serial.println();
            N_nodes++;
          }
        }
        
        data_available = false;
    }
    
  } 
  else {
    if(sleeping && this_node.index == 0){ //starts the wakeup
      delay(1000);
      send_wakeup_order();
      sleeping = 0;

      command can_command {
        "C",
        1,
        29,
        0,
        this_node.index,
        3
      };

      canMsgTx = ch::command2frame(can_command);

      noInterrupts();
      err = can0.sendMessage(&canMsgTx);
      interrupts();

      consensus_flag = true;
    }
    
    if (Serial.available() > 0) { //receives message via Serial
      // read the incoming string
      inputString = Serial.readString();
      split_array_string(inputString, strs);
      basic_commands(strs, StringCount);
      // clear the string:
      inputString = "";   
    }

    if(consensus_flag && millis() >= time_consensus){  //calculates the consensus 
      Serial.println("Calculando consensus");
      Serial.println();
      this_node.r = consensus();

      consensus_flag = false;

      time_consensus = millis() + consensus_delay;
    }
  
    if( data_available ){ //receives data from the can bus
     // noInterrupts();
      can_frame frm {canMsgRx}; //local copy
     // interrupts();
      data_available = false;
  
      command can_command{ch::frame2command(frm)};
  
      Serial.print( this_node.index );
      Serial.print(" - Received command - ");
      Serial.print( can_command.message );
      Serial.print(" - from node ");
      Serial.print( can_command.from_desk_id );
      Serial.print(" - can_id - ");
      Serial.print(frm.can_id,BIN);
      Serial.println();
      //Serial.println();
      if(can_command.command_num == 2 && can_command.isRequest){
        consensus_flag = true;
      }
  
      if(can_command.command_num == 29 && can_command.isRequest){ 
        consensus_flag = !consensus_flag;
        if (consensus_flag) Serial.println("Ativando consensus");
        else Serial.println("Desativando consensus");
        Serial.println();

      }
  
      else if(can_command.command_num == 28 && can_command.isRequest){
        Serial.println("Comando de wakeup recebido");
        int idx = can_command.to_desk_id;
        Serial.print("Idx = "); Serial.println(idx);
        if(idx == 3){ //all leds turned off
          Serial.println("lendo valor y00");
          analogWrite(ledPin, 0); //turn off led
          delay(6000); //delay de 6s
          y00 = read_value_y();
          this_node.o = y00;
          Serial.print("leu valor y00 = ");
          Serial.print(y00);
          Serial.println();
          this_node.o = y00;
        }
        else if(idx == this_node.index){//LED turn on at max
          Serial.print("lendo valor "); Serial.println(idx);
          analogWrite(ledPin, 4095);
          delay(6000);
          this_node.k[this_node.index] = (read_value_y()-y00)/100.0;
          analogWrite(ledPin, 0);
          Serial.print("leu valor proprio ");Serial.println(idx);Serial.println();
        }
        else{ //other nodes turn their led on
          Serial.print("lendo valor "); Serial.println(idx);
          analogWrite(ledPin, 0);
          delay(6000);
          this_node.k[idx] = (read_value_y()-y00)/100.0;
          Serial.print("leu valor "); Serial.println(idx);Serial.println();
        }
       this_node.n = pow(this_node.k[0],2) + pow(this_node.k[1],2) + pow(this_node.k[2],2);
       this_node.m = this_node.n - pow(this_node.k[this_node.index],2);
      }
    }
  }
}

void send_wakeup_order(){
  command can_command {
    "w ",
    1,
    28,
    0,
    this_node.index,
    3
  };

  canMsgTx = ch::command2frame(can_command);
  
  noInterrupts();
  err = can0.sendMessage(&canMsgTx);
  interrupts();
  //turn off the LED and read the value
  Serial.println("lendo valor y00");
  analogWrite(ledPin, 0); //turn off led
  delay(6000); //delay de 6s
  y00 = read_value_y();
  Serial.print("leu valor y00 = ");
  Serial.print(y00);
  this_node.o = y00;
  
  Serial.print("Node_ID: ");
  Serial.println(this_node.index);
  
  int i = 0;
  while(i<3){
    if(millis() >= time_wakeup){
      Serial.print("Mandando a ");Serial.print(i);
      Serial.print("a. mensagem"); Serial.println();

      command can_command {
          "w " + String(i),
          1,
          28,
          0,
          this_node.index,
          i
        };

      canMsgTx = ch::command2frame(can_command);
      //sends the message
      noInterrupts();
      err = can0.sendMessage(&canMsgTx);
      interrupts();

      if(i == this_node.index){
        Serial.print("lendo valor proprio "); Serial.println(i);
        analogWrite(ledPin, 4095);
        delay(6000);
        this_node.k[this_node.index] = (read_value_y()-y00)/100.0;
        analogWrite(ledPin, 0);
        Serial.print("leu valor proprio"); Serial.println(i);Serial.println();
      }

      else{
        Serial.print("lendo valor "); Serial.println(i);
        analogWrite(ledPin, 0);
        delay(6000);
        this_node.k[i] = (read_value_y()-y00)/100.0;
        Serial.print("leu valor "); Serial.println(i);Serial.println();
      }
    time_wakeup = millis() + write_delay;
    i++;
    }
  }

  this_node.n = pow(this_node.k[0],2) + pow(this_node.k[1],2) + pow(this_node.k[2],2);
  this_node.m = this_node.n - pow(this_node.k[this_node.index],2);

  Serial.println("this_node.k = ");
  for(int i =0; i<3; i++) Serial.println(this_node.k[i]);
}


void basic_commands(String array[], int count){
  if(array[0] == "r" && count == 1){
    send_wakeup_order();
  }
  else if(array[0] == "k" && count == 1){
    for(int i =0; i<3; i++){ Serial.print("float k");Serial.print((this_node.index + 1));Serial.print(i+1);Serial.print(" = ");Serial.println(this_node.k[i],7);}
    Serial.print("float o"); Serial.print((this_node.index + 1));Serial.print(" = "); Serial.println(this_node.o,7);
    Serial.print("float L");Serial.print((this_node.index + 1));Serial.print(" = "); Serial.println(this_node.L,7);
    Serial.print("float n = "); Serial.println(this_node.n);
    Serial.print("float m = "); Serial.println(this_node.m);
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


// FALTA ATUALIZAR ILUMINAÇÃO EXTERNA A CADA CONSENSUS
float consensus(){
  float d1[3]{0,0,0};
  float d2[3]{0,0,0};
  float d3[3]{0,0,0};
  for(int i =0; i<maxiter; i++){
    //minimize augmented lagragian with respect to di for the current values of yi and d_av
    float cost1 = consensus_iterate(this_node, rho, 3, d1);
    Serial.println("Consenso do no");
    for(int j=0; j<3; j++){
      this_node.d[j] = d1[j];
      Serial.println(d1[j]);
    }
    //exchanges solution 

    for(int track_state = 0; track_state<3;track_state++){
      int msg_sent[3] {0};
      while(msg_sent[0] < 3 && msg_sent[1] < 3 && msg_sent[2] < 3){
        if(millis() >= time_consensus && msg_sent[this_node.index] < 3 && states[track_state] == 1 ) {// && (send_ack1 == false || send_ack2 == false) && msg_sent < 3 ){
            //creates the message
            unsigned long message = this_node.d[msg_sent[this_node.index]];
            
            uint8_t node_pos;
            if(msg_sent[0] == 0) node_pos= 0x0;
            else if(msg_sent[0] == 1) node_pos = 0x1;
            else if (msg_sent[0] == 2) node_pos = 0x2;
            uint8_t response = 0x0;
            uint8_t iteration = (byte)i;

//            uint8_t command = 0b11100;
//            int int_b_value = this_node.d[msg_sent]*1000;
//            //divide into the proper bytes to transmit
//            uint8_t b_value_1 = int_b_value >> 9; 
//            uint8_t b_value_2 = int_b_value >> 1;
//            uint8_t b_value_3 = (int_b_value & 0x1) << 7;
//
//            canMsgTx.can_id = node_address;
//            canMsgTx.can_dlc = 5;
//            canMsgTx.data[0] = ( (node_pos << 6))|((response << 5)) |(command);
//            canMsgTx.data[1] = iteration;
//            canMsgTx.data[2] = b_value_1;
//            canMsgTx.data[3] = b_value_2;
//            canMsgTx.data[4] = b_value_3;

            int data = this_node.d[msg_sent[this_node.index]]*1000;
            
            command can_command {
              String("C ") + i + String(" ") + data,
              0,
              29,
              (data << 8) + iteration,
              this_node.index,
              node_pos
            };

            canMsgTx = ch::command2frame(can_command);
            //sends the message
            noInterrupts();
            err = can0.sendMessage(&canMsgTx);
            interrupts();
            Serial.print("Sending message ");
            Serial.print( can_command.message );
            Serial.print(" from node ");
            Serial.print( can_command.from_desk_id );
            Serial.print(" for iteration ");
            Serial.print(i);
            Serial.print(" for position ");
            Serial.print(msg_sent[this_node.index]);
            Serial.print(" of value ");
            Serial.print(data/1000.0);
            Serial.println();
            msg_sent[this_node.index]++;
            time_consensus = millis() + write_delay;
        }

        if(data_available && states[track_state] == 0){ //recebeu mensagem e esta no estado de receber
          noInterrupts();
          can_frame frm {canMsgRx}; //local copy
          interrupts();
          
          command can_command{ch::frame2command(frm)};
          int frm_iteration = (can_command.data & 0b11111111);
          //receives data from different node
          if(can_command.command_num == 29 && !can_command.isRequest){
            if(frm_iteration == i){//same iteration
              float value = (can_command.data >> 8)/1000.0;
              int idx = can_command.to_desk_id;


              Serial.print("Recebeu dado do node ");
              Serial.print( can_command.from_desk_id );
              Serial.print(" para a iteracao ");
              Serial.print(frm_iteration); 
              Serial.print(" para a posicao ");
              Serial.print(msg_sent[can_command.from_desk_id]);
              Serial.print(" de valor ");
              Serial.println(value);
              Serial.println();
              

              if (can_command.from_desk_id == (this_node.index + 1)%3) d2[msg_sent[can_command.from_desk_id]] = value;
              if (can_command.from_desk_id == (this_node.index + 2)%3) d3[msg_sent[can_command.from_desk_id]] = value;
              
              msg_sent[can_command.from_desk_id]++;
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
