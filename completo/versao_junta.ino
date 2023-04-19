#include "command_handler.h"
#include <hardware/flash.h> //for flash_get_unique_id

int ledPin = 15; //2
int ldrPin = 26; //28

//Create a pid controller
pid my_pid {1}; //select which assembly
LED my_led {ledPin};
LDR my_ldr {ldrPin};
MedianFilter F{10};

Node my_node{ 
    my_led,
    my_ldr,
    my_pid,
    F
};

//for can bus communication
MCP2515 can0 {spi0, 17, 19, 16, 18, 10000000};
uint8_t this_pico_flash_id[8], node_address;

struct can_frame canMsgTx, canMsgRx;

MCP2515::ERROR err;

uint8_t pico_flash_id[8];


const byte interruptPin {20};
volatile byte data_available {false};

//consensus algorithm
int maxiter = 50;
float rho = 0.07;
int states[3] {0,0,0};
float y00;

Consenso_no my_consenso_no{
    my_node.id,
    {0,0,0},
    {0,0,0},
    {0,0,0},
    {0,0,0},
    my_node.n,
    my_node.m,
    {0,0,0},
    my_node.o,
    my_node.L
};

//the interrupt service routine
void read_interrupt(uint gpio, uint32_t events) {
  can0.readMessage(&canMsgRx);
  data_available = true;
}

unsigned long time_wakeup, time_order, time_wait;
unsigned long write_delay {10};
unsigned long order_delay {1000};
unsigned long time_consensus;
unsigned long consensus_delay {15};
uint8_t node_ids[3][8];
bool firstloop = 1;
bool sleeping = 1;
bool consensus_flag = 0;
int N_nodes = 1;

//variables to read the commands passed by serial monitor
String strs[4]; 
String inputString = "";         // a String to hold incoming data
String message_str;
int StringCount {0};

//alarm 
volatile unsigned long int timer_time {0};
volatile bool timer_fired {false};
struct repeating_timer timer;
bool my_repeating_timer_callback( struct repeating_timer *t ){
  if( ! timer_fired ){
    noInterrupts();
    timer_time = micros();
    timer_fired = true;
    interrupts();
  }
  return true;
}


//functions
void split_array_string(String input, String array[]);
void basic_commands(String array[], int count, int fromNode = 3);
void send_wakeup_order();



void setup() {
  flash_get_unique_id(pico_flash_id);
  for(int i = 0; i < 8; i++) node_ids[0][i] = pico_flash_id[i];

  pinMode(ledPin, OUTPUT_12MA); //makes the output limited to 12mA
  Serial.begin( 9600 ); //initialze Serial
  
  add_repeating_timer_ms( -10, my_repeating_timer_callback, NULL, &timer); //100 Hz
  
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

void loop() {
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
        
        my_node.id = larger_than;

        states[my_node.id] = 1;

        my_node.c[0] = 1;
        my_node.c[1] = 1;
        my_node.c[2] = 1;

        //calibrate LDR
        if(my_node.id == 1) my_node.my_ldr.b = 2.5455; //com14 - 2.9306
        else if (my_node.id == 2) my_node.my_ldr.b = 2.496; //com12 - 3.1694
        else if (my_node.id == 3) my_node.my_ldr.b = 2.51603; //com10 - 3.2067

        my_node.my_pid.setAssembly(my_node.id);


        Serial.print("Node_ID: ");
        Serial.println(my_node.id);

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
    if(sleeping && my_node.id == 0){ //starts the wakeup
      delay(1000);
      send_wakeup_order();
      sleeping = 0;

      command can_command {
        "C",
        1,
        29,
        0,
        my_node.id,
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

      message_str = inputString;
      basic_commands(strs, StringCount);
      // clear the string:
      inputString = "";   
    }

    if(consensus_flag && millis() >= time_consensus){  //calculates the consensus 
      Serial.println("Calculando consensus");
      Serial.println();
      my_node.r = consensus();
      my_node.u_consenso = my_consenso_no.c_d_av[my_node.id];

      consensus_flag = false;

      time_consensus = millis() + consensus_delay;
    }
  
    if( data_available ){ //receives data from the can bus
     // noInterrupts();
      can_frame frm {canMsgRx}; //local copy
     // interrupts();
      data_available = false;
  
      command can_command{ch::frame2command(frm)};
  
      Serial.print( my_node.id );
      Serial.print(" - Received command - ");
      Serial.print( can_command.message );
      Serial.print(" - from node ");
      Serial.print( can_command.from_desk_id );
      Serial.print(" - can_id - ");
      Serial.print(frm.can_id,BIN);
      Serial.println();
      //Serial.println();
      split_array_string(can_command.message, strs);

      message_str = can_command.message;

      if(can_command.isRequest) basic_commands(strs, StringCount, can_command.from_desk_id);
      
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
          y00 = my_node.my_ldr.read_value_y();
          my_node.o = y00;
          Serial.print("leu valor y00 = ");
          Serial.print(y00);
          Serial.println();
          my_node.o = y00;
        }
        else if(idx == my_node.id){//LED turn on at max
          Serial.print("lendo valor "); Serial.println(idx);
          analogWrite(ledPin, 4095);
          delay(6000);
          my_node.k[my_node.id] = (my_node.my_ldr.read_value_y()-y00)/100.0;
          analogWrite(ledPin, 0);
          Serial.print("leu valor proprio ");Serial.println(idx);Serial.println();
        }
        else{ //other nodes turn their led on
          Serial.print("lendo valor "); Serial.println(idx);
          analogWrite(ledPin, 0);
          delay(6000);
          my_node.k[idx] = (my_node.my_ldr.read_value_y()-y00)/100.0;
          Serial.print("leu valor "); Serial.println(idx);Serial.println();
        }
       my_node.n = pow(my_node.k[0],2) + pow(my_node.k[1],2) + pow(my_node.k[2],2);
       my_node.m = my_node.n - pow(my_node.k[my_node.id],2);
      }
    }

    if(timer_fired){
      noInterrupts();
      my_node.control();
      
      interrupts();
      timer_fired = false;
    }
  }
}

void send_wakeup_order(){
  command can_command {
    "w ",
    1,
    28,
    0,
    my_node.id,
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
  y00 = my_node.my_ldr.read_value_y();
  Serial.print("leu valor y00 = ");
  Serial.print(y00);
  my_node.o = y00;
  
  Serial.print("Node_ID: ");
  Serial.println(my_node.id);
  
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
          my_node.id,
          i
        };

      canMsgTx = ch::command2frame(can_command);
      //sends the message
      noInterrupts();
      err = can0.sendMessage(&canMsgTx);
      interrupts();

      if(i == my_node.id){
        Serial.print("lendo valor proprio "); Serial.println(i);
        analogWrite(ledPin, 4095);
        delay(6000);
        my_node.k[my_node.id] = (my_node.my_ldr.read_value_y()-y00)/100.0;
        analogWrite(ledPin, 0);
        Serial.print("leu valor proprio"); Serial.println(i);Serial.println();
      }

      else{
        Serial.print("lendo valor "); Serial.println(i);
        analogWrite(ledPin, 0);
        delay(6000);
        my_node.k[i] = (my_node.my_ldr.read_value_y()-y00)/100.0;
        Serial.print("leu valor "); Serial.println(i);Serial.println();
      }
    time_wakeup = millis() + write_delay;
    i++;
    }
  }

  my_node.n = pow(my_node.k[0],2) + pow(my_node.k[1],2) + pow(my_node.k[2],2);
  my_node.m = my_node.n - pow(my_node.k[my_node.id],2);

  Serial.println("my_node.k = ");
  for(int i =0; i<3; i++) Serial.println(my_node.k[i]);
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

void basic_commands(String array[], int count, int fromNode){
   bool send_to_CANBUS = false;
   String response = "";
   
   if(array[0] == "d" && count == 3){
      if (array[1].toInt() == my_node.id){
         my_node.feedback = false;
         response = my_node.my_led.write(array[2].toInt());

         Serial.println(response);
      } else {
         send_to_CANBUS = true; 
      }
   }
   else if(array[0] == "O" && count == 3){
      if (array[1].toInt() == my_node.id){
        my_node.L_occup = array[2].toInt();
        response = "ack";

        Serial.println(response);
      } else {
         send_to_CANBUS = true; 
      }
   }
   else if(array[0] == "U" && count == 3){
      if (array[1].toInt() == my_node.id){
        my_node.L_unoccup = array[2].toInt();
        response = "ack";

        Serial.println(response);
      } else {
         send_to_CANBUS = true; 
      }
   }
   else if(array[0] == "c" && count == 3){
      my_node.c[array[1].toInt()] = array[2].toInt();
      response = "ack";

      Serial.println(response);

      command can_command {
        String("c ") + array[1].toInt() + String(" ") + array[2].toInt(),
        1,
        26,
        array[2].toInt(),
        my_node.id,
        array[1].toInt()
      };

      canMsgTx = ch::command2frame(can_command);
      //sends the message
      noInterrupts();
      err = can0.sendMessage(&canMsgTx);
      interrupts();
    }


  else if(array[0] == "g" && count == 3){
    if(array[1] == "e"){
      if(array[2].toInt() == my_node.id){
        response = String("e ") + my_node.id + String(" ") + my_node.energy;
        
        Serial.println(response);
      }
      else {
        send_to_CANBUS = true;
      }
    }
    else if(array[1] == "v"){
      if(array[2].toInt() == my_node.id){
        response = String("v ") + my_node.id + String(" ") + my_node.visError;
        
        Serial.println(response);
      }
      else {
        send_to_CANBUS = true;
      }
    }
    else if(array[1] == "f"){
      if(array[2].toInt() == my_node.id){
        response = String("f ") + my_node.id + String(" ") + my_node.flickerError;

        Serial.println(response);
      }
      else {
        send_to_CANBUS = true;
      }
    }
    else if(array[1] == "d"){
      if(array[2].toInt() == my_node.id){
        response = String("d ") + my_node.id + String(" ") + my_node.my_led.dutyCycle;

        Serial.println(response);
      }
      else {
        send_to_CANBUS = true;
      }
    }
    else if(array[1] == "r"){
      if(array[2].toInt() == my_node.id){
        response = String("r ") + my_node.id + String(" ") + my_node.r;

        Serial.println(response);
      }
      else {
        send_to_CANBUS = true;
      }
    } 
    else if(array[1] == "l"){
      if(array[2].toInt() == my_node.id){
        response = String("l ") + my_node.id + String(" ") + my_node.my_ldr.read_value_y();

        Serial.println(response);
      }
      else {
        send_to_CANBUS = true;
      }
    }  
    else if(array[1] == "k"){
      if(array[2].toInt() == my_node.id){
        response = String("k ") + my_node.id + String(" ") + my_node.feedback;

        Serial.println(response);
      }
      else {
        send_to_CANBUS = true;
      }
    }
    else if(array[1] == "p"){
      if(array[2].toInt() == my_node.id){
        float P_led {0.108}; // in Watts
        response = String("p ") + my_node.id + String(" ") + P_led*my_node.my_led.dutyCycle;

        Serial.println(response);
      }
      else {
        send_to_CANBUS = true;
      }
    }
    else if(array[1] == "o"){
      if(array[2].toInt() == my_node.id){
        response = String("o ") + my_node.id + String(" ") + my_node.occupied;

        Serial.println(response);
      }
      else {
        send_to_CANBUS = true;
      }
    }
    else if(array[1] == "t"){
      if(array[2].toInt() == my_node.id){
        response = String("t ") + my_node.id + String(" ") + my_node.lastUpdateTime;

        Serial.println(response);
      }
      else {
        send_to_CANBUS = true;
      }
    }
    else if(array[1] == "x"){
      if(array[2].toInt() == my_node.id){
        float x = my_node.my_ldr.read_value_y() - my_node.k[my_node.id]*my_node.my_led.dutyCycle;

        response = String("x ") + my_node.id + String(" ") + x;

        Serial.println(response);
      }
      else {
        send_to_CANBUS = true;
      }
    }
    else if(array[1] == "a"){
      if(array[2].toInt() == my_node.id){
        response = String("a ") + my_node.id + String(" ") + my_node.antiwind;

        Serial.println(response);
      }
      else {
        send_to_CANBUS = true;
      }
    }
    else if(array[1] == "O"){
      if(array[2].toInt() == my_node.id){
        response = String("O ") + my_node.id + String(" ") + my_node.L_occup;

        Serial.println(response);
      }
      else {
        send_to_CANBUS = true;
      }
    }
    else if(array[1] == "U"){
      if(array[2].toInt() == my_node.id){
        response = String("U ") + my_node.id + String(" ") + my_node.L_unoccup;

        Serial.println(response);
      }
      else {
        send_to_CANBUS = true;
      }
    }
    else if(array[1] == "L"){
      if(array[2].toInt() == my_node.id){
        response = String("L ") + my_node.id + String(" ") + my_node.L;

        Serial.println(response);
      }
      else {
        send_to_CANBUS = true;
      }
    }
  }

  else if(array[0] == "g" && count == 3){
    if(array[1] == "c"){
      response = String("c ") + array[2].toInt() + String(" ") + my_node.c[array[2].toInt()];

      Serial.println(response);
    }
  }

  else if(array[0] == "a" && count == 3){
    if (array[1].toInt() == my_node.id){
      response = my_node.setAntiwind(array[2].toInt());
      
      Serial.println(response);
    } else {
       send_to_CANBUS = true; 
    }
  }

  else if(array[0] == "k" && count == 3){
    if (array[1].toInt() == my_node.id){
      response = my_node.setFeedback(array[2].toInt());
      
      Serial.println(response);
    } else {
       send_to_CANBUS = true; 
    }
  }


  else if(array[0] == "o" && count == 3){
    if (array[1].toInt() == my_node.id){
      response = my_node.setOccup(array[2].toInt());

      Serial.println(response);
    } else {
       send_to_CANBUS = true; 
    }
  }


  else if(array[0] == "r" && count == 3){
    if (array[1].toInt() == my_node.id){
      my_node.feedback = true;
      response = my_node.setRef(array[2].toInt());

      Serial.println(response);
    } else {
      send_to_CANBUS = true; 
    }
  }

  else if(array[0] == "s" && count == 3){
    if (array[2].toInt() == my_node.id){
      response = "err";

      if (array[1] == "l"){ my_node.stream_l = true;}
      else if(array[1] == "d"){ my_node.stream_d = true;}
      else Serial.println("err");      
    } else {
      send_to_CANBUS = true; 
    }
  }

  else if(array[0] == "S" && count == 3){
    if (array[2].toInt() == my_node.id){
      response = "ack";

      if (array[1] == "l") my_node.stream_l = false;
      else if(array[1] == "d") my_node.stream_d = false; 
      else response = "err";

      Serial.print(response);

    } else {
      send_to_CANBUS = true; 
    }
  }

  else if(array[0] == "r" && count == 1){
    Serial.print("ack");

    command can_command {
      String("r"),
      1,
      27,
      0,
      my_node.id,
      3
    };

    canMsgTx = ch::command2frame(can_command);
    //sends the message
    noInterrupts();
    err = can0.sendMessage(&canMsgTx);
    interrupts();
    
    my_node.resetNode();

    send_to_CANBUS = true;
    
    send_wakeup_order();
  }
  else if(array[0] == "k" && count == 1){
    Serial.println("my_node.k = ");
    for(int i = 0; i < 3; i++) Serial.println(my_node.k[i]);
  }
  else if(array[0] == "w" && count == 2){
    response = String("w ") + array[1].toInt();
    
    Serial.println(response);
  }
  else if(array[0] == "C" && count == 1){
    response = "ack";

    Serial.println(response);
  }
  else{
    Serial.println("Error: command not found");
  }

  if(send_to_CANBUS && fromNode > 2){
    command can_command{ ch::request2command(my_node.id, message_str) };

    canMsgTx = ch::command2frame(can_command);
    
    //sends the message
    noInterrupts();
    err = can0.sendMessage(&canMsgTx);
    interrupts();
  } if(response != "" && fromNode < 3){
    command can_command{ ch::response2command(my_node.id, message_str, fromNode, response) };

    canMsgTx = ch::command2frame(can_command);
    
    //sends the message
    noInterrupts();
    err = can0.sendMessage(&canMsgTx);
    interrupts();
  }

}

float consensus(){
  my_consenso_no.c_id = my_node.id;
  my_consenso_no.c_n = my_node.n;
  my_consenso_no.c_m = my_node.m;
  my_consenso_no.c_o = my_node.o;
  my_consenso_no.c_L = 60;
  for(int jjj =0; jjj< 3; jjj++){
      my_consenso_no.c_d[jjj] = my_node.d[jjj];
      my_consenso_no.c_d_av[jjj] = my_node.d[jjj];
      my_consenso_no.c_y[jjj] = my_node.y_c[jjj];
      my_consenso_no.c_k[jjj] = my_node.k[jjj];
      my_consenso_no.c_c[jjj] = my_node.c[jjj];
  }

  float d1[3]{0,0,0};
  float d2[3]{0,0,0};
  float d3[3]{0,0,0};
  for(int iterations =0; iterations<maxiter; iterations++){
    //minimize augmented lagragian with respect to di for the current values of yi and d_av
    float cost1 = consensus_iterate(my_consenso_no, rho, 3, d1);
    Serial.println("Consenso do no");
    for(int j=0; j<3; j++){
      my_consenso_no.c_d[j] = d1[j];
      Serial.println(d1[j]);
    }
    //exchanges solution 

    for(int track_state = 0; track_state<3;track_state++){
      int msg_sent[3] {0};
      while(msg_sent[0] < 3 && msg_sent[1] < 3 && msg_sent[2] < 3){
        if(millis() >= time_consensus && msg_sent[my_consenso_no.c_id] < 3 && states[track_state] == 1 ) {// && (send_ack1 == false || send_ack2 == false) && msg_sent < 3 ){
            //creates the message
            unsigned long message = my_consenso_no.c_d[msg_sent[my_consenso_no.c_id]];
            
            uint8_t node_pos;
            if(msg_sent[0] == 0) node_pos= 0x0;
            else if(msg_sent[0] == 1) node_pos = 0x1;
            else if (msg_sent[0] == 2) node_pos = 0x2;
            uint8_t response = 0x0;
            uint8_t iteration = (byte)iterations;

//            uint8_t command = 0b11100;
//            int int_b_value = my_consenso_no.c_d[msg_sent]*1000;
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

            int data = my_consenso_no.c_d[msg_sent[my_consenso_no.c_id]]*1000;
            
            command can_command {
              String("C ") + iterations + String(" ") + data,
              0,
              29,
              (data << 8) + iteration,
              my_consenso_no.c_id,
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
            Serial.print(iterations);
            Serial.print(" for position ");
            Serial.print(msg_sent[my_consenso_no.c_id]);
            Serial.print(" of value ");
            Serial.print(data/1000.0);
            Serial.println();
            msg_sent[my_consenso_no.c_id]++;
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
            if(frm_iteration == iterations){//same iteration
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
              

              if (can_command.from_desk_id == (my_consenso_no.c_id + 1)%3) d2[msg_sent[can_command.from_desk_id]] = value;
              if (can_command.from_desk_id == (my_consenso_no.c_id + 2)%3) d3[msg_sent[can_command.from_desk_id]] = value;
              
              msg_sent[can_command.from_desk_id]++;
            }
          }
          data_available = false;
        }
      }
    }
    //compute the average of each Node's solution
    for(int j=0; j<3; j++){
      my_consenso_no.c_d_av[j] = (my_consenso_no.c_d[j]+d2[j]+d3[j])/3;
    }
    //update the Lagrange multipliers
    for(int j=0; j<3; j++){
      my_consenso_no.c_y[j] = my_consenso_no.c_y[j] + rho*(my_consenso_no.c_d[j]-my_consenso_no.c_d_av[j]);
    }
  }
  //solucao dos duty cycles
  Serial.println("Solucao final dos duty cycles: ");
    for(int j=0;j<3;j++){
      Serial.println(my_consenso_no.c_d_av[j]);
    }

  //solucao que indica a referencia que cada no precisa seguir
  float reference = 0;
  //mulitiplicar K*d+o
  reference = my_consenso_no.c_k[0]*my_consenso_no.c_d_av[0] + my_consenso_no.c_k[1]*my_consenso_no.c_d_av[1] + my_consenso_no.c_k[2]*my_consenso_no.c_d_av[2] + my_consenso_no.c_o; 
  Serial.println();
  Serial.print("Referencia final do no:  ");
  Serial.println(reference);
  return reference;  
}
