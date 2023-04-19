#include "models.h"

//define which assembly is being used (1,2,3)
const int assembly{1};

int ledPin = 15; //2
int ldrPin = 26; //28

//Create a pid controller
pid my_pid {assembly}; //select which assembly
LED my_led {ledPin};
LDR my_ldr {ldrPin};
MedianFilter F{10};

Node my_node{ 
    my_led,
    my_ldr,
    my_pid,
    F
};

//variables to read the commands passed by serial monitor
String strs[4]; 
String inputString = "";         // a String to hold incoming data
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
void basic_commands(String array[], int count);



void setup() {
  pinMode(ledPin, OUTPUT_12MA); //makes the output limited to 12mA
  Serial.begin( 9600 ); //initialze Serial
  
  add_repeating_timer_ms( -10, my_repeating_timer_callback, NULL, &timer); //100 Hz
  
  analogReadResolution(12); //default is 10
  analogWriteFreq(30000); //30KHz
  analogWriteRange(4095); //Max PWM

  //calibrate LDR
  if(assembly == 1) my_node.my_ldr.b = 2.9306; //com14
  else if (assembly == 2) my_node.my_ldr.b = 3.1694; //com12
  else if (assembly == 3) my_node.my_ldr.b = 3.2067; //com10
}

void loop() {
  if(timer_fired){
     noInterrupts();
     if (Serial.available() > 0) {
       // read the incoming string
       inputString = Serial.readString();
       split_array_string(inputString, strs);
       basic_commands(strs, StringCount);
       // clear the string:
       inputString = "";   
     }
   
    my_node.control();

    interrupts();
    timer_fired = false;
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

void basic_commands(String array[], int count){
   if(array[0] == "d" && array[1].toInt() == assembly && count == 3){
      my_node.feedback = false;
      Serial.println(my_node.my_led.write(array[2].toInt()));
   }


  else if(array[0] == "g" && count == 3 && array[2].toInt() == assembly){
    if(array[1] == "e"){
      Serial.print("e ");Serial.print(assembly); Serial.print(" ");Serial.println(my_node.energy);
    }
    else if(array[1] == "v"){
      Serial.print("v ");Serial.print(assembly); Serial.print(" ");Serial.println(my_node.visError);
    }
    else if(array[1] == "f"){
      Serial.print("f ");Serial.print(assembly); Serial.print(" ");Serial.println(my_node.flickerError);
    }
    else if(array[1] == "d"){
        Serial.print("d "); Serial.print(assembly); Serial.print(" "); Serial.println(my_node.my_led.dutyCycle);}
    else if(array[1] == "r"){
        Serial.print("r "); Serial.print(assembly); Serial.print(" "); Serial.println(my_node.r);} 
    else if(array[1] == "l"){
        Serial.print("l "); Serial.print(assembly); Serial.print(" "); Serial.println(my_node.y);}  
    else if(array[1] == "k"){
      //if (control_on){ Serial.print("k "); Serial.print(assembly); Serial.println("1");}
      //else{ Serial.print("k "); Serial.print(assembly); Serial.print(" "); Serial.println("0");}
        Serial.print("k "); Serial.print(assembly); Serial.print(" "); Serial.println(my_node.feedback);
    }
    else if(array[1] == "p"){
      float P_led {0.108}; // in Watts
      Serial.print("p "); Serial.print(assembly); Serial.print(" "); Serial.println(P_led*my_node.my_led.dutyCycle);
    }
    else if(array[1] == "o"){
//      if(occupancy) {Serial.print("o "); Serial.print(assembly); Serial.print(" "); Serial.println("1");}
//     else {Serial.print("o "); Serial.print(assembly); Serial.print(" "); Serial.println("0");}
      Serial.print("o "); Serial.print(assembly); Serial.print(" "); Serial.println(my_node.occupied);
    }
    else if(array[1] == "t"){
      Serial.print("t "); Serial.print(assembly); Serial.print(" "); Serial.println(my_node.lastUpdateTime);
    }
    else if(array[1] == "x"){
      //REVER QUANDO COLOCAR WAKEUP
      Serial.print("x "); Serial.print(assembly); Serial.print(" "); Serial.println("rever");
    }
    else if(array[1] == "a"){
     // if(antiwindup_on) {Serial.print("a "); Serial.print(assembly); Serial.print(" "); Serial.println("1");}
     // else {Serial.print("a "); Serial.print(assembly); Serial.print(" "); Serial.println("0");}
      Serial.print("a "); Serial.print(assembly); Serial.print(" "); Serial.println(my_node.antiwind);
    }
  }


//       else if(array[0] == "g" && array[3].toInt() == assembly && count == 4){
//         if(array[1] == "b"){
//           if(array[2] == "l"){
//             Serial.print("b l "); Serial.println(assembly);
//             int read_idx; //idx to read
//             if (complete_minute == false || write_idx == 5999 ) read_idx = 0;
//             else read_idx = write_idx + 1;
//             int complete_buffer;
//             if(complete_minute) complete_buffer = 6000;
//             else complete_buffer = write_idx;
//             for( ; read_idx < complete_buffer; read_idx ++){
//               Serial.print(luminance[read_idx]); Serial.print(", ");
//             }
//             if(complete_minute && write_idx != 0){
//               for(read_idx = 0; read_idx < write_idx; read_idx++){
//                   Serial.print(luminance[read_idx]); Serial.print(", ");
//               }
//             }
//           }
//           else if(array[2] == "d"){
//             Serial.print("b d "); Serial.println(assembly);
//             int read_idx; //idx to read
//             if (complete_minute == false || write_idx == 5999 ) read_idx = 0;
//             else read_idx = write_idx + 1;
//             int complete_buffer;
//             if(complete_minute) complete_buffer = 6000;
//             else complete_buffer = write_idx;
//             for( ; read_idx < complete_buffer; read_idx ++){
//               Serial.print(duty_cycle[read_idx]); Serial.print(", ");
//             }
//             if(complete_minute && write_idx != 0){
//               for(read_idx = 0; read_idx < write_idx; read_idx++){
//                   Serial.print(duty_cycle[read_idx]); Serial.print(", ");
//               }
//             }
//           }
//         Serial.println();
//         }
//       }

  else if(array[0] == "a" && array[1].toInt() == assembly && count == 3){
    Serial.println(my_node.setAntiwind(array[2].toInt()));
  }

  else if(array[0] == "k" && array[1].toInt() == assembly && count == 3){
    Serial.println(my_node.setFeedback(array[2].toInt()));
  }


  else if(array[0] == "o" && array[1].toInt() == assembly && count == 3){
    Serial.println(my_node.setOccup(array[2].toInt()));
  }


  else if(array[0] == "r" && array[1].toInt() == assembly && count == 3){
    my_node.feedback = true;
    Serial.println(my_node.setRef(array[2].toFloat()));
  }

  else if(array[0] == "s" && count == 3 && array[2].toInt() == assembly){
    if (array[1] == "l"){ my_node.stream_l = true;}
    else if(array[1] == "d"){my_node.stream_d = true;}
    else Serial.println("err");
  }

  else if(array[0] == "S" && count == 3 && array[2].toInt() == assembly){
    if (array[1] == "l"){ my_node.stream_l = false; Serial.println("ack");}
    else if(array[1] == "d"){my_node.stream_d = false; Serial.println("ack");}    
    else Serial.println("err");
  }

  else{
    Serial.println("Error: command not found");
  }

}
