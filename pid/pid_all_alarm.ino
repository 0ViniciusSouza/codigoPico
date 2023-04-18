#include "pid.h"

//define which assembly is being used (1,2,3)
const int assembly{1};

//Create a pid controller
pid my_pid {assembly}; //select which assembly

const int ldrPin = 28;    // select the input pin for the LDR
const int ledPin = 2;      // select the pin for the LED
float b; //value is defined in the setup
const float m{-0.8}; //from datasheet
const float R{10}; //resistance in kOhms
float G; //gain of LDR calculated in the setup

//median filter size
const int filter_size{7};

float r {0.0}; //reference
bool r_change{false}; //checks whether there is a reference change
float y {0.0}; //current read of sensor

//variables to read the commands passed by serial monitor
String strs[4]; 
String inputString = "";         // a String to hold incoming data
int StringCount {0};

//last minute buffer and performance computation
int write_idx {0}; //current idx to save
const int size_array = 6000;
int duty_cycle[size_array]; //array to store the values of duty cycle in the last minute
float luminance[size_array]; 
bool complete_minute = false;
int total_samples {0}; //for all the time
double energy{0};
double flicker{0};
double visibility{0};
int pwm{0};
int pwm_prev{0};
int pwm_prev2{0};
int samples_flicker{0}; //only add 6,000 samples after reference changed
bool r_never_changed{true};
int samples_change{0};

//variables from simple commands
bool print_all = false;
bool occupancy = false;
bool print_l = false;
bool print_d = false;
bool control_on = true; 
bool antiwindup_on = true;
int force_duty_cycle{0}; //variable to force the duty cycle set by user

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
unsigned long start_time;


//functions
float read_value_y();
int compute_median(int array[], int filter_size);
void split_array_string(String input, String array[]);
void basic_commands(String array[], int count);



void setup() {
  pinMode(ledPin, OUTPUT_12MA); //makes the output limited to 12mA
  Serial.begin( 115200 ); //initialze Serial
  add_repeating_timer_ms( -10, my_repeating_timer_callback, NULL, &timer); //100 Hz
  analogReadResolution(12); //default is 10
  analogWriteFreq(30000); //30KHz
  analogWriteRange(4095); //Max PWM

  //calibrate LDR
  if(assembly == 1) b = 2.9306; //com14
  else if (assembly == 2) b = 3.1694; //com12
  else if (assembly == 3) b = 3.2067; //com10

  //get gain for LDR to then calculate the external illumination: total illumination = G*duty_cycle + external_illuminance
  Serial.println("Getting the gain of LDR...");
  analogWrite(ledPin, 0);
  Serial.println("LED turned off");
  delay(6000); //delay to make sure the system is steady
  float y0 = read_value_y(); //external_illuminance
  analogWrite(ledPin, 4905);
  Serial.println("LED turned on");
  delay(6000); //delay to make sure the system is steady
  float ymax = read_value_y(); //external_illuminance + G*duty_cycle
  analogWrite(ledPin, 0); //turn off LED
  Serial.println("LED turned off");
  G = (ymax - y0)/4095;
  Serial.println("Got the gain");
  delay(6000);
  Serial.println(G);
  Serial.println("Start");
  start_time = micros();
}

void loop() {
  if(timer_fired){
    noInterrupts();
    my_pid.compute_coefficients(r, r_change);

    if (Serial.available() > 0) {
      // read the incoming string
      inputString = Serial.readString();
      split_array_string(inputString, strs);
      basic_commands(strs, StringCount);
      // clear the string:
      inputString = "";   
    }
   
    y = read_value_y();

    float u = my_pid.compute_control(r, y);
    if(control_on)  pwm = (int) u;
    else pwm = force_duty_cycle; 
    
    analogWrite(ledPin, pwm);

    energy+= pwm; //to avoid going to zero, only muliply by PLed, (tk-tk-1) and compute the duty cycle later
    if((r-y)>0) visibility += (r-y); //only adds if it is greater than 0

       
    if((total_samples - samples_change > 6000) || r_never_changed){
      samples_flicker++;
      if(((pwm-pwm_prev)*(pwm_prev-pwm_prev2)>2) && total_samples > 2){
        flicker += (abs(pwm-pwm_prev)+abs(pwm_prev-pwm_prev2));  
      }
    }
    
    pwm_prev2 = pwm_prev;
    pwm_prev = pwm;

        
    duty_cycle[write_idx] = pwm/4095; //values between 0 and 1
    luminance[write_idx] = y;
    
    if (print_d){
      Serial.print("s d "); Serial.print(assembly); Serial.print(" ");Serial.print(pwm); Serial.print(" "); Serial.println((timer_time - start_time)/1000);
    }
    if (print_l){
      Serial.print("s l "); Serial.print(assembly); Serial.print(" ");Serial.print(y); Serial.print(" "); Serial.println((timer_time - start_time)/1000);    }
    if(print_all){
      Serial.print((timer_time- start_time)/1000); Serial.print(", "); Serial.print(r);Serial.print(", "); Serial.print(y);Serial.print(", "); Serial.print(pwm); Serial.println(";"); 
    }


    my_pid.housekeep(r, y, antiwindup_on);

    if(write_idx < size_array-1) write_idx++;
    else{
      complete_minute = true;
      write_idx = 0;
    }

    total_samples ++;
    r_change = false;

    interrupts();
    timer_fired = false;
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
  double Vadc = ldrValue*3.3/4096; //voltage in the LDR
  double Rldr = 3.3*R/(Vadc) - R; //resistance LDR kohms
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
        control_on = false;
        force_duty_cycle = array[2].toInt();
        Serial.println("ack");
      }

      else if(array[0] == "g" && count == 3 && array[2].toInt() == assembly){
        if(array[1] == "e"){
          float P_led {0.108}; // in Watts
          Serial.print("e ");Serial.print(assembly); Serial.print(" ");Serial.println(energy*P_led/4095.0*0.01);
        }
        else if(array[1] == "v" && total_samples!=0){
          Serial.print("v ");Serial.print(assembly); Serial.print(" ");Serial.println(visibility/total_samples);
        }
        else if(array[1] == "f" && total_samples!=0){
          Serial.print("f ");Serial.print(assembly); Serial.print(" ");Serial.println(flicker/samples_flicker);
        }
        else if(array[1] == "d"){
            Serial.print("d "); Serial.print(assembly); Serial.print(" "); Serial.println(duty_cycle[write_idx]);}
        else if(array[1] == "r"){
             Serial.print("r "); Serial.print(assembly); Serial.print(" "); Serial.println(r);} 
        else if(array[1] == "l"){
             Serial.print("l "); Serial.print(assembly); Serial.print(" "); Serial.println(y);}  
        else if(array[1] == "k"){
          if (control_on){ Serial.print("k "); Serial.print("<i>"); Serial.println("1");}
          else{ Serial.print("k "); Serial.print(assembly); Serial.print(" "); Serial.println("0");}
        }
        else if(array[1] == "p"){
          float P_led {0.108}; // in Watts
          Serial.print("p "); Serial.print(assembly); Serial.print(" "); Serial.println(P_led*pwm);
        }
        else if(array[1] == "o"){
          if(occupancy) {Serial.print("o "); Serial.print(assembly); Serial.print(" "); Serial.println("1");}
          else {Serial.print("o "); Serial.print(assembly); Serial.print(" "); Serial.println("0");}
        }
        else if(array[1] == "t"){
          Serial.print("t "); Serial.print(assembly); Serial.print(" "); Serial.println((timer_time - start_time)/1000000);
        }
        else if(array[1] == "x"){
          Serial.print("x "); Serial.print(assembly); Serial.print(" "); Serial.println(y - G*pwm);
        }
        else if(array[1] == "a"){
          if(antiwindup_on) {Serial.print("a "); Serial.print(assembly); Serial.print(" "); Serial.println("1");}
          else {Serial.print("a "); Serial.print(assembly); Serial.print(" "); Serial.println("0");}
        }
      }

      else if(array[0] == "g" && array[3].toInt() == assembly && count == 4){
        if(array[1] == "b"){
          if(array[2] == "l"){
            Serial.print("b l "); Serial.println(assembly);
            int read_idx; //idx to read
            if (complete_minute == false || write_idx == 5999 ) read_idx = 0;
            else read_idx = write_idx + 1;
            int complete_buffer;
            if(complete_minute) complete_buffer = 6000;
            else complete_buffer = write_idx;
            for( ; read_idx < complete_buffer; read_idx ++){
              Serial.print(luminance[read_idx]); Serial.print(", ");
            }
            if(complete_minute && write_idx != 0){
              for(read_idx = 0; read_idx < write_idx; read_idx++){
                  Serial.print(luminance[read_idx]); Serial.print(", ");
              }
            }
          }
          else if(array[2] == "d"){
            Serial.print("b d "); Serial.println(assembly);
            int read_idx; //idx to read
            if (complete_minute == false || write_idx == 5999 ) read_idx = 0;
            else read_idx = write_idx + 1;
            int complete_buffer;
            if(complete_minute) complete_buffer = 6000;
            else complete_buffer = write_idx;
            for( ; read_idx < complete_buffer; read_idx ++){
              Serial.print(duty_cycle[read_idx]); Serial.print(", ");
            }
            if(complete_minute && write_idx != 0){
              for(read_idx = 0; read_idx < write_idx; read_idx++){
                  Serial.print(duty_cycle[read_idx]); Serial.print(", ");
              }
            }
          }
        Serial.println();
        }
      }

      else if(array[0] == "k" && array[1].toInt() == assembly && count == 3){
        if (array[2] == "0\n") control_on = false;
        else if(array[2] == "1\n") control_on = true;
      }

      else if(array[0] == "o" && array[1].toInt() == assembly && count == 3){
        if(array[2] == "0\n"){ occupancy = false; Serial.println("ack");}
        else if(array[2] == "1\n") {occupancy = true; Serial.println("ack");}
        else Serial.println("err");
      }

      else if(array[0] == "r" && array[1].toInt() == assembly && count == 3){
        r_change = true;
        r_never_changed = false;
        samples_change = total_samples;
        r = array[2].toFloat();  //reference
        Serial.println("ack");
      }

      else if(array[0] == "s" && count == 3 && array[2].toInt() == assembly){
        if (array[1] == "l"){ print_l = true;}
        else if(array[1] == "d"){print_d = true;}
        else if(array[1] == "all"){print_all = true; Serial.print( "x = [");}
        else Serial.println("err");
      }

      else if(array[0] == "S" && count == 3 && array[2].toInt() == assembly){
        if (array[1] == "l"){ print_l = false; Serial.println("ack");}
        else if(array[1] == "d"){print_d = false; Serial.println("ack");}
        else if(array[1] == "all"){print_all = false; Serial.println("ack");}
        
        else Serial.println("err");
      }

      else if(array[0] == "a" && array[1].toInt() == assembly && count == 3){
        if(array[2] == "0\n"){antiwindup_on = false; Serial.println("ack");}
        else if(array[2] == "1\n"){antiwindup_on = true; Serial.println("ack");}
        else Serial.println("err"); 
      }

      else{
        Serial.println("Error: command not found");
      }

}
