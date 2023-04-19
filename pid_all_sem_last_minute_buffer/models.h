#include "pid.h"
#include "MedianFilter.h"
#include <iostream>
#include <bits/stdc++.h>
#include <algorithm>

struct LED
{
    /* data */
    int pin;
    int DAC_RANGE;
    int dutyCycle;

    float Vcc = 3.3;

    LED(int pin_): pin{pin_}, DAC_RANGE{4096} {
        analogWrite(pin, 0);
        dutyCycle = 0;
    };
    LED(int pin_, int DAC_RANGE_): 
        pin{pin_}, DAC_RANGE{DAC_RANGE_} {
        analogWrite(pin, 0);
        dutyCycle = 0;
    };

    String write(int dutyCycle_){
        if (dutyCycle_ < 0 || dutyCycle_ > DAC_RANGE){
            return "err";
        } else {
            analogWrite(pin, dutyCycle_);
            dutyCycle = dutyCycle_;
            return "ack";
        }
    };
};

struct LDR
{
    /* data */
    int ldrPin;
    int DAC_RANGE;
    float Vcc = 3.3;
    double b;

    LDR(int ldrPin_): 
        ldrPin{ldrPin_}, DAC_RANGE{4096} {
            b = 2.9;
        };

    LDR(int ldrPin_, int DAC_RANGE_): 
        ldrPin{ldrPin_}, DAC_RANGE{DAC_RANGE_} {
            b = 2.9;
        };

    // float ADC2Volts(int ADC_read){
    //     return Vcc * float(ADC_read)/float(LDR::DAC_RANGE);
    // };

    // float Volt2Ohms(float LDRVoltage){
    //     return (Vcc - LDRVoltage) * 10000/LDRVoltage;
    // };

    // double Omhs2Lux(double LDROmhs){
    //     double m = -0.8;
  
    //     return pow(10, (log10(LDROmhs) - b) / m); 
    // };

    // double LUXmeter(int val){
    //     return Omhs2Lux(Volt2Ohms(ADC2Volts(val)));
    // };

    int readPWM(){
        int LDRpwm = analogRead(ldrPin);
        return LDRpwm;
    };
    
    // float readLUX(){
    //     int LDRpwm = analogRead(ldrPin);
    //     return LUXmeter(LDRpwm);
    // };
    double read_value_y(int LDR_read){
        int R = 10; 
        double m = -0.8;
        double Vadc = LDR_read*3.3/4096; //voltage in the LDR
        double Rldr = 3.3*R/(Vadc) - R; //resistance LDR kohms
        return pow(10,(log10(Rldr)-b)/m); //convert from resistance to LUX (y)
    }
    
};



struct Node {
    /* data */
    int id = 1;

    float d[3];
    float d_av[3];
    float y_c[3];
    float k[3];
    float c[3];
    float n;
    float m;
    float o;
    float L;

    float L_occup;
    float L_unoccup;

    LED my_led;
    LDR my_ldr;
    pid my_pid;
    MedianFilter F;

    float u;
    float x;
    float y;
    float r;
    float r_old;

    int LDRval;

    bool consensus;
    bool sleeping;

    bool occupied;
    bool feedback;
    bool antiwind;
    bool stream_l;
    bool stream_d;
    
   // float dutyCycle;
    float last_resTime;

    float lastUpdateTime;
    unsigned long int updateCounter;
    float energy;
    float visError;
    float flickerError;
    float duty[3];

    float last_min_d[6000];
    float last_min_l[6000];
    float last_min_e[6000];
    float last_min_v[6000];
    float last_min_f[6000];


    Node(LED led_, LDR ldr_, pid pid_, MedianFilter F_):
        my_led{led_}, my_ldr{ldr_}, my_pid{pid_}, F{F_}
        {
        resetNode();
        };
    
    void resetNode() {
        n = 0;
        m = 0;
        o = 0;

        consensus = 0;
        sleeping = 1;
        occupied = 1;
        feedback = 1;
        antiwind = 1;
        stream_l = 0;
        stream_d = 0;

        my_led.dutyCycle = 0;
        last_resTime = 0;
        lastUpdateTime = 0;
        updateCounter = 0;
        energy = 0;
        visError = 0;
        flickerError = 0;
//        duty = { 0 };
//        last_min_d = { 0 };
//        last_min_l = { 0 };
//        last_min_e = { 0 };
//        last_min_v = { 0 };
//        last_min_f = { 0 };
    };

    void resetFunction() {
        last_resTime = millis();
        resetNode();
        
        analogWrite(my_led.pin, 0);
        my_pid.resetPID();

        resetCommand();
    };

    void control(){
        int x =  my_ldr.readPWM();
        
         
        y = my_ldr.read_value_y(x);

      //  Serial.print("y = "); Serial.println(y);

        my_pid.compute_coefficients(r, r_old != r);

        r_old = r;

        u = my_pid.compute_control(r, y);

        if (feedback){   
          analogWrite(my_led.pin, (int) u);
          my_led.dutyCycle = (int) u;
        }
    
        my_pid.housekeep(r, y, antiwind);
   
        updateData(my_led.dutyCycle, y);
        updatePerformances();

        if(stream_l){
            float time = millis() - last_resTime;

            String str1 = "s l ";
            String space = " ";
            String msg = str1 + id;
            msg += space + y;
            msg += space + time;

            Serial.println(msg);
        }
        if(stream_d){
            float time = millis() - last_resTime;

            String str1 = "s d ";
            String space = " ";

            String msg = str1 + id;
            msg += space + my_led.dutyCycle;
            msg += space + time;

            Serial.println(msg);
        }
    };

    String setRef(float r_){
        if (r_ < 0){
            return "err";
        } else {
            r = r_;
            return "ack";
        }
    };

    String setOccup(int o_){
        if (o_ != 0 || o_ != 1){
            return "err";
        } else {
            occupied = o_;
            return "ack";
        }
    };

    String setAntiwind(int a_){
        if (a_ != 0 || a_ != 1){
            return "err";
        } else {
            antiwind = a_;
            return "ack";
        }
    };

    String setFeedback(int k_){
        if (k_ != 0 || k_ != 1){
            return "err";
        } else {
            feedback = k_;
            return "ack";
        }
    };

    float power(){
     //nao entendi a conta
     //   return (my_led.Vcc*dutyCycle/100)*12*0.001;
     return 108*0.001*my_led.dutyCycle/4096;
    };

    void updatePerformances(){
        energy += power() * (millis() - lastUpdateTime) * 0.001;
        
        visError = (visError*updateCounter + max(0,(r - y)))/(updateCounter+1);
        
        float flicker = 0;
        if ((duty[2] - duty[1])*(duty[1] - duty[0]) > 0){
            flicker = abs(duty[2] - duty[1]) + abs(duty[1] - duty[0]);
        }

        flickerError = (flickerError*updateCounter + flicker)/(updateCounter + 1);

        std::memmove(last_min_v,last_min_v+1,sizeof(float)*(6000-1));
        last_min_d[6000 - 1] = visError;

        std::memmove(last_min_f,last_min_f+1,sizeof(float)*(6000-1));
        last_min_d[6000 - 1] = flickerError;

        std::memmove(last_min_e,last_min_e+1,sizeof(float)*(6000-1));
        last_min_l[6000 - 1] = energy;
        
        updateCounter++;
        lastUpdateTime = millis();
    };

    void (* resetCommand) (void) = 0;

    void updateData(float dutyCycle, float luminance){
        std::memmove(duty,duty+1,sizeof(float)*(3-1));
        duty[3 - 1] = dutyCycle;

        std::memmove(last_min_d,last_min_d+1,sizeof(float)*(6000-1));
        last_min_d[6000 - 1] = dutyCycle;

        std::memmove(last_min_l,last_min_l+1,sizeof(float)*(6000-1));
        last_min_l[6000 - 1] = luminance;
    };
};
