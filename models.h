#include "pid.h"
#include "MedianFilter.h"


struct Desk
{
    /* data */
    LED my_led;
    LDR my_ldr;
    pid my_pid;
    MedianFilter F;

    bool occupied = 0;
    bool antiWind = 1;
    bool feedback = 1;

    float external_LUX = 0;
    float power = 0;
    float last_resTime = 0;

    float lower_bound = 0;
    float lower_bound_occup = 0;
    float lower_bound_unocc = 0;

    Desk(LED led_, LDR ldr_, pid pid_, MedianFilter F_):
        my_led{led_}, my_ldr{ldr_}, my_pid{pid_}, F{F_}
        {};

    void control(){
        // perform your operations here
        LDRval = analogRead(my_ldr.readPWM());
        x = F.updateFilter(LDRval);
        y = LDR::LUXmeter(x);

        u = my_pid.compute_control(r, y);

        analogWrite(my_led.pin, (int) u);
        my_pid.housekeep(r, y);
    };
};

struct LED
{
    /* data */
    int pin;
    int DAC_RANGE;

    int dutyCycle = 0;
    int pwm = 0;
    int r = 0;

    LED(int pin_): pin{pin_}, DAC_RANGE{4096} {};
    LED(int pin_, int DAC_RANGE_): 
        pin{pin_}, DAC_RANGE{DAC_RANGE_} {};

    String write(int dutyCycle_){
        if (dutyCycle_ > 1 || dutyCycle_ < 0){
            return "err";
        } else {
            dutyCycle = dutyCycle_;
            pwm = (int) (dutyCycle * DAC_RANGE);
            analogWrite(pin, pwm);
            return "ack";
        }
    };

    String setRef(int r_){
        if (r_ < 0 || r_ > 100){
            return "err";
        } else {
            r = r_;
            return "ack";
        }
    };

};

struct LDR
{
    /* data */
    int ldrPin;
    int DAC_RANGE;
    float Vcc;

    int LDRpwm = 0;

    LDR(int ldrPin_): 
        ldrPin{ldrPin_}, DAC_RANGE{4096}, Vcc{3.3} {};

    LDR(int ldrPin_, int DAC_RANGE_): 
        ldrPin{ldrPin_}, DAC_RANGE{DAC_RANGE_}, Vcc{3.3} {};

    float ADC2Volts(int ADC_read){
        return Vcc * float(ADC_read)/float(LDR::DAC_RANGE);
    };

    float Volt2Ohms(float LDRVoltage){
        return (Vcc - LDRVoltage) * 10000/LDRVoltage;
    };

    double Omhs2Lux(double LDROmhs){
        double b = -0.8*log10(10) - log10(150000);
        double m = -0.8;
  
        return pow(10, (log10(LDROmhs) + b) / m); 
    };

    double LUXmeter(int val){
        return Omhs2Lux(Volt2Ohms(ADC2Volts(val)));
    };
    
    int readPWM(){
        LDRpwm = analogRead(ldrPin);
        return LDRpwm;
    };
    
    int readLUX(){
        LDRpwm = analogRead(ldrPin);
        return LUXmeter(LDRpwm);
    };    
};

