#include <Braccio.h>
#include <Servo.h>

#include <Wire.h>
#include <VL53L0X.h>

#define UL_PIN A3
#define LL_PIN A2

VL53L0X tof_sensor;

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_roll;
Servo wrist_pitch;
Servo gripper;


int rec_base, rec_shoulder, rec_elbow, rec_wrist_roll, rec_wrist_pitch, rec_gripper;

void setup() 
{    
    Serial.begin(2000);
    Braccio.begin();

    Wire.begin();

    tof_sensor.init();
    tof_sensor.setTimeout(500);

    // tof_sensor.setSignalRateLimit(0.1);
    // tof_sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    // tof_sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    
    tof_sensor.setMeasurementTimingBudget(200000);

    // tof_sensor.startContinuous();

    init_arm();
}

void loop() 
{
    static String serial_string = "";
    static unsigned long prev_height_time = 0;
    static unsigned long prev_ul_time = 0;
    static unsigned long prev_ll_time = 0;
    

    if((millis() -  prev_height_time) >= 200)
    {   
        Serial.print(get_arm_height());
        Serial.print('h');
        prev_height_time = millis();
    }

     if((millis() -  prev_ul_time) >= 200)
    {   
        Serial.print(get_upper_limit());
        Serial.print('u');
        prev_ul_time = millis();
    }

     if((millis() -  prev_ll_time) >= 200)
    {   
        Serial.print(get_lower_limit());
        Serial.print('l');
        prev_ll_time = millis();
    }

    while (Serial.available())
    {
        char character = Serial.read(); 
        serial_string.concat(character); 

        if (character == 'b')
        {
            rec_base = serial_string.toInt();
            serial_string = "";
        }

        else if (character == 's')
        {
            rec_shoulder = serial_string.toInt();
            serial_string = "";

        }

        else if (character == 'e')
        {
            rec_elbow = serial_string.toInt();
            serial_string = "";
        }

        else if (character == 'r')
        {
            rec_wrist_roll = serial_string.toInt();
            serial_string = "";

        }

        else if (character == 'p')
        {
            rec_wrist_pitch = serial_string.toInt();
            serial_string = "";
        }

        else if (character == 'g')
        {
            rec_gripper = serial_string.toInt();
            serial_string = "";
        }

    }

    Braccio.ServoMovement(20, rec_base, rec_shoulder, rec_elbow, rec_wrist_roll, rec_wrist_pitch, rec_gripper);  

}

void init_arm()
{
    rec_base = 90;
    rec_shoulder = 90; 
    rec_elbow = 0;
    rec_wrist_roll = 0; 
    rec_wrist_pitch = 0;
    rec_gripper = 0;
}

double get_arm_height()
{
    // return tof_sensor.readRangeContinuousMillimeters();
    double height;
    int measured_val;

    measured_val = tof_sensor.readRangeSingleMillimeters();
    height = (0.0003 * pow((double)measured_val, 2)) + ((0.4082 * (double) measured_val) + 125.9);
    return height;
}

int get_upper_limit()
{
    return is_triggered(analogRead(UL_PIN));
}

int get_lower_limit()
{
    return is_triggered(analogRead(LL_PIN));
}

float map_float(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

int is_triggered(int val)
{
    if(val > 1000)
        return 1;
    else
        return 0;
}

