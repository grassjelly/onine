#include <Braccio.h>
#include <Servo.h>

#define UL_PIN A3
#define LL_PIN A2

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_roll;
Servo wrist_pitch;
Servo gripper;

int rec_base, rec_shoulder, rec_elbow, rec_wrist_roll, rec_wrist_pitch, rec_gripper;

void setup() 
{    
    Serial.begin(4800);
    Braccio.begin();

    init_arm();
}

void loop() 
{
    static String serial_string = "";
    static unsigned long prev_ls_time = 0;
    
     if((millis() -  prev_ls_time) >= 200)
    {   
        Serial.print(check_limit_switches());
        Serial.print('l');
        prev_ls_time = millis();
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

    Braccio.ServoMovement(20, rec_base, rec_shoulder + 10, rec_elbow, rec_wrist_pitch, rec_wrist_roll, rec_gripper);  

}

void init_arm()
{
    rec_base = 5;
    rec_shoulder = 90; 
    rec_elbow = 0;
    rec_wrist_pitch = 5;
    rec_wrist_roll = 0; 
    rec_gripper = 0;
}

int check_limit_switches()
{
    if(is_triggered(analogRead(LL_PIN)))
        return -1;

    else if(is_triggered(analogRead(UL_PIN)))
        return 1;

    else    
        return 0;
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

