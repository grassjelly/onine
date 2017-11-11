#include <Braccio.h>

#include <Servo.h>

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_roll;
Servo wrist_pitch;
Servo gripper;

int rec_base, rec_shoulder, rec_elbow, rec_wrist_roll, rec_wrist_pitch, rec_gripper;

void setup() 
{  
    init_arm();
    Serial.begin(2400);
    Braccio.begin();
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
        Serial.flush();
        prev_height_time = millis();
    }

     if((millis() -  prev_ul_time) >= 200)
    {   
        Serial.print(get_upper_limit());
        Serial.print('u');
        Serial.flush();
        prev_ul_time = millis();
    }

     if((millis() -  prev_ll_time) >= 200)
    {   
        Serial.print(get_lower_limit());
        Serial.print('l');
        Serial.flush();
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

int get_arm_height()
{
    return 23;
}

int get_upper_limit()
{
    return 1;
}

int get_lower_limit()
{
    return 0;
}