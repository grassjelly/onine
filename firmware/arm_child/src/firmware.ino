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
    static String Data = "";
    //90b90s0e0r0p73g
    while (Serial.available()){
        char character = Serial.read(); 
        Data.concat(character); 

        if (character == 'b'){
            rec_base = Data.toInt();
            Data = "";
        }

        else if (character == 's'){
            rec_shoulder = Data.toInt();
            Data = "";

        }

        else if (character == 'e'){
            rec_elbow = Data.toInt();
            Data = "";
        }

        else if (character == 'r'){
            rec_wrist_roll = Data.toInt();
            Data = "";

        }

        else if (character == 'p'){
            rec_wrist_pitch = Data.toInt();
            Data = "";
        }

        else if (character == 'g'){
            rec_gripper = Data.toInt();
            Data = "";
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