#include <Braccio.h>

#include <Servo.h>

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_roll;
Servo wrist_pitch;
Servo gripper;

void setup() {  
    Serial.begin(2400);
    Braccio.begin();
}

void loop() {
    static int rec_base, rec_shoulder, rec_elbow, rec_wrist_roll, rec_wrist_pitch, rec_gripper;
    static String Data = "";
    //90b90s0e0r0p73g
    while (Serial.available())
    {
        char character = Serial.read(); 
        Data.concat(character); 

        if (character == 'b')
        {
            rec_base = Data.toInt();
            Data = "";
        }

        else if (character == 's')
        {
            rec_shoulder = Data.toInt();
            Data = "";

        }

        else if (character == 'e')
        {
            rec_elbow = Data.toInt();
            Data = "";
        }

        else if (character == 'r')
        {
            rec_wrist_roll = Data.toInt();
            Data = "";

        }

        else if (character == 'p')
        {
            rec_wrist_pitch = Data.toInt();
            Data = "";
        }

        else if (character == 'g')
        {
            rec_gripper = Data.toInt();
            Data = "";
        }

    }
  Braccio.ServoMovement(20, rec_base, rec_shoulder, rec_elbow, rec_wrist_roll, rec_wrist_pitch, rec_gripper);  
}