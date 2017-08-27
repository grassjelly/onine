/*
 Braccio.cpp - board library Version 2.0
 Written by Andrea Martino and Angelo Ferrante
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "Braccio.h"

extern Servo base;
extern Servo shoulder;
extern Servo elbow;
extern Servo wrist_roll;
extern Servo wrist_pitch;
extern Servo gripper;

extern int step_base = 0;
extern int step_shoulder = 45;
extern int step_elbow = 180;
extern int step_wrist_roll = 180;
extern int step_wrist_pitch = 90;
extern int step_gripper = 10;
 

_Braccio Braccio;

//Initialize Braccio object
_Braccio::_Braccio() {
}

/**
 * Braccio initialization and set intial position
 * Modifing this function you can set up the initial position of all the
 * servo motors of Braccio
 * @param soft_start_level: default value is 0 (SOFT_START_DEFAULT)
 * You should set begin(SOFT_START_DISABLED) if you are using the Arm Robot shield V1.6
 * SOFT_START_DISABLED disable the Braccio movements
 */
unsigned int _Braccio::begin(int soft_start_level) {
	//Calling Braccio.begin(SOFT_START_DISABLED) the Softstart is disabled and you can use the pin 12
	if(soft_start_level!=SOFT_START_DISABLED){
		pinMode(SOFT_START_CONTROL_PIN,OUTPUT);
		digitalWrite(SOFT_START_CONTROL_PIN,LOW);
	}

	// initialization pin Servo motors
	base.attach(11);
	shoulder.attach(10);
	elbow.attach(9);
	wrist_roll.attach(6);
	wrist_pitch.attach(5);
	gripper.attach(3);
        
	//For each step motor this set up the initial degree
	base.write(0);
	shoulder.write(90);
	elbow.write(0);
	wrist_pitch.write(0);
	wrist_roll.write(0);
	gripper.write(0);
	//Previous step motor position
	step_base = 0;
	step_shoulder = 90;
	step_elbow = 0;
	step_wrist_pitch = 0;
	step_wrist_roll = 0;
	step_gripper = 0;

	if(soft_start_level!=SOFT_START_DISABLED)
    		_softStart(soft_start_level);
	return 1;
}

/*
Software implementation of the PWM for the SOFT_START_CONTROL_PIN,HIGH
@param high_time: the time in the logic level high
@param low_time: the time in the logic level low
*/
void _Braccio::_softwarePWM(int high_time, int low_time){
	digitalWrite(SOFT_START_CONTROL_PIN,HIGH);
	delayMicroseconds(high_time);
	digitalWrite(SOFT_START_CONTROL_PIN,LOW);
	delayMicroseconds(low_time); 
}

/*
* This function, used only with the Braccio Shield V4 and greater,
* turn ON the Braccio softly and save it from brokes.
* The SOFT_START_CONTROL_PIN is used as a software PWM
* @param soft_start_level: the minimum value is -70, default value is 0 (SOFT_START_DEFAULT)
*/
void _Braccio::_softStart(int soft_start_level){      
	long int tmp=millis();
	while(millis()-tmp < LOW_LIMIT_TIMEOUT)
		_softwarePWM(80+soft_start_level, 450 - soft_start_level);   //the sum should be 530usec	

	while(millis()-tmp < HIGH_LIMIT_TIMEOUT)
		_softwarePWM(75 + soft_start_level, 430 - soft_start_level); //the sum should be 505usec

	digitalWrite(SOFT_START_CONTROL_PIN,HIGH);
}

/**
 * This functions allow you to control all the servo motors
 * 
 * @param stepDelay The delay between each servo movement
 * @param vBase next base servo motor degree
 * @param vShoulder next shoulder servo motor degree
 * @param vElbow next elbow servo motor degree
 * @param vwrist_pitch next wrist rotation servo motor degree
 * @param vwrist_roll next wrist vertical servo motor degree
 * @param vgripper next gripper servo motor degree
 */
int _Braccio::ServoMovement(int stepDelay, int vBase, int vShoulder, int vElbow,int vwrist_pitch, int vwrist_roll, int vgripper) {

	// Check values, to avoid dangerous positions for the Braccio
    	if (stepDelay > 30) stepDelay = 30;
	if (stepDelay < 10) stepDelay = 10;
	if (vBase < 0) vBase=0;
	if (vBase > 180) vBase=180;
	if (vShoulder < 15) vShoulder=15;
	if (vShoulder > 165) vShoulder=165;
	if (vElbow < 0) vElbow=0;
	if (vElbow > 180) vElbow=180;
	if (vwrist_pitch < 0) vwrist_pitch=0;
	if (vwrist_pitch > 180) vwrist_pitch=180;
	if (vwrist_roll > 180) vwrist_roll=180;
	if (vwrist_roll < 0) vwrist_roll=0;
    	if (vgripper < 10) vgripper = 10;
	if (vgripper > 73) vgripper = 73;

	int exit = 1;

	//Until the all motors are in the desired position
	while (exit) 
	{			
		//For each servo motor if next degree is not the same of the previuos than do the movement		
		if (vBase != step_base) 
		{			
			base.write(step_base);
			//One step ahead
			if (vBase > step_base) {
				step_base++;
			}
			//One step beyond
			if (vBase < step_base) {
				step_base--;
			}
		}

		if (vShoulder != step_shoulder)  
		{
			shoulder.write(step_shoulder);
			//One step ahead
			if (vShoulder > step_shoulder) {
				step_shoulder++;
			}
			//One step beyond
			if (vShoulder < step_shoulder) {
				step_shoulder--;
			}

		}

		if (vElbow != step_elbow)  
		{
			elbow.write(step_elbow);
			//One step ahead
			if (vElbow > step_elbow) {
				step_elbow++;
			}
			//One step beyond
			if (vElbow < step_elbow) {
				step_elbow--;
			}

		}

		if (vwrist_pitch != step_wrist_roll) 
		{
			wrist_roll.write(step_wrist_roll);
			//One step ahead
			if (vwrist_pitch > step_wrist_roll) {
				step_wrist_roll++;				
			}
			//One step beyond
			if (vwrist_pitch < step_wrist_roll) {
				step_wrist_roll--;
			}

		}

		if (vwrist_roll != step_wrist_pitch)
		{
			wrist_pitch.write(step_wrist_pitch);
			//One step ahead
			if (vwrist_roll > step_wrist_pitch) {
				step_wrist_pitch++;
			}
			//One step beyond
			if (vwrist_roll < step_wrist_pitch) {
				step_wrist_pitch--;
			}
		}

		if (vgripper != step_gripper)
		{
			gripper.write(step_gripper);
			if (vgripper > step_gripper) {
				step_gripper++;
			}
			//One step beyond
			if (vgripper < step_gripper) {
				step_gripper--;
			}
		}
		
		//delay between each movement
		delay(stepDelay);
		
		//It checks if all the servo motors are in the desired position 
		if ((vBase == step_base) && (vShoulder == step_shoulder)
				&& (vElbow == step_elbow) && (vwrist_pitch == step_wrist_roll)
				&& (vwrist_roll == step_wrist_pitch) && (vgripper == step_gripper)) {
			step_base = vBase;
			step_shoulder = vShoulder;
			step_elbow = vElbow;
			step_wrist_roll = vwrist_pitch;
			step_wrist_pitch = vwrist_roll;
			step_gripper = vgripper;
			exit = 0;
		} else {
			exit = 1;
		}
	}
}