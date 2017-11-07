#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

#define LINEAR_ACTUATOR_SPEED 0.01
#define TORSO_MIN_HEIGHT 0.21

#define MOTOR_IN_A 9
#define MOTOR_IN_B 10

float req_joint_state[7];
float prev_linear_state;

void jointstates_callback( const sensor_msgs::JointState& joint);
void gripper_callback( const std_msgs::Bool& state);

ros::NodeHandle  nh;

ros::Subscriber<sensor_msgs::JointState> joinstates_sub("joint_states", jointstates_callback);
ros::Subscriber<std_msgs::Bool> gripper_sub("braccio_gripper", gripper_callback);

void setup() 
{
    pinMode(MOTOR_IN_A, OUTPUT);
    pinMode(MOTOR_IN_B, OUTPUT);

    init_arm();

    Serial1.begin(2400);
    nh.getHardware()->setBaud(57600);
    nh.initNode();
    nh.subscribe(joinstates_sub);
    nh.subscribe(gripper_sub);

    while (!nh.connected()){
        nh.spinOnce();
    }

    nh.loginfo("BRACCIO CONNECTED!");
}

void loop() 
{ 
    move_arm();
    nh.spinOnce();
}

void move_arm()
{
    if(prev_linear_state > req_joint_state[0]){
        move_z(80);
        nh.loginfo("going down");
    }

    else if(prev_linear_state < req_joint_state[0]){
        move_z(-80);
        nh.loginfo("going up");
    }

    else{
        move_z(0);
    }

    prev_linear_state = req_joint_state[0];
   
    char log_msg[50];    
    char result[8];
    dtostrf(req_joint_state[0], 6, 2, result);
    sprintf(log_msg,"Arm Height = %s", result);
    nh.loginfo(log_msg);

    Serial1.print(req_joint_state[1]);
    Serial1.print('b');

    Serial1.print(req_joint_state[2]);
    Serial1.print('s');

    Serial1.print(req_joint_state[3]);
    Serial1.print('e');

    Serial1.print(req_joint_state[4]);
    Serial1.print('r');

    Serial1.print(req_joint_state[5]);
    Serial1.print('p');

    Serial1.print(map(req_joint_state[6], 70, 90, 0, 70));
    Serial1.print('g');
}

void jointstates_callback( const sensor_msgs::JointState& joint)
{
    for(int i = 0; i < 7; i++){
        if(i == 0)
            req_joint_state[i] = joint.position[i]; 
        else
            req_joint_state[i] = joint.position[i] * 57.2958;
    }
}

void gripper_callback( const std_msgs::Bool& state)
{
    if(state.data)
        req_joint_state[6] = 1.217;
    else
        req_joint_state[6] = 1.5708;
}

void move_z(int speed)
{
    if (speed > 0){
        analogWrite(MOTOR_IN_A, 0);
        analogWrite(MOTOR_IN_B, abs(speed));
    }
    else if (speed < 0){
        analogWrite(MOTOR_IN_B, 0);
        analogWrite(MOTOR_IN_A, abs(speed));
    }
    else{
        analogWrite(MOTOR_IN_B, 0);
        analogWrite(MOTOR_IN_A, 0);
    }
}

void init_arm()
{
    prev_linear_state = TORSO_MIN_HEIGHT;

    for(int i = 0; i < 7; i++){
        if(i == 0)
            req_joint_state[i] = TORSO_MIN_HEIGHT; 
        else if(i == 1 || i == 2)
            req_joint_state[i] = 90; 
        else
            req_joint_state[i] = 0;
    }
}