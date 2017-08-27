#include <Servo.h>

#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

int req_servo_angle[7];

void jointstates_callback( const sensor_msgs::JointState& joint);
void gripper_callback( const std_msgs::Bool& state);

ros::NodeHandle  nh;

ros::Subscriber<sensor_msgs::JointState> joinstates_sub("joint_states", jointstates_callback);
ros::Subscriber<std_msgs::Bool> gripper_sub("braccio_gripper", gripper_callback);

void setup() 
{
    Serial1.begin(2400);
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(joinstates_sub);
    nh.subscribe(gripper_sub);

    while (!nh.connected())
    {
        nh.spinOnce();
    }

    nh.loginfo("BRACCIO CONNECTED!");
}

void loop() 
{ 
    nh.spinOnce();
    move_arm();
}

void move_arm()
{
    String command = "";

    
    Serial1.print(req_servo_angle[0]);
    Serial1.print('b');

    Serial1.print(req_servo_angle[1]);
    Serial1.print('s');

    Serial1.print(req_servo_angle[2]);
    Serial1.print('e');

    Serial1.print(req_servo_angle[3]);
    Serial1.print('r');

    Serial1.print(req_servo_angle[4]);
    Serial1.print('p');

    Serial1.print(map(req_servo_angle[5], 70, 90, 0, 70));
    Serial1.print('g');

    // Serial1.print(command);
    // Braccio.ServoMovement(20, 
    //         req_servo_angle[0], 
    //         req_servo_angle[1], 
    //         req_servo_angle[2], 
    //         req_servo_angle[3], 
    //         req_servo_angle[4], 
    //     map(req_servo_angle[5], 70, 90, 0, 70)
    // );  
}

void jointstates_callback( const sensor_msgs::JointState& joint)
{
    for(int i = 0; i < 7; i++)
    {
        req_servo_angle[i] = joint.position[i] * 57.2958;
    }
}

void gripper_callback( const std_msgs::Bool& state)
{
    if(state.data)
        req_servo_angle[6] = 1.217;
    else
        req_servo_angle[6] = 1.5708;
}