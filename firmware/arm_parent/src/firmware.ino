#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

#define LINEAR_ACTUATOR_SPEED 0.01
#define TORSO_MIN_HEIGHT 0.21

#define MOTOR_IN_A 9
#define MOTOR_IN_B 10

double req_joint_state[7];
double arm_height;
int lower_limit;
int upper_limit;

void jointstates_callback( const sensor_msgs::JointState& joint);
void gripper_callback( const std_msgs::Bool& state);

ros::NodeHandle  nh;

sensor_msgs::JointState joints;
ros::Publisher jointstates_pub("braccio/joint_states", &joints);
ros::Subscriber<sensor_msgs::JointState> joinstates_sub("move_group/fake_controller_joint_states", jointstates_callback);
ros::Subscriber<std_msgs::Bool> gripper_sub("braccio_gripper", gripper_callback);

void setup() 
{
    pinMode(MOTOR_IN_A, OUTPUT);
    pinMode(MOTOR_IN_B, OUTPUT);

    init_arm();

    joints.name_length = 8;
    joints.velocity_length = 8;
    joints.position_length = 8; 
    joints.effort_length = 8; 

    Serial1.begin(2000);
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(joinstates_sub);
    nh.subscribe(gripper_sub);
    nh.advertise(jointstates_pub);

    while (!nh.connected())
    {
        nh.spinOnce();
    }

    nh.loginfo("ONINE CONNECTED!");
    delay(1);
}

void loop() 
{ 
    static unsigned long prev_pub_time = 0;

    if((millis() -  prev_pub_time) >= 100)
    {
        publish_joints();
        prev_pub_time = millis();
    }

    move_arm();

    get_height_state();
  
    nh.spinOnce();
}

void move_arm()
{
    if((arm_height - req_joint_state[3]) > 0.005)
    {
        move_z(80);
        nh.loginfo("going down");
    }

    else if((arm_height - req_joint_state[3]) < -0.005)
    {
        move_z(-80);
        nh.loginfo("going up");
    }

    else
    {
        move_z(0);
    }

    char log_msg[50];    
    char result[8];
    dtostrf(arm_height, 6, 2, result);
    sprintf(log_msg,"Arm Height = %s", result);
    nh.loginfo(log_msg);

    Serial1.print(rad_to_deg(req_joint_state[0]));
    Serial1.print('b');

    Serial1.print(rad_to_deg(req_joint_state[2]));
    Serial1.print('s');

    Serial1.print(rad_to_deg(req_joint_state[1]));
    Serial1.print('e');

    Serial1.print(rad_to_deg(req_joint_state[4]));
    Serial1.print('r');

    Serial1.print(rad_to_deg(req_joint_state[5]));
    Serial1.print('p');

    Serial1.print(map(rad_to_deg(req_joint_state[6]), 70, 90, 0, 70));
    Serial1.print('g');
}

void jointstates_callback( const sensor_msgs::JointState& joint)
{
    for(int i = 0; i < 5; i++)
    {
        req_joint_state[i] = joint.position[i]; 
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
    if (speed > 0)
    {
        analogWrite(MOTOR_IN_A, 0);
        analogWrite(MOTOR_IN_B, abs(speed));
    }
    else if (speed < 0)
    {
        analogWrite(MOTOR_IN_B, 0);
        analogWrite(MOTOR_IN_A, abs(speed));
    }
    else
    {
        analogWrite(MOTOR_IN_B, 0);
        analogWrite(MOTOR_IN_A, 0);
    }
}

void init_arm()
{
    req_joint_state[0] = 1.57;
    req_joint_state[1] = 0.00;
    req_joint_state[2] = 1.57;
    req_joint_state[3] = TORSO_MIN_HEIGHT;
    req_joint_state[4] = 0.00;
    req_joint_state[5] = 0.00;
    req_joint_state[6] = 1.39;
}

void publish_joints()
{
    joints.header.frame_id = "";

    char * joints_name[8] = {"base_joint", "elbow_joint", "shoulder_joint", "torso_joint", "wrist_pitch_joint", "wrist_roll_joint", "gripper_joint", "sub_gripper_joint"};
    double joints_position[8];
    
    for(int i = 0; i < 8; i ++)
    {
        if(i == 3)
            joints_position[i] = arm_height;
        else if(i == 7)
            joints_position[i] = joints_position[i-1];
        else
            joints_position[i] = req_joint_state[i];
    }

    joints.position = joints_position;
    joints.name = joints_name;

    jointstates_pub.publish(&joints);
}

double get_arm_height()
{
    return req_joint_state[3];
}

void get_height_state()
{
    static String serial_string = "";

    while (Serial1.available())
    {
        char character = Serial1.read(); 
        serial_string.concat(character); 

        if (character == 'h')
        {
            arm_height = ((serial_string.toInt()) / 1000.00);

            float offset;

            if(arm_height < 0.5)
                offset = 0.01;
            else
                offset = 0.035;

            arm_height = arm_height + offset;
            
            serial_string = "";
        }

        else if (character == 'u')
        {
            upper_limit = serial_string.toInt();
            serial_string = "";
        }

        else if (character == 'l')
        {
            lower_limit = serial_string.toInt();
            serial_string = "";
        }
    }
}

double rad_to_deg(double angle)
{
    return angle * 57.2958;
}