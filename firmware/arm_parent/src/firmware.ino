#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <StepControl.h>

#define STEP_PIN 3
#define DIR_PIN 2

#define LINEAR_ACTUATOR_SPEED 0.01
#define TORSO_MIN_HEIGHT 0.21
#define TORSO_MAX_HEIGHT 0.90

#define MOTOR_IN_A 10
#define MOTOR_IN_B 9

#define STEP_PER_SEC 4500
#define STEPS_PER_REV 1600
#define DIST_PER_REV 0.00812 //m


Stepper motor(STEP_PIN, DIR_PIN);         
StepControl<> controller; 

double req_joint_state[7];
double arm_height;

int lower_limit;
int upper_limit;

void jointstates_callback( const sensor_msgs::JointState& joint);
void gripper_callback( const std_msgs::Bool& state);

ros::NodeHandle nh;

sensor_msgs::JointState joints;
ros::Publisher jointstates_pub("onine/joint_states", &joints);
ros::Subscriber<sensor_msgs::JointState> joinstates_sub("move_group/fake_controller_joint_states", jointstates_callback);
ros::Subscriber<std_msgs::Bool> gripper_sub("onine_gripper", gripper_callback);

void setup() 
{
    motor.setAcceleration(abs(STEP_PER_SEC*10));

    pinMode(MOTOR_IN_A, OUTPUT);
    pinMode(MOTOR_IN_B, OUTPUT);

    init_arm();

    joints.name_length = 8;
    joints.velocity_length = 8;
    joints.position_length = 8; 
    joints.effort_length = 8; 

    Serial3.begin(2000);
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

    move_z(-1);

    delay(1);
}

void loop() 
{ 
    static unsigned long prev_pub_time = 0;
    static unsigned long prev_torso_time = 0;
    if((millis() -  prev_pub_time) >= 100)
    {
        publish_joints();
        prev_pub_time = millis();
    }

    move_arm();
    move_torso();

    nh.spinOnce();
}

void move_torso()
{
    static bool is_done = true;
    static long int pos = 0;
    float est_distance = 0;
    static float ref_height = 0.00;
    static float height_diff = 0.00;

    if((arm_height - req_joint_state[3]) > 0.005)
    {
        if(is_done)
        {
            move_z(-1);
            is_done = false;
            ref_height = arm_height;
        }
        else
        {
            height_diff =  (float) ( (motor.getPosition() / STEPS_PER_REV) * DIST_PER_REV);
            arm_height = ref_height - height_diff;
        }
        nh.loginfo("going down");
    }

    else if((arm_height - req_joint_state[3]) < -0.005)
    {
        if(is_done)
        {
            move_z(1);
            is_done = false;
            ref_height = arm_height;
        }
        else
        {
            height_diff =  (float) ( (motor.getPosition() / STEPS_PER_REV) * DIST_PER_REV);
            arm_height = ref_height - height_diff;
        }
        nh.loginfo("going up");
    }

    else
    {   
        if(!is_done)
        {
            is_done = true;
            controller.stop();  
            height_diff =  (float) ( (motor.getPosition() / STEPS_PER_REV) * DIST_PER_REV);
            arm_height = ref_height - height_diff;
            nh.loginfo("stopped");
        }
    }
}

void move_arm()
{
    get_height_state();

    Serial3.print(rad_to_deg(req_joint_state[0]));
    Serial3.print('b');

    Serial3.print(rad_to_deg(req_joint_state[2]));
    Serial3.print('s');

    Serial3.print(rad_to_deg(req_joint_state[1]));
    Serial3.print('e');

    Serial3.print(rad_to_deg(req_joint_state[4]));
    Serial3.print('r');

    Serial3.print(rad_to_deg(req_joint_state[5]));
    Serial3.print('p');

    Serial3.print(map(rad_to_deg(req_joint_state[6]), 70, 90, 0, 70));
    Serial3.print('g');
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

void move_z(int dir)
{
    motor.setPosition(0); 
    motor.setMaxSpeed(-dir * STEP_PER_SEC);        
    controller.rotateAsync(motor); 
}

void init_arm()
{
    arm_height = TORSO_MIN_HEIGHT;
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

    while (Serial3.available())
    {
        char character = Serial3.read(); 
        serial_string.concat(character); 

        if (character == 'l')
        {
            lower_limit = serial_string.toInt();
            serial_string = "";

            if(lower_limit > 0)
            {
                arm_height = TORSO_MAX_HEIGHT;
                controller.stop();    
            }
            else if(lower_limit < 0)
            {
                arm_height = TORSO_MIN_HEIGHT;
                controller.stop();
                motor.setPosition(0);
                motor.setTargetAbs(-400);
                controller.moveAsync(motor);
            }

        }
    }
}

double rad_to_deg(double angle)
{
    return angle * 57.2958;
}