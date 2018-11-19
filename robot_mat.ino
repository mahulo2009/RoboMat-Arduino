#include <ESP8266WiFi.h>
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Range.h>
#include <rosserial_arduino/Test.h>
#include "network_connection.h"

#include <DifferentialWheeledRobot.h>
#include <RobotFactorySingleMotor.h>
#include <RobotFactoryDualMotor.h>

IPAddress server(192, 168, 1, 40); // IP address of the ROS server
const uint16_t serverPort = 11411; // Port of the ROS serial server

void setupWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if(i == 21){
    Serial.print("Could not connect to"); Serial.println(ssid);
    while(1) delay(500);
  }
  Serial.print("Ready! Use ");
  Serial.print(WiFi.localIP());
  Serial.println(" to access client");
}

//Robot
DifferentialWheeledRobot * robot = 0;

/*
WheelEncoder * wheel_left;
Pid * pid_left;

WheelEncoder * wheel_right;
Pid * pid_right;
*/

//Ros node handler
ros::NodeHandle nh;

void cmd_velCallback( const geometry_msgs::Twist& CVel) {
  nh.logdebug("cmd_velCallback");
  if (robot != 0 ) {
    robot->move(CVel.linear.x,CVel.angular.z);  
  }
}
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/car/cmd_vel", &cmd_velCallback );

void sonar_pose_callback( const geometry_msgs::Pose2D & sonar_pose) {
  nh.logdebug("sonar_pose_callback");
  if (robot != 0 ) {
    robot->sonarTo(sonar_pose.theta);
  }
}
ros::Subscriber<geometry_msgs::Pose2D> sonar_pose_sub("/car/sonar_pose", &sonar_pose_callback );

nav_msgs::Odometry odom_nav_msg;              
ros::Publisher odom_pub("/car/odom", &odom_nav_msg); 

sensor_msgs::Range ultrasonic_msg;   
ros::Publisher pub_ultrasonic("/car/ultrasound", &ultrasonic_msg);      

geometry_msgs::Vector3Stamped pid_telemetry_wheel_1_msg;              
ros::Publisher pid_telemetry_wheel_1_pub("/car/pid_telemetry_wheel_1_msg", &pid_telemetry_wheel_1_msg); 

geometry_msgs::Vector3Stamped pid_telemetry_wheel_2_msg;              
ros::Publisher pid_telemetry_wheel_2_pub("/car/pid_telemetry_wheel_2_msg", &pid_telemetry_wheel_2_msg); 

tf::TransformBroadcaster broadcaster;
geometry_msgs::TransformStamped odom_trans;
geometry_msgs::TransformStamped ultrasonic_trans;   
geometry_msgs::Twist odom_geometry_msg;

ros::Time current_time = nh.now();
ros::Time last_time = current_time;

using rosserial_arduino::Test;
void callback(const Test::Request & req, Test::Response & res){
  /*
  res.output = "hello";

  float pid_kp, pid_ki, pid_kd;
  if (! nh.getParam("/robomat/pid_kp", &pid_kp) ) 
    { pid_kp = 1.0;};
  if (! nh.getParam("/robomat/pid_ki", &pid_ki) ) 
    { pid_ki = 0.0;};
  if (! nh.getParam("/robomat/pid_kd", &pid_kd) ) 
    { pid_kd = 0.0;};

  pid_left->reset();
  pid_left->setKp(pid_kp);
  pid_left->setKi(pid_ki);
  pid_left->setKd(pid_kd);

  wheel_left->stop();
  delay(1000);
  wheel_left->move(18.0);
  */
}

ros::ServiceServer<Test::Request, Test::Response> server_1("test_srv",&callback);

void setup() {
  //Setup Serial line.
  Serial.begin(115200);
  //Setup Wifi
  setupWiFi();
  delay(2000);
  
  //Init rose node & subscribe
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  broadcaster.init(nh);
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(sonar_pose_sub);
  nh.advertise(odom_pub);
  nh.advertise(pub_ultrasonic);
  nh.advertise(pid_telemetry_wheel_1_pub);
  nh.advertise(pid_telemetry_wheel_2_pub);
  nh.advertiseService(server_1);

  //Configure ultrasonic
  ultrasonic_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  ultrasonic_msg.header.frame_id = "/ultrasound";   
  ultrasonic_msg.field_of_view = 0.1;
  ultrasonic_msg.min_range = 0.0;
  ultrasonic_msg.max_range = 20;

  while(!nh.connected()) {nh.spinOnce();}

  int robot_type;
  if (! nh.getParam("/robomat/robot_type", &robot_type) ) 
    { robot_type = 1.0;};

  params_t params_robot;
  params_t params_left;
  params_t params_right;

  //max_speed
  if (! nh.getParam("/robomat/max_speed", &params_robot.max_speed) ) 
    { params_robot.max_speed = 11;};   

  //pin_power_left,pin_power_right
  if (! nh.getParam("/robomat/pin_power_left", &params_left.pin_power) ) 
    { params_left.pin_power = 4;};
  if (! nh.getParam("/robomat/pin_power_right", &params_right.pin_power) ) 
    { params_right.pin_power = 5;};

  //power_min,power_max;
  if (! nh.getParam("/robomat/power_min", &params_robot.power_min) ) 
    { params_robot.power_min = 256;};
  if (! nh.getParam("/robomat/power_max", &params_robot.power_max) ) 
    { params_robot.power_max = 1023;};

  //float robot_wheel_separation,robot_wheel_radious;
  if (! nh.getParam("/robomat/robot_wheel_separation", &params_robot.robot_wheel_separation) ) 
    { params_robot.robot_wheel_separation = 0.135;};
  if (! nh.getParam("/robomat/robot_wheel_radious", &params_robot.robot_wheel_radious) ) 
    { params_robot.robot_wheel_radious = 0.0325;};

  int pin_sonar_trigger, pin_sonar_echo, pin_servo;
  if (! nh.getParam("/robomat/pin_sonar_trigger", &pin_sonar_trigger) ) 
    { pin_sonar_trigger = 16;};
  if (! nh.getParam("/robomat/pin_sonar_echo", &pin_sonar_echo) ) 
    { pin_sonar_echo = 15;};
  if (! nh.getParam("/robomat/pin_servo", &pin_servo) ) 
    { pin_servo = 15;};

  if (robot_type == 1) {

    if (! nh.getParam("/robomat/pid_kp", &params_robot.pid_kp) ) 
      { params_robot.pid_kp = 1.0;};
    if (! nh.getParam("/robomat/pid_ki", &params_robot.pid_ki) ) 
      { params_robot.pid_ki = 0.0;};
    if (! nh.getParam("/robomat/pid_kd", &params_robot.pid_kd) ) 
      { params_robot.pid_kd = 0.0;};

    //encoder_ticks_per_revoloution;
    if (! nh.getParam("/robomat/encoder_ticks_per_revoloution", &params_robot.encoder_ticks_per_revoloution) ) 
      { params_robot.encoder_ticks_per_revoloution = 21;};

    //int pin_encoder_left,pin_encoder_right;
    if (! nh.getParam("/robomat/pin_encoder_left", &params_left.pin_encoder) ) 
      { params_left.pin_encoder = 12;};
    if (! nh.getParam("/robomat/pin_encoder_right", &params_right.pin_encoder) ) 
      { params_right.pin_encoder = 14;};

    //int pin_direction_left,pin_direction_right;
    if (! nh.getParam("/robomat/pin_direction_left", &params_left.pin_direction) ) 
      { params_left.pin_direction = 2;};
    if (! nh.getParam("/robomat/pin_direction_right", &params_right.pin_direction) ) 
      { params_right.pin_direction = 0;};

    //dinamic cast not available on arduino.
    RobotFactory * factory = new  RobotFactorySingleMotor(); 
    robot = (DifferentialWheeledRobot* ) ( factory->assembly(params_robot,params_left,params_right) ) ;

  } else if (robot_type == 2) {
    //int pin_direction_left_1,pin_direction_left_2;
    if (! nh.getParam("/robomat/pin_direction_left_1", &params_left.pin_direction_1) ) 
      { params_left.pin_direction_1 = 2;};          
    if (! nh.getParam("/robomat/pin_direction_left_2", &params_left.pin_direction_2) ) 
      { params_left.pin_direction_2 = 15;};

    //int pin_direction_right_1,pin_direction_right_2;
    if (! nh.getParam("/robomat/pin_direction_right_1", &params_right.pin_direction_1) ) 
      { params_right.pin_direction_1 = 12;};   
    if (! nh.getParam("/robomat/pin_direction_right_2", &params_right.pin_direction_2) ) 
      { params_right.pin_direction_2 = 14;};

    //dinamic cast not available on arduino.
    RobotFactory * factory = new  RobotFactoryDualMotor(); 
    robot = (DifferentialWheeledRobot * ) ( factory->assembly(params_robot,params_left,params_right) ) ;
  }
  
  //Servo
  Servo * servo = new Servo();
  servo->attach(pin_servo);

  //Sonar 
  Sonar * sonar = new Sonar();
  sonar->attachTrigger(pin_sonar_trigger);
  sonar->attachEcho(pin_sonar_echo);
  sonar->attachServo(servo);

  //Robot
  robot->attachSonar(sonar);
}

void loop() {
  if (nh.connected()) {

    current_time = nh.now();
    double dt = current_time.toSec() - last_time.toSec();
    
    robot->update(dt);
/*
    pid_telemetry_wheel_1_msg.header.stamp = current_time;
    pid_telemetry_wheel_1_msg.vector.x=wheel_left->getTargetVelocity();
    pid_telemetry_wheel_1_msg.vector.y=wheel_left->getVelocity();
    pid_telemetry_wheel_1_msg.vector.z=wheel_left->getDemandedVelocity();
    pid_telemetry_wheel_1_pub.publish(&pid_telemetry_wheel_1_msg);   

    pid_telemetry_wheel_2_msg.header.stamp = current_time;
    pid_telemetry_wheel_2_msg.vector.x=wheel_right->getTargetVelocity();
    pid_telemetry_wheel_2_msg.vector.y=wheel_right->getVelocity();
    pid_telemetry_wheel_2_msg.vector.z=wheel_right->getDemandedVelocity();
    pid_telemetry_wheel_2_pub.publish(&pid_telemetry_wheel_2_msg);   

*/
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(robot->getTheta()); //TODO REVIEW THIS THE ANGLE IS NOT CORRECT
    // tf odom->base_link
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "/odom";
    odom_trans.child_frame_id = "/base_link";
    odom_trans.transform.translation.x = robot->getX();
    odom_trans.transform.translation.y = robot->getY();
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    broadcaster.sendTransform(odom_trans);
 
    //BEGIN odometry  
    odom_nav_msg.header.stamp = current_time;
    odom_nav_msg.header.frame_id = "/odom";
    //set the position
    odom_nav_msg.pose.pose.position.x = robot->getX();
    odom_nav_msg.pose.pose.position.y = robot->getY();
    odom_nav_msg.pose.pose.position.z = 0.0;
    odom_nav_msg.pose.pose.orientation = odom_quat;
    //set the velocity
    odom_nav_msg.child_frame_id = "/base_link";
    odom_nav_msg.twist.twist.linear.x = robot->getVx();
    odom_nav_msg.twist.twist.linear.y = robot->getVy();
    odom_nav_msg.twist.twist.angular.z = robot->getVtheta();
    odom_pub.publish(&odom_nav_msg);
    //END odometry  

    //BEGIN Ultrasonic
    ultrasonic_trans.header.frame_id = "/base_link";
    ultrasonic_trans.child_frame_id = "/ultrasound";
    ultrasonic_trans.transform.translation.x = 0.045; 
    ultrasonic_trans.transform.translation.y = 0.0; 
    ultrasonic_trans.transform.translation.z = 0.075;
    ultrasonic_trans.transform.rotation = tf::createQuaternionFromYaw(robot->getSonarAngle()); //TODO INCLUDE ORIENTATION + RELATIVE POSITION SENSOR.
    ultrasonic_trans.header.stamp = current_time;
    broadcaster.sendTransform(ultrasonic_trans);

    ultrasonic_msg.range = robot->getDistance();
    ultrasonic_msg.header.stamp = current_time;
    pub_ultrasonic.publish(&ultrasonic_msg);    
    //END Ultrasonic

    last_time = current_time;

    //Ros spin once
    nh.spinOnce(); 
    //Delay
    delay(100);  //TODO REDUCE THIS
  } 
}