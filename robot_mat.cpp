#include <ESP8266WiFi.h>
#include "network_connection.h"
#include <RosController.h>
#include <RosAdapterRobot.h>
#include <RosAdapterSonar.h>
#include <RosAdapterPid.h>
#include <DifferentialWheeledRobot.h>

#include <RobotFactorySingleMotor.h> 
#include <RobotFactoryDualMotor.h>
#include <RosConfigSonar.h>
#include <Sonar.h>
#include <Servo.h>
#include <TaskScheduler.h>

#define ROBOT_SINGLE 

void setupWiFi();

void robot_ros_publish_callback();
void robot_kinematic_callback();
void robot_close_loop_callback();

Task robot_ros_publish_task(500, TASK_FOREVER, &robot_ros_publish_callback);
Task robot_close_loop_task(250, TASK_FOREVER, &robot_close_loop_callback);
Task robot_kinematic_task(250, TASK_FOREVER, &robot_kinematic_callback);


IPAddress server(192, 168, 1, 40); // IP address of the ROS server
const uint16_t serverPort = 11411; // Port of the ROS serial server

//Robot
RosController  * ros_controller = 0;
RobotBase * robot = 0;
Scheduler runner;

void setup() {

  Serial.begin(115200);

  setupWiFi();
  
  RosAdapterRobot * ros_adapter_robot = new RosAdapterRobot();
  RosAdapterSonar * ros_adapter_soner = new RosAdapterSonar();
  RosAdapterPid * ros_adapter_pid = new RosAdapterPid();
  
  ros_controller = new RosController(&server,serverPort);
  ros_controller->addNode(ros_adapter_robot);
  ros_controller->addNode(ros_adapter_soner);
  ros_controller->addNode(ros_adapter_pid);
  ros_controller->init();

  #ifdef ROBOT_SINGLE
  RosConfigArduinoDutySingleMotor * ros_config_motor = new RosConfigArduinoDutySingleMotor();
  #else
  RosConfigArduinoDutyDualMotor * ros_config_motor = new RosConfigArduinoDutyDualMotor();  
  #endif
  RosConfigSonar * ros_config_sonar = new RosConfigSonar();
   
  ros_controller->readConfiguration(ros_config_motor);
  ros_controller->readConfiguration(ros_config_sonar); 
       
  #ifdef ROBOT_SINGLE
  RobotFactory * factory = new  RobotFactorySingleMotor(ros_config_motor); 
  #else
  RobotFactory * factory = new  RobotFactoryDualMotor(ros_config_motor); 
  #endif
    
  robot =  factory->assembly();
  
  ros_adapter_robot->attachRobot(robot);
  ros_adapter_pid->attachWheel(robot->getWheels()); 

  //Servo
  Servo * servo = new Servo();
  servo->attach(ros_config_sonar->pin_servo);

  //Sonar 
  Sonar * sonar = new Sonar();
  sonar->attachTrigger(ros_config_sonar->pin_sonar_trigger);
  sonar->attachEcho(ros_config_sonar->pin_sonar_echo);
  sonar->attachServo(servo);

  ros_adapter_soner->attachSonar(sonar);

  runner.init();

  runner.addTask(robot_ros_publish_task);
  Serial.println("robot_ros_publish_task task created");

  runner.addTask(robot_close_loop_task);
  Serial.println("robot_close_loop_task task created");

  runner.addTask(robot_kinematic_task);
  Serial.println("robot_kinematic_task task created");


  Serial.println("robot_kinematic_task task  enabled");
 	robot_ros_publish_task.enable();

  Serial.println("robot_close_loop_task task enabled");
  robot_close_loop_task.enable();

  Serial.println("robot_kinematic_task task enabled"); 	
  robot_kinematic_task.enable();
 
}

void robot_ros_publish_callback() 
{
  if (ros_controller != 0)
  {
    ros_controller->update();
  }
}

void robot_kinematic_callback() 
{
  if (robot != 0) 
  {
    robot->update(0.250); //TODO MAKE SURE THIS VALUE 
  }
}

void robot_close_loop_callback()
{
  if (robot != 0) 
  {
    robot->update_close_loop(0.250); //TODO MAKE SURE THIS VALUE 
  }

  
  
}

void loop() {
    
  runner.execute();

}

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
