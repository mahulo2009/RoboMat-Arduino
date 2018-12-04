#include <ESP8266WiFi.h>
#include "network_connection.h"
#include <RosController.h>
#include <RosAdapterRobot.h>
#include <RosAdapterSonar.h>
#include <DifferentialWheeledRobot.h>

#include <RobotFactorySingleMotor.h> //TODO WHY I CAN NOT REMOVE THIS
#include <RobotFactoryDualMotor.h>
#include <RosConfigSonar.h>
#include <Sonar.h>
#include <Servo.h>

#define ROBOT_SINGLE 

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

IPAddress server(192, 168, 1, 40); // IP address of the ROS server
const uint16_t serverPort = 11411; // Port of the ROS serial server

//Robot
RosController  * ros_controller = 0;
RobotBase * robot = 0;

void setup() {

  Serial.begin(115200);

  setupWiFi();

  delay(2000);

  RosAdapterRobot * ros_adapter_robot = new RosAdapterRobot();
  RosAdapterSonar * ros_adapter_soner = new RosAdapterSonar();
  
  ros_controller = new RosController(&server,serverPort);
  ros_controller->addNode(ros_adapter_robot);
  ros_controller->addNode(ros_adapter_soner);
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

  //Servo
  Servo * servo = new Servo();
  servo->attach(ros_config_sonar->pin_servo);

  //Sonar 
  Sonar * sonar = new Sonar();
  sonar->attachTrigger(ros_config_sonar->pin_sonar_trigger);
  sonar->attachEcho(ros_config_sonar->pin_sonar_echo);
  sonar->attachServo(servo);

  ros_adapter_soner->attachSonar(sonar);
}

void loop() {
    
  if (ros_controller != 0)
  {
    ros_controller->update();
  }

  if (robot != 0) 
  {
    robot->update(0.1); 
  }

}

