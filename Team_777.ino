#include <Wire.h>

#include "RobotWifiAP.h"

/** @brief WiFi access-point and TCP server used for optional telemetry. */
RobotWifiAP_c server("lvhAP", "----lvhAP", 80);

#include "Robot.h"
#include "RobotIMU.h"
#include "Controller.h"


/** @brief Hardware-facing interface to the Pololu robot. */
Robot_c robot;
/** @brief Direct interface to the 3Pi+ accelerometer, gyro, and magnetometer. */
RobotIMU_c imu;
/** @brief Supplied nominal line-following controller. */
Controller_c controller;

//debounce state for the user button
static bool last_button_state = false;


/** @brief Initialises serial, WiFi, IMU, and the supplied robot-controller baseline. */
void setup() {

  Serial.begin(115200);
  delay(500);
  Serial.println("=== BOOT ===");

  server.begin();

  robot.initialise("Team777");
  Serial.println("=== ROBOT OK ===");
//  robot.initialise("team name");


  // Robot_c starts the shared I2C bus before RobotIMU_c uses it. IMU readings
  // come directly from the sensor ICs and do not pass through the middleware.
  if (!imu.initialise() ) {
    Serial.println("Warning: LSM6DS33 IMU was not detected.");
    robot.setLED( 90, 50, 0, 100 ); // orange warning
  }
  Serial.println("=== IMU DONE ===");
}


/** @brief Maintains telemetry transport and runs the controller repeatedly. */
void loop() {

  bool button_now = robot.isButtonPressed();

  if (button_now && !last_button_state) {
    if (controller.getSignal() == 0) {
      controller.setSignal(1);
      Serial.println("# START");
    } else {
      controller.setSignal(0);
      Serial.println("# STOP");
      robot.setMotorPWM(0, 0);
    }
  }

  last_button_state = button_now;

  // Accept or maintain the optional Processing telemetry connection. The
  // controller continues to run even when no client is connected.
  server.update();

  
  // Run an iteration of the robot controller code.
  // Develop controller routines in Controller.cpp; edit parameters in Controller.h.
  controller.update(robot, server);
}
