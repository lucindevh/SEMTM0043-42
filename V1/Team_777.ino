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


/** @brief Initialises serial, WiFi, IMU, and the supplied robot-controller baseline. */
void setup() {

  Serial.begin(115200);
  delay(500);
  Serial.println("=== BOOT ===");

  server.begin();

  robot.initialise("Team777");
  Serial.println("=== ROBOT OK ===");

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

  // Accept or maintain the optional Processing telemetry connection. The
  // controller continues to run even when no client is connected.
  server.update();

  // Run an iteration of the robot controller code. The controller owns the
  // button: it drives the LED menu and starts/stops each routine.
  controller.update(robot, server, imu);
}