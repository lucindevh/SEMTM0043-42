#include "Controller.h"

const unsigned long Controller_c::UPDATE_INTERVAL_MS;
const unsigned long Controller_c::TELEMETRY_INTERVAL_MS;
const uint8_t Controller_c::LED_BRIGHTNESS;
const unsigned long Controller_c::ENCODER_TEST_T1_MS;
const unsigned long Controller_c::ENCODER_TEST_WINDOW_MS;
const unsigned long Controller_c::ENCODER_TEST_TIMEOUT_MS;
constexpr float Controller_c::ENCODER_TEST_LEFT_PWM;
constexpr float Controller_c::ENCODER_TEST_RIGHT_PWM;

Controller_c::Controller_c()
  : update_timer(UPDATE_INTERVAL_MS),
    telemetry_timer(TELEMETRY_INTERVAL_MS),
    led_set(false) {
  reset();
}

void Controller_c::reset() {
  signal = 0;
  encoders_fresh = false;
  et_phase = ET_START;
  et_start_ms = 0;
  et_t1_ms = 0;
  et_t2_ms = 0;
  et_left_count1 = 0;
  et_left_count2 = 0;
  et_right_count1 = 0;
  et_right_count2 = 0;
  // led_set is intentionally left alone so the LED is only written once.
}

uint8_t Controller_c::getSignal() const {
  return signal;
}

void Controller_c::setSignal(uint8_t requested_signal) {
  if (requested_signal == 0) {
    reset();
    return;
  }

  if (signal == 0) {
    signal = 1;
    et_phase = ET_START;
  }
}

void Controller_c::update(Robot_c &robot, RobotWifiAP_c &server) {

  // Get the current count of milliseconds since the StampC3 was powered on.
  unsigned long now = robot.getMillis();

  // Check if our TaskTimer_c for the general controller update indicates
  // that it is the correct time to operate.  Note, we give the TaskTimer_c
  // the current count in milliseconds.
  // We do this because we don't want to query the robot faster than at 10ms
  // intervals.
  if (!update_timer.isReady(now)) {

    // TaskTimer_c says it is not ready, so we simply end this call to
    // the controller here by calling return.
    return;
  }

  // The TaskTimer_c was ready, so we now reset the TaskTimer_c so it
  // begins counting again.  We now run the remaining controller code.
  update_timer.resetTimer(now);

  // The LED stays red, the colour of the encoder test. It is written once.
  if (!led_set) {
    led_set = true;
    robot.setLED(255, 0, 0, LED_BRIGHTNESS);
  }

  // While the encoder test runs, the surface and pose reads and the normal
  // telemetry are paused so only the test's own readings are shown. The
  // encoders are still acquired once per cycle because the test needs them.
  bool encoder_test_running = (signal == 1);

  // Ask the robot for latest information on surface reflectance sensors.
  if (!encoder_test_running) {
    robot.getSurfaceSensors();
  }

  // Ask the robot for the latest encoder information, and then use this
  // to update the odometry on board the StampC3 (see OdometryModel.h).
  // encoders_fresh records whether the cache holds a NEW snapshot this cycle.
  encoders_fresh = robot.getEncoders();
  if (encoders_fresh) {
    odometry.update(robot.getLeftEncoderCount(), robot.getRightEncoderCount());
  }

  // Pose estimation from the Pololu 3Pi robot itself.
  if (!encoder_test_running) {
    robot.getPose();
  }

  // While waiting, a button press starts the encoder test and sets signal to 1
  // (the transmitted signal also starts an armed Digital Twin live comparison).
  // Presses are ignored here while signal is 1, so the press that starts the
  // test can never also end it. isButtonPressed() is called in exactly one
  // place per state because it consumes the press when it returns true.
  if (signal == 0) {
    if (robot.isButtonPressed()) {
      setSignal(1);
    }
  }

  // The test has been started, so run it.
  if (signal == 1) {
    encoderTest(robot, server, now);
  }

  // Use another TaskTimer_c to limit how often we transmit telemetry data
  // on WiFi and Serial. Paused while the encoder test is running.
  if (!encoder_test_running && telemetry_timer.isReady(now)) {
    telemetry_timer.resetTimer(now);
    publishTelemetry(robot, server, now);

  }
}

void Controller_c::report(RobotWifiAP_c &server, const char *format, ...) {
  char buffer[160];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  server.send(buffer);          // WiFi client, if one is connected
  if (Serial) Serial.print(buffer);
}

void Controller_c::encoderTest(Robot_c &robot, RobotWifiAP_c &server, unsigned long now) {
  // Safety limit: if the test overruns (for example because encoder reads keep
  // failing), stop the motor rather than leaving it running.
  if ((et_phase == ET_WAIT_T1 || et_phase == ET_WAIT_T2) &&
      now - et_start_ms >= ENCODER_TEST_TIMEOUT_MS) {
    report(server, "ENC TIMEOUT after %lu ms: no result, stopping motor\n", now - et_start_ms);
    et_phase = ET_STOP;
  }

  if (et_phase == ET_START) {
    // Both motors. setMotorPWM(left, right). Timing starts only once the
    // command has been accepted.
    if (robot.setMotorPWM(ENCODER_TEST_LEFT_PWM, ENCODER_TEST_RIGHT_PWM)) {
      et_start_ms = now;
      et_phase = ET_WAIT_T1;
      report(server, "ENC start: left PWM %.0f, right PWM %.0f\n",
             ENCODER_TEST_LEFT_PWM, ENCODER_TEST_RIGHT_PWM);
    }
    return;
  }

  if (et_phase == ET_WAIT_T1) {
    // Re-send the command every cycle, as the other routines did, in case the
    // robot stops its motors when it is not refreshed.
    robot.setMotorPWM(ENCODER_TEST_LEFT_PWM, ENCODER_TEST_RIGHT_PWM);

    // First FRESH snapshot at or after t1. A failed read is skipped, so the
    // stale cached counts are never used. Both wheels come from this one
    // snapshot, so they share the same t1.
    if (encoders_fresh && now - et_start_ms >= ENCODER_TEST_T1_MS) {
      et_t1_ms = now;
      et_left_count1 = robot.getLeftEncoderCount();
      et_right_count1 = robot.getRightEncoderCount();
      et_phase = ET_WAIT_T2;
      report(server, "ENC t1 = %lu ms, left count = %ld, right count = %ld\n",
             et_t1_ms - et_start_ms, (long)et_left_count1, (long)et_right_count1);
    }
    return;
  }

  if (et_phase == ET_WAIT_T2) {
    robot.setMotorPWM(ENCODER_TEST_LEFT_PWM, ENCODER_TEST_RIGHT_PWM);

    if (encoders_fresh && now - et_t1_ms >= ENCODER_TEST_WINDOW_MS) {
      et_t2_ms = now;
      et_left_count2 = robot.getLeftEncoderCount();
      et_right_count2 = robot.getRightEncoderCount();
      unsigned long elapsed_ms = et_t2_ms - et_t1_ms;

      report(server, "ENC t2 = %lu ms, left count = %ld, right count = %ld\n",
             et_t2_ms - et_start_ms, (long)et_left_count2, (long)et_right_count2);
      if (elapsed_ms == 0) {
        report(server, "ENC invalid: zero elapsed time, no speed calculated\n");
      } else {
        printWheelSpeed(server, "left", et_left_count1, et_left_count2,
                        elapsed_ms, odometry.leftWheelRadiusMm);
        printWheelSpeed(server, "right", et_right_count1, et_right_count2,
                        elapsed_ms, odometry.rightWheelRadiusMm);
      }
      et_phase = ET_STOP;
    }
    return;
  }

  if (et_phase == ET_STOP) {
    // Explicit stop FIRST (reset() sends no motor command). Waiting for the
    // button only starts once the stop write succeeds; otherwise the stop is
    // retried next cycle.
    if (robot.setMotorPWM(0, 0)) {
      et_phase = ET_WAIT_BUTTON;
      report(server, "ENC motor stopped. Press the button to resume normal readings\n");
    }
    return;
  }

  if (et_phase == ET_WAIT_BUTTON) {
    // The motor is already stopped. Telemetry stays paused (signal is still 1),
    // so the lines above stay on screen. This is the only place the button is
    // read while the test runs, because isButtonPressed() consumes the press
    // when it returns true.
    if (robot.isButtonPressed()) {
      report(server, "ENC button pressed, normal readings resume\n");
      setSignal(0);   // soft reset: back to waiting, telemetry resumes
    }
  }
}

void Controller_c::printWheelSpeed(RobotWifiAP_c &server, const char *label, int32_t count1,
                                   int32_t count2, unsigned long elapsed_ms, float radius_mm) {
  // Current minus previous, forward positive (OdometryModel.h). The
  // subtraction is done on unsigned values so a counter wrap is harmless.
  int32_t delta = (int32_t)((uint32_t)count2 - (uint32_t)count1);

  // counts/s = delta / (elapsed in seconds); milliseconds converted explicitly.
  float counts_per_s = (float)delta * 1000.0f / (float)elapsed_ms;

  // mm/s = revolutions/s * wheel circumference. The radius and counts per
  // revolution are the NOMINAL values in odometry, not calibrated ones.
  float mm_per_s = counts_per_s / odometry.encoderCountsPerRevolution
                   * TWO_PI * radius_mm;

  report(server, "ENC %s: delta = %ld counts over %lu ms -> %.1f counts/s = %.1f mm/s\n",
         label, (long)delta, elapsed_ms, counts_per_s, mm_per_s);
}

void Controller_c::publishTelemetry(Robot_c &robot, RobotWifiAP_c &server, unsigned long timestamp_ms) {
  server.printf(
    "%lu,%.2f,%.2f,%.5f,%d,%d,%ld,%ld,%u,%u,%u,%u,%u,%u\n",
    timestamp_ms,
    odometry.pose.x,
    odometry.pose.y,
    odometry.pose.theta,
    robot.getLeftMotorPWM(),
    robot.getRightMotorPWM(),
    (long)robot.getLeftEncoderCount(),
    (long)robot.getRightEncoderCount(),
    robot.surface.reading[0],
    robot.surface.reading[1],
    robot.surface.reading[2],
    robot.surface.reading[3],
    robot.surface.reading[4],
    signal
  );
  if ( Serial ) { // If serial is connected
    Serial.printf(
      "%lu,%.2f,%.2f,%.5f,%d,%d,%ld,%ld,%u,%u,%u,%u,%u,%u\n",
      timestamp_ms,
      odometry.pose.x,
      odometry.pose.y,
      odometry.pose.theta,
      robot.getLeftMotorPWM(),
      robot.getRightMotorPWM(),
      (long)robot.getLeftEncoderCount(),
      (long)robot.getRightEncoderCount(),
      robot.surface.reading[0],
      robot.surface.reading[1],
      robot.surface.reading[2],
      robot.surface.reading[3],
      robot.surface.reading[4],
      signal
    );
  }
}