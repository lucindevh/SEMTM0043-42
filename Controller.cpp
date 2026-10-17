#include "Controller.h"

const unsigned long Controller_c::UPDATE_INTERVAL_MS;
const unsigned long Controller_c::TELEMETRY_INTERVAL_MS;
const unsigned long Controller_c::RECOVERY_TIMEOUT_MS;
const uint16_t Controller_c::LINE_THRESHOLD;
constexpr float Controller_c::BASE_PWM;
constexpr float Controller_c::LINE_FOLLOW_GAIN;
constexpr float Controller_c::RECOVERY_LEFT_PWM;
constexpr float Controller_c::RECOVERY_RIGHT_PWM;
const unsigned long Controller_c::MENU_INTERVAL_MS;
const unsigned int Controller_c::MOTOR_TEST_CALLS;
constexpr float Controller_c::MOTOR_TEST_MAX_PWM;
const uint8_t Controller_c::MENU_LED_BRIGHTNESS;
const unsigned long Controller_c::ENCODER_TEST_T1_MS;
const unsigned long Controller_c::ENCODER_TEST_WINDOW_MS;
const unsigned long Controller_c::ENCODER_TEST_TIMEOUT_MS;
constexpr float Controller_c::ENCODER_TEST_LEFT_PWM;
constexpr float Controller_c::ENCODER_TEST_RIGHT_PWM;

// ---------------------------------------------------------------------------
// Construction / reset
// ---------------------------------------------------------------------------

Controller_c::Controller_c()
  : update_timer(UPDATE_INTERVAL_MS),
    telemetry_timer(TELEMETRY_INTERVAL_MS),
    selected_routine(ROUTINE_LINE_FOLLOW),
    menu_timer(MENU_INTERVAL_MS),
    menu_started(false) {
  reset();
  trial_num = 0;   // only set here: reset() must not wipe it between trials
  odometry.setLeftWheelRadiusMm(16.35);
  odometry.setRightWheelRadiusMm(16.35);
  odometry.setWheelSeparationMm(87.0);
}

void Controller_c::reset() {
  signal = 0;
  mode = WAITING;
  recovery_start_ms = 0;
  call_count = 0;
  active_routine = ROUTINE_LINE_FOLLOW;
  led_dirty = true;   // redraw the menu LED next time the menu runs
  encoders_fresh = false;
  et_phase = ET_START;
  et_start_ms = 0;
  et_t1_ms = 0;
  et_t2_ms = 0;
  et_left_count1 = 0;
  et_left_count2 = 0;
  et_right_count1 = 0;
  et_right_count2 = 0;
  last_surface_us = 0;
  sample_num = 0;
  // selected_routine and menu_timer are intentionally left alone.
  // trial_num is NOT reset here: it counts trials across button presses.
}

// ---------------------------------------------------------------------------
// Signal handling
// ---------------------------------------------------------------------------

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
    mode = FOLLOWING_LINE;
    recovery_start_ms = 0;
  }
}

// ---------------------------------------------------------------------------
// Main update loop
// ---------------------------------------------------------------------------

void Controller_c::update(Robot_c &robot, RobotWifiAP_c &server) {

  // Get the current count of milliseconds since the StampC3 was powered on.
  unsigned long now = robot.getMillis();

  // Gate everything on the 10 ms update timer so we don't query the robot
  // faster than that.
  if (!update_timer.isReady(now)) {
    return;
  }
  update_timer.resetTimer(now);

  // While the encoder test runs, the surface and pose reads and the normal
  // telemetry are paused so only the test's own readings are shown.
  bool encoder_test_running = (signal == 1 && active_routine == ROUTINE_ENCODER_TEST);

  // The odometry test and the sensor test print CSV rows to Serial, so the
  // normal telemetry is paused for them too (it would corrupt the CSV).
  bool csv_logging_running = (signal == 1 &&
                              (active_routine == ROUTINE_ODOMETRY_TEST ||
                               active_routine == ROUTINE_SENSOR_TEST));

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

    // Odometry test: one CSV row per fresh encoder update.
    if (signal == 1 && active_routine == ROUTINE_ODOMETRY_TEST) {
      if (rotation_test) {
        reportOdometryRotation(robot, now);
      } else {
        reportOdometryTranslation(robot, now);
      }
    }
  }

  // Pose estimation from the Pololu 3Pi robot itself.
  if (!encoder_test_running) {
    robot.getPose();
  }

  // While waiting, the LED menu steps between routines every MENU_INTERVAL_MS.
  // A button press starts whichever routine the LED is currently showing and
  // sets signal to 1 (the transmitted signal also starts an armed Digital Twin
  // live comparison). Presses are ignored while signal is 1, so the press that
  // starts a routine can never also reset it. isButtonPressed() is called in
  // exactly one place per routine because it consumes the press when it
  // returns true.
  if (signal == 0) {
    updateMenu(robot, now);
    if (robot.isButtonPressed()) {
      active_routine = selected_routine;
      call_count = 0;
      et_phase = ET_START;
      setSignal(1);

      // Odometry test needs its own set-up when it starts.
      if (active_routine == ROUTINE_ODOMETRY_TEST) {
        odometry.reset();
        move_attempted = false;
        move_started = false;
        move_running = false;
        move_finished = false;
        press_ms = now;
        if (Serial) {
          if (rotation_test) {
            Serial.println("test_name,trial_num,commanded_angle_rad,sample_time_ms,left_encoder_count,right_encoder_count,odometry_x_mm,odometry_y_mm,odometry_theta_rad");
          } else {
            Serial.println("test_name,trial_num,commanded_distance_mm,sample_time_ms,left_encoder_count,right_encoder_count,odometry_x_mm,odometry_y_mm,odometry_theta_rad");
          }
        }
      }
    }
  }

  // A routine has been started, so run the one that was selected.
  if (signal == 1) {
    if (active_routine == ROUTINE_MOTOR_TEST) {
      motorTest(robot, now);
    } else if (active_routine == ROUTINE_ENCODER_TEST) {
      encoderTest(robot, now);
    } else if (active_routine == ROUTINE_ODOMETRY_TEST) {
      odometryTest(robot, now);
    } else if (active_routine == ROUTINE_SENSOR_TEST) {
      // ---- Choose ONE sensor test function to run below ----
      testOffset(robot, now);       // Exercise 2
      // sensorTest(robot, now);    // Exercise 1

      // The sensor test runs until the button is pressed again.
      if (robot.isButtonPressed()) {
        robot.setMotorPWM(0, 0);
        menu_timer.resetTimer(now);
        setSignal(0);               // soft reset: back to the menu
      }
    } else {
      runLineFollower(robot, now);
    }
  }

  // Use another TaskTimer_c to limit how often we transmit telemetry data
  // on WiFi and Serial. Paused while the encoder test, odometry test or
  // sensor test is running.
  if (!encoder_test_running && !csv_logging_running && telemetry_timer.isReady(now)) {
    telemetry_timer.resetTimer(now);
    publishTelemetry(robot, server, now);
  }
}

// ---------------------------------------------------------------------------
// LED menu
// ---------------------------------------------------------------------------

void Controller_c::updateMenu(Robot_c &robot, unsigned long now) {
  if (!menu_started) {
    menu_started = true;
    menu_timer.resetTimer(now);
  }

  if (menu_timer.isReady(now)) {
    menu_timer.resetTimer(now);
    // White -> green -> red -> blue -> yellow -> white ...
    if (selected_routine == ROUTINE_LINE_FOLLOW) {
      selected_routine = ROUTINE_MOTOR_TEST;
    } else if (selected_routine == ROUTINE_MOTOR_TEST) {
      selected_routine = ROUTINE_ENCODER_TEST;
    } else if (selected_routine == ROUTINE_ENCODER_TEST) {
      selected_routine = ROUTINE_ODOMETRY_TEST;
    } else if (selected_routine == ROUTINE_ODOMETRY_TEST) {
      selected_routine = ROUTINE_SENSOR_TEST;
    } else {
      selected_routine = ROUTINE_LINE_FOLLOW;
    }
    led_dirty = true;
  }

  // Only touch the LED when it needs to change: setLED() waits on the RMT
  // peripheral, so calling it every 10 ms cycle would add needless delay.
  if (led_dirty) {
    led_dirty = false;
    if (selected_routine == ROUTINE_LINE_FOLLOW) {
      robot.setLED(255, 255, 255, MENU_LED_BRIGHTNESS);   // white
    } else if (selected_routine == ROUTINE_MOTOR_TEST) {
      robot.setLED(0, 255, 0, MENU_LED_BRIGHTNESS);       // green
    } else if (selected_routine == ROUTINE_ENCODER_TEST) {
      robot.setLED(255, 0, 0, MENU_LED_BRIGHTNESS);       // red
    } else if (selected_routine == ROUTINE_ODOMETRY_TEST) {
      robot.setLED(0, 0, 255, MENU_LED_BRIGHTNESS);       // blue
    } else {
      robot.setLED(255, 255, 0, MENU_LED_BRIGHTNESS);     // yellow
    }
  }
}

// ---------------------------------------------------------------------------
// Motor test
// ---------------------------------------------------------------------------

void Controller_c::motorTest(Robot_c &robot, unsigned long now) {
  if (call_count >= MOTOR_TEST_CALLS) {
    // Finished. Send an explicit stop FIRST: reset() sends no motor command.
    // Only return to the menu once the stop write has succeeded; otherwise
    // signal stays 1 and the stop is retried on the next cycle.
    if (robot.setMotorPWM(0, 0)) {
      menu_timer.resetTimer(now);   // show the menu colour for a full interval
      setSignal(0);                 // soft reset: back to the menu
    }
    return;
  }

  // Ramp 0 -> MOTOR_TEST_MAX_PWM across calls 0 .. MOTOR_TEST_CALLS-1.
  float pwm = MOTOR_TEST_MAX_PWM * (float)call_count / (float)(MOTOR_TEST_CALLS - 1);
  call_count = call_count + 1;

  // Opposite signs: the left wheel is driven negative, the right positive.
  // Which way each wheel visibly turns is a convention: check with the encoders.
  robot.setMotorPWM(-pwm, pwm);
}

// ---------------------------------------------------------------------------
// Encoder test
// ---------------------------------------------------------------------------

void Controller_c::encoderTest(Robot_c &robot, unsigned long now) {
  // Safety limit: if the test overruns (for example because encoder reads keep
  // failing), stop the motor rather than leaving it running.
  if ((et_phase == ET_WAIT_T1 || et_phase == ET_WAIT_T2) &&
      now - et_start_ms >= ENCODER_TEST_TIMEOUT_MS) {
    if (Serial) {
      Serial.printf("ENC TIMEOUT after %lu ms: no result, stopping motor\n", now - et_start_ms);
    }
    et_phase = ET_STOP;
  }

  if (et_phase == ET_START) {
    // Both motors. setMotorPWM(left, right). Timing starts only once the
    // command has been accepted.
    if (robot.setMotorPWM(ENCODER_TEST_LEFT_PWM, ENCODER_TEST_RIGHT_PWM)) {
      et_start_ms = now;
      et_phase = ET_WAIT_T1;
      if (Serial) {
        Serial.printf("ENC start: left PWM %.0f, right PWM %.0f\n",
                      ENCODER_TEST_LEFT_PWM, ENCODER_TEST_RIGHT_PWM);
      }
    }
    return;
  }

  if (et_phase == ET_WAIT_T1) {
    // First FRESH snapshot at or after t1. A failed read is skipped, so the
    // stale cached counts are never used. Both wheels come from this one
    // snapshot, so they share the same t1.
    if (encoders_fresh && now - et_start_ms >= ENCODER_TEST_T1_MS) {
      et_t1_ms = now;
      et_left_count1 = robot.getLeftEncoderCount();
      et_right_count1 = robot.getRightEncoderCount();
      et_phase = ET_WAIT_T2;
      if (Serial) {
        Serial.printf("ENC t1 = %lu ms, left count = %ld, right count = %ld\n",
                      et_t1_ms - et_start_ms, (long)et_left_count1, (long)et_right_count1);
      }
    }
    return;
  }

  if (et_phase == ET_WAIT_T2) {
    if (encoders_fresh && now - et_t1_ms >= ENCODER_TEST_WINDOW_MS) {
      et_t2_ms = now;
      et_left_count2 = robot.getLeftEncoderCount();
      et_right_count2 = robot.getRightEncoderCount();
      unsigned long elapsed_ms = et_t2_ms - et_t1_ms;

      if (Serial) {
        Serial.printf("ENC t2 = %lu ms, left count = %ld, right count = %ld\n",
                      et_t2_ms - et_start_ms, (long)et_left_count2, (long)et_right_count2);
        if (elapsed_ms == 0) {
          Serial.printf("ENC invalid: zero elapsed time, no speed calculated\n");
        } else {
          printWheelSpeed("left", et_left_count1, et_left_count2,
                          elapsed_ms, odometry.leftWheelRadiusMm);
          printWheelSpeed("right", et_right_count1, et_right_count2,
                          elapsed_ms, odometry.rightWheelRadiusMm);
        }
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
      if (Serial) {
        Serial.printf("ENC motor stopped. Press the button to return to the menu\n");
      }
    }
    return;
  }

  if (et_phase == ET_WAIT_BUTTON) {
    // The motor is already stopped. Telemetry stays paused (signal is still 1 and
    // the encoder test is still active), so the lines above stay on screen. This is
    // the only place the button is read while this routine runs, because
    // isButtonPressed() consumes the press when it returns true.
    if (robot.isButtonPressed()) {
      if (Serial) {
        Serial.printf("ENC button pressed, normal readings resume\n");
      }
      menu_timer.resetTimer(now);   // show the menu colour for a full interval
      setSignal(0);                 // soft reset: back to the menu
    }
  }
}

void Controller_c::printWheelSpeed(const char *label, int32_t count1, int32_t count2,
                                   unsigned long elapsed_ms, float radius_mm) {
  // Current minus previous, forward positive (OdometryModel.h). The
  // subtraction is done on unsigned values so a counter wrap is harmless.
  int32_t delta = (int32_t)((uint32_t)count2 - (uint32_t)count1);

  // counts/s = delta / (elapsed in seconds); milliseconds converted explicitly.
  float counts_per_s = (float)delta * 1000.0f / (float)elapsed_ms;

  // mm/s = revolutions/s * wheel circumference. The radius and counts per
  // revolution are the NOMINAL values in odometry, not calibrated ones.
  float mm_per_s = counts_per_s / odometry.encoderCountsPerRevolution
                   * TWO_PI * radius_mm;

  Serial.printf("ENC %s: delta = %ld counts over %lu ms -> %.1f counts/s = %.1f mm/s\n",
                label, (long)delta, elapsed_ms, counts_per_s, mm_per_s);
}

// ---------------------------------------------------------------------------
// Odometry test (move a commanded distance or angle, log CSV rows)
// The per-row logging happens in update(); this runs the move itself.
// ---------------------------------------------------------------------------

void Controller_c::odometryTest(Robot_c &robot, unsigned long now) {

  // 1. Start the move once, after a stationary period of logging.
  if (!move_attempted && now - press_ms >= still_ms) {
    move_attempted = true;                      // set first, so it can never retry
    bool started = rotation_test
                     ? robot.startRotateAngle(commanded_angle_rad)
                     : robot.startMoveDistance(commanded_distance_mm);
    if (started) {
      move_started = true;
      move_start_ms = now;
    } else {
      Serial.println("# start of move failed");
      robot.setMotorPWM(0, 0);
      move_finished = true;
      finish_ms = now;
    }
  }

  // 2. Poll the helper's status until it finishes, fails or times out.
  if (move_started && !move_finished) {
    bool stop_now = false;

    if (robot.getMotionStatus()) {
      if (robot.isMotionHelperRunning()) {
        move_running = true;
      } else if (move_running) {
        stop_now = true;                        // it ran and has now stopped
      }
    } else {
      Serial.println("# getMotionStatus failed");
      stop_now = true;                          // abort and diagnose
    }

    if (!stop_now && now - move_start_ms > move_timeout_ms) {
      Serial.println("# move timed out");
      stop_now = true;
    }

    if (stop_now) {
      robot.setMotorPWM(0, 0);                  // explicit stop; also cancels the helper
      move_finished = true;
      finish_ms = now;
    }
  }

  // 3. Keep logging stationary rows after the move, then end the trial.
  if (move_finished && now - finish_ms >= still_ms) {
    trial_num++;
    menu_timer.resetTimer(now);                 // show the menu colour for a full interval
    setSignal(0);                               // back to the menu, ready for the next press
  }
}

void Controller_c::reportOdometryTranslation(Robot_c &robot, unsigned long timestamp_ms) {
  if (Serial) {
    Serial.printf(
      "%s,%d,%.1f,%lu,%ld,%ld,%.2f,%.2f,%.5f\n",
      test_name,
      trial_num,
      commanded_distance_mm,
      timestamp_ms,
      (long)robot.getLeftEncoderCount(),
      (long)robot.getRightEncoderCount(),
      odometry.pose.x,
      odometry.pose.y,
      odometry.pose.theta
    );
  }
}

void Controller_c::reportOdometryRotation(Robot_c &robot, unsigned long timestamp_ms) {
  if (Serial) {
    Serial.printf(
      "%s,%d,%.5f,%lu,%ld,%ld,%.2f,%.2f,%.5f\n",
      test_name,
      trial_num,
      commanded_angle_rad,
      timestamp_ms,
      (long)robot.getLeftEncoderCount(),
      (long)robot.getRightEncoderCount(),
      odometry.pose.x,
      odometry.pose.y,
      odometry.pose.theta
    );
  }
}

// ---------------------------------------------------------------------------
// Line following
// ---------------------------------------------------------------------------

bool Controller_c::lineDetected(const Robot_c &robot) const {
  // Note: only using the central sensor to detect a line.
  if (robot.surface.reading[2] >= LINE_THRESHOLD) {
    return true;
  }

  return false;
}

void Controller_c::runLineFollower(Robot_c &robot, unsigned long now) {
  bool line_detected = lineDetected(robot);

  if (!line_detected && mode == FOLLOWING_LINE) {
    mode = SEARCHING_FOR_LINE;
    recovery_start_ms = now;
  }

  if (mode == SEARCHING_FOR_LINE) {
    if (line_detected) {
      mode = FOLLOWING_LINE;
    } else if (now - recovery_start_ms >= RECOVERY_TIMEOUT_MS) {
      mode = STOPPED;
    }
  }

  if (mode == STOPPED) {
    robot.setMotorPWM(0, 0);
    return;
  }

  if (mode == SEARCHING_FOR_LINE) {
    robot.setMotorPWM(RECOVERY_LEFT_PWM, RECOVERY_RIGHT_PWM);
    return;
  }

  float error = (float)robot.surface.reading[1] - (float)robot.surface.reading[3];
  float turn = error * LINE_FOLLOW_GAIN;
  robot.setMotorPWM(BASE_PWM - turn, BASE_PWM + turn);
}

// ---------------------------------------------------------------------------
// Exercise 1: sensorTest - raw 5-value CSV line for Serial Plotter
// ---------------------------------------------------------------------------

void Controller_c::sensorTest(Robot_c &robot, unsigned long now) {
  Serial.print( robot.surface.reading[0] );
  Serial.print( "," );
  Serial.print( robot.surface.reading[1] );
  Serial.print( "," );
  Serial.print( robot.surface.reading[2] );
  Serial.print( "," );
  Serial.print( robot.surface.reading[3] );
  Serial.print( "," );
  Serial.print( robot.surface.reading[4] );
  Serial.print( "\n" );

  robot.setMotorPWM(0, 0);

  // No delay: the 10 ms update timer governs the sampling rate.
}

// ---------------------------------------------------------------------------
// Exercise 2: testOffset - labelled per-sensor CSV rows
// ---------------------------------------------------------------------------

void Controller_c::testOffset(Robot_c &robot, unsigned long now) {
  // Only process genuinely new snapshots.
  if (robot.surface.timestamp_us <= last_surface_us) {
    return;
  }
  last_surface_us = robot.surface.timestamp_us;

  // Print one row per sensor with the agreed label scheme:
  // test_name,trial_num,sample_num,sample_time,sensor_num,sensor_reading
  for (uint8_t i = 0; i < 5; i++) {
    Serial.print("offsetTest_white");          Serial.print(",");
    Serial.print(trial_num);                   Serial.print(",");
    Serial.print(sample_num);                  Serial.print(",");
    Serial.print(robot.surface.timestamp_us);  Serial.print(",");
    Serial.print(i);                           Serial.print(",");
    Serial.println(robot.surface.reading[i]);
  }

  sample_num++;
  robot.setMotorPWM(0, 0);
}

// ---------------------------------------------------------------------------
// Telemetry
// ---------------------------------------------------------------------------

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