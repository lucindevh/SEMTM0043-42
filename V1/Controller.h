#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Robot.h"
#include "RobotIMU.h"
#include "RobotWifiAP.h"
#include "TaskTimer.h"
#include "OdometryModel.h"

/**
 * @brief Supplies the nominal line-following controller for the StampC3 example.
 *
 * Controller_c owns the controller state, while Robot_c owns the latest hardware
 * snapshot and RobotWifiAP_c owns optional telemetry transport. Students may read
 * or replace this controller, but should treat its constants as a demonstrator
 * baseline rather than a calibration of every physical robot.
 */
class Controller_c {
  
  public:
    /** @brief Editable local encoder-to-pose model; inspect odometry.pose.
     * @note Controller mode resets do not reset this estimate. Use
     * odometry.reset(...) explicitly to start a new physical trial.
     */
    OdometryModel_c odometry;

    /** @brief Fastest interval between controller and I2C acquisition cycles. */
    static const unsigned long UPDATE_INTERVAL_MS = 10;
    /** @brief Interval between telemetry records in milliseconds. */
    static const unsigned long TELEMETRY_INTERVAL_MS = 50;
    /** @brief Maximum line-search duration before the controller stops. */
    static const unsigned long RECOVERY_TIMEOUT_MS = 5000;
    /** @brief Surface-reading threshold used to decide whether a line is present. */
    static const uint16_t LINE_THRESHOLD = 1400;
    /** @brief Nominal forward PWM bias for line following. */
    static constexpr float BASE_PWM = 45.0f;
    /** @brief Proportional gain applied to the DN2-minus-DN4 surface error. */
    static constexpr float LINE_FOLLOW_GAIN = 0.01f;
    /** @brief Left PWM used during the one-direction line-search rotation. */
    static constexpr float RECOVERY_LEFT_PWM = -35.0f;
    /** @brief Right PWM used during the one-direction line-search rotation. */
    static constexpr float RECOVERY_RIGHT_PWM = 35.0f;

    /** @brief Time each routine's LED colour is shown in the menu before it advances. */
    static const unsigned long MENU_INTERVAL_MS = 3000;
    /** @brief Number of test-function calls with the ramp running (about 3 s at 10 ms). */
    static const unsigned int MOTOR_TEST_CALLS = 300;
    /** @brief Peak PWM magnitude of the motor-test ramp. Agree this with your team. */
    static constexpr float MOTOR_TEST_MAX_PWM = 150.0f;
    /** @brief Overall LED brightness used for the menu colours. */
    static const uint8_t MENU_LED_BRIGHTNESS = 20;

    /** @brief Encoder test: time after the test starts at which snapshot 1 is taken (t1). */
    static const unsigned long ENCODER_TEST_T1_MS = 1000;
    /** @brief Encoder test: minimum time after snapshot 1 before snapshot 2 (t2 = t1 + this). */
    static const unsigned long ENCODER_TEST_WINDOW_MS = 200;
    /** @brief Encoder test: safety limit; the motor is stopped if the test overruns this. */
    static const unsigned long ENCODER_TEST_TIMEOUT_MS = 5000;
    /** @brief Encoder test: constant PWM sent to the LEFT motor. Set to 0 to test the right wheel alone. */
    static constexpr float ENCODER_TEST_LEFT_PWM = 50.0f;
    /** @brief Encoder test: constant PWM sent to the RIGHT motor. Set to 0 to test the left wheel alone. */
    static constexpr float ENCODER_TEST_RIGHT_PWM = 50.0f;

    // ---- Sensor test state (from the sensor code) ----
    /** @brief Timestamp (us) of the last surface snapshot already logged by testOffset(). */
    unsigned long last_surface_us;
    /** @brief Sample counter for the sensor test. Cleared by reset(). */
    unsigned long sample_num;
    /** @brief Trial counter shared by the sensor and odometry tests. NOT cleared by reset(). */
    unsigned long trial_num;

    // ---- Odometry test settings (from the odometry code) ----
    // !! PLACEHOLDER VALUES: the odometry header was not available when this
    // !! was merged. Replace the values/types below with the ones from your
    // !! groupmate's Controller.h if they differ.
    /** @brief false = straight-line (translation) test, true = rotation test. */
    bool rotation_test = false;
    /** @brief Label written in the first CSV column of the odometry test. */
    const char *test_name = "stationary_test";
    /** @brief Commanded distance in mm for the translation test. */
    float commanded_distance_mm = 200.0f;
    /** @brief Commanded angle in radians for the rotation test. */
    float commanded_angle_rad = 1.5708f;
    /** @brief Stationary logging time before the move starts and after it ends. */
    unsigned long still_ms = 1000;
    /** @brief The move is aborted if it takes longer than this. */
    unsigned long move_timeout_ms = 15000;

    /** @brief Creates a controller in its waiting state. */
    Controller_c();

    /** @brief Clears signal and mode state, returning the controller to waiting.
     * @note This does not reset the task timers, the local odometry estimate
     * or trial_num. It sends no motor command: returning to waiting does not
     * stop the motors. Send robot.setMotorPWM(0, 0) separately and check the
     * command result.
     */
    void reset();

    /**
     * @brief Returns whether the controller has received its start signal.
     * @return Zero while waiting; one after the button or caller starts it.
     */
    uint8_t getSignal() const;

    /**
     * @brief Starts the controller or resets it to waiting.
     * @param requested_signal Zero resets to waiting without sending a motor stop;
     * a non-zero value starts line following only if the current signal is zero.
     * @note The signal remains one after a line-search timeout. Reset to zero
     * before requesting another start; a repeated non-zero value has no effect.
     */
    void setSignal(uint8_t requested_signal);

    /**
     * @brief Runs one time-controlled acquisition, control, and telemetry cycle.
     * @param robot Hardware-facing interface and cached measurement snapshot.
     * @param server Optional WiFi/TCP telemetry transport.
     * @note Each accepted update cycle attempts to refresh sensors, encoders,
     * and the middleware pose. Failed reads retain their previous cached values;
     * local odometry is updated only after a successful encoder read. The method
     * returns early between update intervals.
     */
    void update(Robot_c &robot, RobotWifiAP_c &server, RobotIMU_c &imu);

  private:
    /** @brief Internal modes used by the nominal line-following demonstration. */
    enum Mode {
      WAITING,
      FOLLOWING_LINE,
      SEARCHING_FOR_LINE,
      STOPPED
    };

    /** @brief Routines selectable from the LED menu. */
    enum Routine {
      ROUTINE_LINE_FOLLOW,     // LED white (placeholder for surface classification later)
      ROUTINE_MOTOR_TEST,      // LED green (no longer in the menu)
      ROUTINE_ENCODER_TEST,    // LED red in the old menu (no longer in the menu)
      ROUTINE_ODOMETRY_TEST,   // LED blue (no longer in the menu)
      ROUTINE_SENSOR_TEST,     // LED yellow (no longer in the menu)
      ROUTINE_COMBINED_TEST    // LED red: combined encoder/sensor/odometry/IMU test
    };

    void reportCombined(Robot_c &robot, const RobotIMUSample_t &s, float yaw_rate, unsigned long timestamp_ms);

    const char* condition = "stationary";   // edit per record: stationary, cw_turn, ccw_turn, surface name...
    int yaw_axis = 2;                      // z axis
    float yaw_sign = 1.0f;

    static const unsigned int BIAS_SAMPLES = 100;  // about 1.5 s at ~15 ms per sample
    float gyro_bias_dps[3] = {0, 0, 0};
    float gyro_bias_sum[3] = {0, 0, 0};
    unsigned int bias_count = 0;
    bool bias_ready = false;

    /** @brief Steps of the non-blocking encoder test. */
    enum EncoderTestPhase {
      ET_START,      // send the motor command, then start timing
      ET_WAIT_T1,    // wait for the first fresh snapshot at or after t1
      ET_WAIT_T2,    // wait for the first fresh snapshot at or after t1 + window
      ET_STOP,       // stop the motor
      ET_WAIT_BUTTON // motor stopped; readings stay paused until the button is pressed, then return to the menu
    };

    uint8_t signal;
    Mode mode;
    TaskTimer_c update_timer;
    TaskTimer_c telemetry_timer;
    unsigned long recovery_start_ms;

    // Menu state. selected_routine and menu_timer are deliberately NOT cleared
    // by reset(), so a reset returns to the menu on the routine last shown.
    Routine selected_routine;
    TaskTimer_c menu_timer;
    // False until the first menu update, so the first colour (white) is shown for
    // a full interval even if the StampC3 has already been running for a while.
    bool menu_started;
    // Routine started by the last button press. reset() returns it to line following.
    Routine active_routine;
    // True when the LED must be redrawn to match selected_routine.
    bool led_dirty;
    // Calls made to motorTest() since it started. Cleared by reset().
    unsigned int call_count;

    // True when this cycle's getEncoders() call succeeded, so the cached counts
    // are a new snapshot. A failed read leaves the old counts in the cache.
    bool encoders_fresh;

    // Encoder test state. All cleared by reset().
    EncoderTestPhase et_phase;
    unsigned long et_start_ms;   // time the motor command was accepted
    unsigned long et_t1_ms;      // actual time of snapshot 1
    unsigned long et_t2_ms;      // actual time of snapshot 2
    // Both wheels are read from the SAME snapshot at each time, so they share t1 and t2.
    int32_t et_left_count1;      // left encoder count at snapshot 1
    int32_t et_left_count2;      // left encoder count at snapshot 2
    int32_t et_right_count1;     // right encoder count at snapshot 1
    int32_t et_right_count2;     // right encoder count at snapshot 2

    // Odometry test run state. Set up when the test is started from the menu.
    // (Also placeholders in the sense that names come from the odometry code.)
    unsigned long press_ms = 0;        // time the button started the test
    unsigned long move_start_ms = 0;   // time the move was accepted
    unsigned long finish_ms = 0;       // time the move ended
    bool move_attempted = false;       // start of the move has been tried once
    bool move_started = false;         // the move helper accepted the command
    bool move_running = false;         // the helper has been seen running
    bool move_finished = false;        // the move has ended (finished, failed or timed out)

    /** @brief Advances the menu every MENU_INTERVAL_MS and keeps the LED in step. */
    void updateMenu(Robot_c &robot, unsigned long now);

    /** @brief Ramps the wheels in opposite directions, then stops and returns to the menu. */
    void motorTest(Robot_c &robot, unsigned long now);

    /**
     * @brief Measures both wheels' speeds from two encoder snapshots.
     * @note Powers both motors, takes snapshot 1 at about t1 and snapshot 2
     * at least ENCODER_TEST_WINDOW_MS later, prints counts and speed over Serial,
     * then stops the motor and keeps the normal readings paused until the button is
     * pressed again, so the lines can be read and noted; that press sets signal to 0
     * and returns to the menu. The times used in the calculation
     * are the actual cycle times of the two snapshots, not the nominal ones.
     */
    void encoderTest(Robot_c &robot, unsigned long now);

    /**
     * @brief Prints one wheel's count change and speed over Serial.
     * @param label Wheel name shown in the output, e.g. "left".
     * @param count1 Cumulative count at snapshot 1.
     * @param count2 Cumulative count at snapshot 2.
     * @param elapsed_ms Actual time between the two snapshots; must be > 0.
     * @param radius_mm Nominal wheel radius used only for the mm/s conversion.
     */
    void printWheelSpeed(const char *label, int32_t count1, int32_t count2,
                         unsigned long elapsed_ms, float radius_mm);

    /**
     * @brief Runs one commanded move (distance or angle) with stationary logging
     * before and after, then ends the trial and returns to the menu.
     * @note CSV rows are printed from update() on every fresh encoder read.
     */
    void odometryTest(Robot_c &robot, unsigned long now);

    /** @brief Prints one CSV row for the translation test over Serial. */
    void reportOdometryTranslation(Robot_c &robot, unsigned long timestamp_ms);

    /** @brief Prints one CSV row for the rotation test over Serial. */
    void reportOdometryRotation(Robot_c &robot, unsigned long timestamp_ms);

    /** @brief Exercise 1: prints the five raw surface readings as one CSV line. */
    void sensorTest(Robot_c &robot, unsigned long now);

    /** @brief Exercise 2: prints one labelled CSV row per sensor for each new surface snapshot. */
    void testOffset(Robot_c &robot, unsigned long now);

    /** @brief Returns whether cached DN3 is at or above LINE_THRESHOLD. */
    bool lineDetected(const Robot_c &robot) const;

    /** @brief Applies the nominal controller behaviour for the current mode. */
    void runLineFollower(Robot_c &robot, unsigned long now);

    /**
     * @brief Sends the current controller and robot snapshot as one CSV record.
     * @note Controller.cpp writes these 14 columns, in order: timestamp_ms,
     * x, y, theta, left_pwm, right_pwm, left_count, right_count,
     * DN1, DN2, DN3, DN4, DN5, signal. These labels are not sent as a header.
     * x and y are in mm and theta is in radians, from the local Euler estimate,
     * not robot.pose from I2C. PWM values are cached requests, not measured output.
     * The timestamp is the controller-cycle start time; it does not guarantee
     * that every cached reading was successfully refreshed in that cycle.
     */
    void publishTelemetry(Robot_c &robot, RobotWifiAP_c &server, unsigned long timestamp_ms);
};

#endif