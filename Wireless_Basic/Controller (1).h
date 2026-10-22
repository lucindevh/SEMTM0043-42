#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Robot.h"
#include "RobotWifiAP.h"
#include "TaskTimer.h"
#include "OdometryModel.h"

/**
 * @brief Supplies the encoder-test controller for the StampC3 example.
 *
 * Controller_c owns the controller state, while Robot_c owns the latest hardware
 * snapshot and RobotWifiAP_c owns optional telemetry transport.
 *
 * While waiting, all sensor readings are printed as normal telemetry. A button
 * press starts the encoder test: telemetry pauses, both wheels are driven, two
 * encoder snapshots are taken and the wheel speeds are printed over Serial.
 * A second button press returns to waiting and the normal readings resume.
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
    /** @brief Overall brightness of the red LED shown by the encoder test. */
    static const uint8_t LED_BRIGHTNESS = 20;

    /** @brief Encoder test: time after the test starts at which snapshot 1 is taken (t1). */
    static const unsigned long ENCODER_TEST_T1_MS = 1000;
    /** @brief Encoder test: minimum time after snapshot 1 before snapshot 2 (t2 = t1 + this). */
    static const unsigned long ENCODER_TEST_WINDOW_MS = 200;
    /** @brief Encoder test: safety limit; the motor is stopped if the test overruns this. */
    static const unsigned long ENCODER_TEST_TIMEOUT_MS = 5000;
    /** @brief Encoder test: constant PWM sent to the LEFT motor. Set to 0 to test the right wheel alone. */
    static constexpr float ENCODER_TEST_LEFT_PWM = 100.0f;
    /** @brief Encoder test: constant PWM sent to the RIGHT motor. Set to 0 to test the left wheel alone. */
    static constexpr float ENCODER_TEST_RIGHT_PWM = 100.0f;

    /** @brief Creates a controller in its waiting state. */
    Controller_c();

    /** @brief Clears signal and encoder-test state, returning the controller to waiting.
     * @note This does not reset the task timers or the local odometry estimate.
     * It sends no motor command: returning to waiting does not stop the motors.
     * Send robot.setMotorPWM(0, 0) separately and check the command result.
     */
    void reset();

    /**
     * @brief Returns whether the encoder test has been started.
     * @return Zero while waiting; one while the encoder test is active.
     */
    uint8_t getSignal() const;

    /**
     * @brief Starts the encoder test or resets to waiting.
     * @param requested_signal Zero resets to waiting without sending a motor stop;
     * a non-zero value starts the encoder test only if the current signal is zero.
     */
    void setSignal(uint8_t requested_signal);

    /**
     * @brief Runs one time-controlled acquisition, test, and telemetry cycle.
     * @param robot Hardware-facing interface and cached measurement snapshot.
     * @param server Optional WiFi/TCP telemetry transport.
     * @note Each accepted update cycle attempts to refresh sensors, encoders,
     * and the middleware pose. Failed reads retain their previous cached values;
     * local odometry is updated only after a successful encoder read. The method
     * returns early between update intervals.
     */
    void update(Robot_c &robot, RobotWifiAP_c &server);

  private:
    /** @brief Steps of the non-blocking encoder test. */
    enum EncoderTestPhase {
      ET_START,      // send the motor command, then start timing
      ET_WAIT_T1,    // wait for the first fresh snapshot at or after t1
      ET_WAIT_T2,    // wait for the first fresh snapshot at or after t1 + window
      ET_STOP,       // stop the motor
      ET_WAIT_BUTTON // motor stopped; readings stay paused until the button is pressed
    };

    uint8_t signal;
    TaskTimer_c update_timer;
    TaskTimer_c telemetry_timer;

    // False until the LED has been set to red, so it is only written once.
    bool led_set;

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

    /**
     * @brief Measures both wheels' speeds from two encoder snapshots.
     * @note Powers both motors, takes snapshot 1 at about t1 and snapshot 2
     * at least ENCODER_TEST_WINDOW_MS later, prints counts and speed over Serial,
     * then stops the motor and keeps the normal readings paused until the button is
     * pressed again, so the lines can be read and noted; that press sets signal to 0
     * and the normal readings resume. The times used in the calculation
     * are the actual cycle times of the two snapshots, not the nominal ones.
     */
    void encoderTest(Robot_c &robot, RobotWifiAP_c &server, unsigned long now);

    /**
     * @brief Sends one text message to the WiFi client and to Serial, if connected.
     * @note The formatted message must be under 160 bytes and should end in a newline.
     */
    void report(RobotWifiAP_c &server, const char *format, ...);

    /**
     * @brief Prints one wheel's count change and speed over Serial.
     * @param label Wheel name shown in the output, e.g. "left".
     * @param count1 Cumulative count at snapshot 1.
     * @param count2 Cumulative count at snapshot 2.
     * @param elapsed_ms Actual time between the two snapshots; must be > 0.
     * @param radius_mm Nominal wheel radius used only for the mm/s conversion.
     */
    void printWheelSpeed(RobotWifiAP_c &server, const char *label, int32_t count1,
                         int32_t count2, unsigned long elapsed_ms, float radius_mm);

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