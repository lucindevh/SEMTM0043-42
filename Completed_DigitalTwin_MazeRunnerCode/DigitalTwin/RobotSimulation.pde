/**
 * @brief Top-level model of the robot, excluding the environment.
 * @details Coordinates independent motor, surface-sensor, and odometry
 * models. Its public methods mirror the subset of the StampC3 Robot_c
 * (Robot.h) used by Controller_c: sensors, encoders, PWM, LED, button and
 * the middleware motion helpers. Black pixels on the surface image are solid
 * walls that the robot body cannot pass through.
 */
class Robot_c {
  /** @brief Radius used only to draw the robot body, in mm. */
  final float bodyRadiusMm = 50.0;
  /** @brief Radius used for wall collisions, in mm. A 3pi+ is about 97 mm across. */
  final float collisionRadiusMm = 48.5;
  /** @brief Pixels with brightness at or below this are solid walls. */
  final float WALL_BRIGHTNESS_MAX = 64;
  final float wheelRadiusMm;
  final float wheelSeparationMm;
  final float encoderCountsPerRevolution = 358.3;

  /** @brief Simulated clock, advanced explicitly in milliseconds. */
  long simulationMillis = 0;
  PImage surfaceImage;
  final float surfaceOffsetX;
  final float surfaceOffsetY;
  RobotSurface_c surface = new RobotSurface_c();
  MotorsModel_c motors = new MotorsModel_c();
  OdometryModel_c odometry;
  SurfaceSensorModel_c[] sensors = new SurfaceSensorModel_c[5];

  // ---- Walls ----
  boolean[] wallMask;
  int[] discDx;
  int[] discDy;
  /** @brief True when the last physics step was stopped by a wall. */
  boolean blockedLastStep = false;

  // ---- StampC3 extras: LED and button ----
  int ledR = 255, ledG = 128, ledB = 178;   // Robot_c::setupLED() start colour
  /** @brief Set by the simulator's B key; consumed by isButtonPressed(). */
  boolean buttonPressPending = false;

  // ---- Middleware motion helper (startRotateAngle / startMoveDistance) ----
  // The real helper uses a fixed internal PWM and a safety timeout that are
  // not published. These are assumptions: compare the turn time with your
  // real robot and adjust.
  final float MOTION_HELPER_PWM = 40;
  final int MOTION_HELPER_TIMEOUT_MS = 10000;
  final int MOTION_STATUS_IDLE = 0, MOTION_STATUS_RUNNING = 1,
            MOTION_STATUS_COMPLETE = 2, MOTION_STATUS_TIMEOUT = 3,
            MOTION_STATUS_CANCELLED = 4;
  final int MOTION_MODE_NONE = 0, MOTION_MODE_DISTANCE = 1, MOTION_MODE_ROTATION = 2;
  // Geometry the helper uses to work out encoder targets (Robot.h defaults
  // are not published; nominal 16 mm radius and 90 mm separation assumed).
  float helperLeftRadiusMm = 16.0;
  float helperRightRadiusMm = 16.0;
  float helperSeparationMm = 90.0;
  int motionState = MOTION_STATUS_IDLE;
  int motionMode = MOTION_MODE_NONE;
  long motionStartMs = 0;
  float helperStartLeft = 0, helperStartRight = 0;
  float helperCountsLeft = 0, helperCountsRight = 0;

  /**
   * @param wheelRadiusMmIn Wheel radius used by the odometry model, in mm.
   * @param wheelSeparationMmIn Distance between wheels, in mm.
   * @param surfaceIn Environment image sampled by the five surface sensors.
   * @param surfaceOffsetXIn Surface-image x offset in world millimetres.
   * @param surfaceOffsetYIn Surface-image y offset in world millimetres.
   */
  Robot_c(float wheelRadiusMmIn, float wheelSeparationMmIn,
    PImage surfaceIn, float surfaceOffsetXIn, float surfaceOffsetYIn) {
    wheelRadiusMm = wheelRadiusMmIn;
    wheelSeparationMm = wheelSeparationMmIn;
    surfaceImage = surfaceIn;
    surfaceOffsetX = surfaceOffsetXIn;
    surfaceOffsetY = surfaceOffsetYIn;
    odometry = new OdometryModel_c(wheelRadiusMm, encoderCountsPerRevolution);
    sensors[0] = new SurfaceSensorModel_c("DN1", 45, 45);
    sensors[1] = new SurfaceSensorModel_c("DN2", 45, 14);
    sensors[2] = new SurfaceSensorModel_c("DN3", 45, 0);
    sensors[3] = new SurfaceSensorModel_c("DN4", 45, -14);
    sensors[4] = new SurfaceSensorModel_c("DN5", 45, -45);
    odometry.reset(0.0,0.0, -PI/2.0, 0.0, 0.0);
    buildWallModel();
  }

  /** @brief Marks dark pixels as walls and lists the pixels under the body. */
  void buildWallModel() {
    surfaceImage.loadPixels();
    wallMask = new boolean[surfaceImage.width * surfaceImage.height];
    for (int i = 0; i < wallMask.length; i++) {
      wallMask[i] = brightness(surfaceImage.pixels[i]) <= WALL_BRIGHTNESS_MAX;
    }

    int r = ceil(collisionRadiusMm);
    IntList dxList = new IntList();
    IntList dyList = new IntList();
    for (int dy = -r; dy <= r; dy++) {
      for (int dx = -r; dx <= r; dx++) {
        if (dx * dx + dy * dy <= collisionRadiusMm * collisionRadiusMm) {
          dxList.append(dx);
          dyList.append(dy);
        }
      }
    }
    discDx = dxList.array();
    discDy = dyList.array();
  }

  /** @return Number of wall pixels under the body if its centre were at (x, y). */
  int wallPixelsUnderBody(float x, float y) {
    int centreX = floor(surfaceImage.width / 2.0 + x - surfaceOffsetX);
    int centreY = floor(surfaceImage.height / 2.0 - y + surfaceOffsetY);
    int count = 0;
    for (int i = 0; i < discDx.length; i++) {
      int px = centreX + discDx[i];
      int py = centreY + discDy[i];
      if (px < 0 || px >= surfaceImage.width || py < 0 || py >= surfaceImage.height) continue;
      if (wallMask[py * surfaceImage.width + px]) count++;
    }
    return count;
  }

  /** @return Current simulated time in milliseconds. */
  long getMillis() { return simulationMillis; }
  /** @brief Advances the simulated clock.
   * @param dtMs Positive elapsed model time in milliseconds.
   */
  void advanceTime(int dtMs) { simulationMillis += dtMs; }

  /** @brief Samples all five ideal surface sensors into surface.reading. */
  boolean getSurfaceSensors() {
    RobotPose_c pose = odometry.pose;
    for (int i = 0; i < sensors.length; i++) {
      surface.reading[i] = sensors[i].sample(surfaceImage, surfaceOffsetX,
        surfaceOffsetY, pose.x, pose.y, pose.theta);
    }
    return true;
  }

  /** @brief Compatibility method; encoder values update during physics steps. */
  boolean getEncoders() { return true; }
  /** @brief Compatibility method; pose values update during physics steps. */
  boolean getPose() { return true; }

  /** @return Whole-count left encoder value, like Robot_c::getLeftEncoderCount(). */
  int getLeftEncoderCount() { return floor(odometry.encoders.left); }
  /** @return Whole-count right encoder value, like Robot_c::getRightEncoderCount(). */
  int getRightEncoderCount() { return floor(odometry.encoders.right); }

  /** @brief Stores requested left and right motor PWM values.
   * @note Like the real Robot_c, values are truncated to whole numbers and a
   * new motor command cancels a running motion helper.
   */
  boolean setMotorPWM(float leftPwm, float rightPwm) {
    if (motionState == MOTION_STATUS_RUNNING) motionState = MOTION_STATUS_CANCELLED;
    motors.setPWM((int) leftPwm, (int) rightPwm);
    return true;
  }

  /** @brief Stores the LED colour; it is drawn on the robot. */
  void setLED(int r, int g, int b, int brightness) {
    ledR = constrain(r, 0, 255);
    ledG = constrain(g, 0, 255);
    ledB = constrain(b, 0, 255);
  }

  /** @return True once per press of the simulator's B key. */
  boolean isButtonPressed() {
    if (buttonPressPending) {
      buttonPressPending = false;
      return true;
    }
    return false;
  }

  // ---- Motion helpers ----

  /** @brief Nominal in-place rotation; positive angles turn left. */
  boolean startRotateAngle(float angleRad) {
    if (Float.isNaN(angleRad) || Float.isInfinite(angleRad) || abs(angleRad) > TWO_PI) {
      return false;
    }
    float leftTravelMm = -angleRad * helperSeparationMm / 2.0;
    float rightTravelMm = angleRad * helperSeparationMm / 2.0;
    startHelper(MOTION_MODE_ROTATION,
      leftTravelMm / (TWO_PI * helperLeftRadiusMm) * encoderCountsPerRevolution,
      rightTravelMm / (TWO_PI * helperRightRadiusMm) * encoderCountsPerRevolution);
    return true;
  }

  /** @brief Nominal straight-line move; negative distances move backwards. */
  boolean startMoveDistance(float distanceMm) {
    if (Float.isNaN(distanceMm) || Float.isInfinite(distanceMm) || abs(distanceMm) > 2000) {
      return false;
    }
    startHelper(MOTION_MODE_DISTANCE,
      distanceMm / (TWO_PI * helperLeftRadiusMm) * encoderCountsPerRevolution,
      distanceMm / (TWO_PI * helperRightRadiusMm) * encoderCountsPerRevolution);
    return true;
  }

  boolean setMotionHelperGeometry(float leftRadiusMm, float rightRadiusMm, float separationMm) {
    if (motionState == MOTION_STATUS_RUNNING) return true;   // write ignored while running
    if (leftRadiusMm >= 5 && leftRadiusMm <= 50 && rightRadiusMm >= 5 && rightRadiusMm <= 50
        && separationMm >= 20 && separationMm <= 200) {
      helperLeftRadiusMm = leftRadiusMm;
      helperRightRadiusMm = rightRadiusMm;
      helperSeparationMm = separationMm;
    }
    return true;
  }

  /** @brief Always succeeds in simulation (no I2C). */
  boolean getMotionStatus() { return true; }

  boolean isMotionHelperRunning() { return motionState == MOTION_STATUS_RUNNING; }

  void startHelper(int mode, float countsLeft, float countsRight) {
    motionMode = mode;
    motionState = MOTION_STATUS_RUNNING;
    motionStartMs = simulationMillis;
    helperStartLeft = odometry.encoders.left;
    helperStartRight = odometry.encoders.right;
    helperCountsLeft = countsLeft;
    helperCountsRight = countsRight;
  }

  /** @brief Drives each wheel at the helper PWM until its encoder target is reached. */
  void updateMotionHelper() {
    if (motionState != MOTION_STATUS_RUNNING) return;

    float doneLeft = odometry.encoders.left - helperStartLeft;
    float doneRight = odometry.encoders.right - helperStartRight;
    boolean leftDone = abs(doneLeft) >= abs(helperCountsLeft);
    boolean rightDone = abs(doneRight) >= abs(helperCountsRight);

    if (leftDone && rightDone) {
      motionState = MOTION_STATUS_COMPLETE;
      motors.setPWM(0, 0);
      return;
    }
    if (simulationMillis - motionStartMs > MOTION_HELPER_TIMEOUT_MS) {
      motionState = MOTION_STATUS_TIMEOUT;
      motors.setPWM(0, 0);
      return;
    }
    float leftPwm = leftDone ? 0 : MOTION_HELPER_PWM * Math.signum(helperCountsLeft);
    float rightPwm = rightDone ? 0 : MOTION_HELPER_PWM * Math.signum(helperCountsRight);
    motors.setPWM(leftPwm, rightPwm);
  }

  /** @brief Resets pose and encoder state for a controlled run.
   * @param x Initial x position in mm.
   * @param y Initial y position in mm.
   * @param theta Initial heading in radians.
   * @param leftEncoder Initial left encoder count.
   * @param rightEncoder Initial right encoder count.
   */
  void resetForRun(float x, float y, float theta, float leftEncoder,
    float rightEncoder) {
    odometry.reset(x, y, theta, leftEncoder, rightEncoder);
    motionState = MOTION_STATUS_IDLE;
    motionMode = MOTION_MODE_NONE;
    setMotorPWM(0, 0);
  }

  /** @brief Advances motor response, wall contact and odometry.
   * @param dtMs Fixed simulation step in milliseconds.
   */
  void updatePhysics(int dtMs) {
    updateMotionHelper();
    motors.update();
    float leftDistance = motors.left.speedMmPerMs * dtMs;
    float rightDistance = motors.right.speedMmPerMs * dtMs;

    // Would the forward/backward part of this step push the body into a wall?
    RobotPose_c pose = odometry.pose;
    float distance = (leftDistance + rightDistance) / 2.0;
    blockedLastStep = false;
    if (distance != 0) {
      float newX = pose.x + distance * cos(pose.theta);
      float newY = pose.y + distance * sin(pose.theta);
      int wallsAfter = wallPixelsUnderBody(newX, newY);
      if (wallsAfter > 0 && wallsAfter >= wallPixelsUnderBody(pose.x, pose.y)) {
        // Blocked: the wheels stall, so neither the body nor the encoders
        // move forward. A round robot can still turn on the spot, so the
        // turning part of the wheel motion is kept.
        float turnPart = (rightDistance - leftDistance) / 2.0;
        leftDistance = -turnPart;
        rightDistance = turnPart;
        blockedLastStep = true;
      }
    }
    odometry.update(leftDistance, rightDistance, wheelSeparationMm);
  }

  /** @return The current five-channel ideal surface-sensor readings. */
  float[] sensorReadings() { return surface.reading; }

  void draw() {
    RobotPose_c pose = odometry.pose;
    pushMatrix();
    translate(pose.x, pose.y);
    rotate(pose.theta);
    fill(190, 205, 220);
    stroke(blockedLastStep ? color(220, 40, 40) : color(40));
    strokeWeight(2);
    ellipse(0, 0, bodyRadiusMm, bodyRadiusMm);
    stroke(40, 100, 220);
    strokeWeight(3);
    line(0, 0, bodyRadiusMm * .72, 0);
    fill(40, 100, 220);
    noStroke();
    triangle(bodyRadiusMm * .92, 0, bodyRadiusMm * .62, bodyRadiusMm * .14,
      bodyRadiusMm * .62, -bodyRadiusMm * .14);
    // StampC3 LED, drawn behind the centre.
    fill(ledR, ledG, ledB);
    stroke(40);
    strokeWeight(1);
    ellipse(-bodyRadiusMm * 0.45, 0, 7, 7);
    for (int i = 0; i < sensors.length; i++) sensors[i].draw(surface.reading[i]);
    popMatrix();
  }
}
