/**
 * @brief Simulation port of the StampC3 surface-classification maze routine.
 * @details Mirrors Controller_c::surfaceClassify() in Controller.cpp (Team_777)
 * line by line, with the same constants as Controller.h:
 *   - stop and measure the centre sensor (DN3) over REST_SAMPLES readings,
 *   - classify: < 495 tinfoil, < 811 carpet, < 970 cardboard, else table,
 *   - show the surface colour on the LED,
 *   - first table starts the maze, second table finishes it,
 *   - tinfoil: rotate right 90 deg, carpet: rotate left 90 deg,
 *     cardboard: carry straight on,
 *   - a surface change needs SURFACE_CHANGE_SAMPLES readings in a row, then
 *     the robot stops straight away and measures the new surface,
 *   - PWM = BASE_PWM * surface scale + stall "kick" from the encoders.
 * Only the surface-classification routine is ported. The menu's combined
 * test needs the IMU, which the simulation does not model.
 * Press B in the simulator to act as the StampC3 button.
 */
class Controller_c {
  // ---- From Controller.h ----
  final int UPDATE_INTERVAL_MS = 10;
  final int TELEMETRY_INTERVAL_MS = 50;
  final float BASE_PWM = 45.0;
  final int MENU_LED_BRIGHTNESS = 20;
  final int move_timeout_ms = 15000;

  final int SURFACE_CHANGE_SAMPLES = 5;
  final int REST_SAMPLES = 50;              // ~0.5 s with motors off
  final float IR_TINFOIL_MAX   = 495.0;     // below = tinfoil
  final float IR_CARPET_MAX    = 811.0;     // below = carpet
  final float IR_CARDBOARD_MAX = 970.0;     // below = cardboard, else table
  final int STALL_WINDOW_MS = 250;
  final int STALL_TICKS = 50;
  final float KICK_STEP = 5.0;
  final float MAX_KICK  = 30.0;

  final int SURF_UNKNOWN = 0, SURF_TABLE = 1, SURF_CARDBOARD = 2,
            SURF_CARPET = 3, SURF_TINFOIL = 4;
  final String[] SURFACE_NAMES = {"unknown", "table", "cardboard", "carpet", "tinfoil"};

  int signal = 0;
  boolean led_dirty = true;

  // Surface classification
  int surface = SURF_UNKNOWN;
  float rest_ir_sum = 0;
  int rest_ir_count = 0;
  float pwm_kick = 0;
  long stall_window_ms = 0;
  int stall_left_start = 0, stall_right_start = 0;

  // Maze state
  boolean maze_started = false;
  boolean maze_finished = false;
  boolean surface_action_done = false;

  // Turn state
  boolean maze_turn_started = false;
  boolean maze_turn_seen_running = false;
  long maze_turn_start_ms = 0;

  int candidate_surface = SURF_UNKNOWN;
  int candidate_surface_count = 0;

  TaskTimer_c update_timer = new TaskTimer_c( UPDATE_INTERVAL_MS );
  TaskTimer_c telemetry_timer = new TaskTimer_c( TELEMETRY_INTERVAL_MS );

  /** @brief Same state reset as Controller_c::reset() for this routine. */
  void reset() {
    signal = 0;
    led_dirty = true;

    surface = SURF_UNKNOWN;
    rest_ir_sum = 0;
    rest_ir_count = 0;
    pwm_kick = 0;
    stall_window_ms = 0;

    maze_started = false;
    maze_finished = false;
    surface_action_done = false;

    maze_turn_started = false;
    maze_turn_seen_running = false;
    maze_turn_start_ms = 0;

    candidate_surface = SURF_UNKNOWN;
    candidate_surface_count = 0;
  }

  int getSignal() {
    return signal;
  }

  void setSignal(int requested) {
    if (requested == 0) {
      reset();
      return;
    }
    if (signal == 0) {
      signal = 1;
    }
  }

  void update(Robot_c robot) {
    long now = robot.getMillis();

    if (!update_timer.isReady(now)) {
      return;
    }
    update_timer.resetTimer(now);

    robot.getSurfaceSensors();
    boolean encoders_fresh = robot.getEncoders();   // always succeeds in simulation
    robot.getPose();

    if (signal == 0) {
      // The real menu alternates white/red; white is the surface routine.
      if (led_dirty) {
        led_dirty = false;
        robot.setLED(255, 255, 255, MENU_LED_BRIGHTNESS);
      }
      if (robot.isButtonPressed()) {
        setSignal(1);
      }
    }

    if (signal == 1) {
      if (encoders_fresh) {
        surfaceClassify(robot, now);
      }
      if (robot.isButtonPressed()) {
        robot.setMotorPWM(0, 0);    // the motors may be running now, so stop them
        setSignal(0);
      }
    }

    if (telemetry_timer.isReady(now)) {
      telemetry_timer.resetTimer(now);
      publishTelemetry(robot);
    }
  }

  void surfaceClassify(Robot_c robot, long now) {

    // 1) If the maze has finished, keep the motors stopped.
    if (maze_finished) {
      robot.setMotorPWM(0, 0);
      return;
    }

    // Use only the centre sensor (DN3), so a reading is never a mix of two
    // surfaces when the robot is straddling an edge.
    float ir = robot.surface.reading[2];

    // 2) MEASURE: classify a new surface while stationary.
    if (surface == SURF_UNKNOWN) {
      robot.setMotorPWM(0, 0);

      rest_ir_sum += ir;
      rest_ir_count++;
      if (rest_ir_count < REST_SAMPLES) {
        return;
      }

      float level = rest_ir_sum / rest_ir_count;

      if (level < IR_TINFOIL_MAX) {
        surface = SURF_TINFOIL;
      } else if (level < IR_CARPET_MAX) {
        surface = SURF_CARPET;
      } else if (level < IR_CARDBOARD_MAX) {
        surface = SURF_CARDBOARD;
      } else {
        surface = SURF_TABLE;
      }

      if (surface == SURF_TABLE) {
        robot.setLED(0, 0, 255, MENU_LED_BRIGHTNESS);
      } else if (surface == SURF_CARDBOARD) {
        robot.setLED(255, 140, 0, MENU_LED_BRIGHTNESS);
      } else if (surface == SURF_CARPET) {
        robot.setLED(255, 192, 203, MENU_LED_BRIGHTNESS);
      } else {
        robot.setLED(0, 255, 0, MENU_LED_BRIGHTNESS);
      }

      report("# surface = " + surface + " (" + SURFACE_NAMES[surface]
        + "), mean IR = " + round(level));

      surface_action_done = false;
      candidate_surface_count = 0;

      stall_window_ms = now;
      stall_left_start = robot.getLeftEncoderCount();
      stall_right_start = robot.getRightEncoderCount();
      return;
    }

    // 3) HANDLE AN ACTIVE TURN.
    if (maze_turn_started) {
      boolean stop_now = false;

      if (robot.getMotionStatus()) {
        if (robot.isMotionHelperRunning()) {
          maze_turn_seen_running = true;
        } else if (maze_turn_seen_running) {
          stop_now = true;
        }
      } else {
        report("# turn status failed");
        stop_now = true;
        maze_finished = true;
      }

      if (!stop_now && now - maze_turn_start_ms > move_timeout_ms) {
        report("# turn timed out");
        stop_now = true;
        maze_finished = true;
      }

      if (stop_now) {
        robot.setMotorPWM(0, 0);
        maze_turn_started = false;
        maze_turn_seen_running = false;
      }
      return;
    }

    // 4) CALCULATE FORWARD PWM.
    float scale = 1.0;
    if (surface == SURF_CARPET) {
      scale = 1.4;
    } else if (surface == SURF_TINFOIL) {
      scale = 0.95;
    }

    // Stuck detection and PWM adjustment.
    if (now - stall_window_ms >= STALL_WINDOW_MS) {
      int left_count = robot.getLeftEncoderCount();
      int right_count = robot.getRightEncoderCount();
      int ticks = abs(left_count - stall_left_start)
                + abs(right_count - stall_right_start);

      if (ticks < STALL_TICKS) {
        pwm_kick = min(pwm_kick + KICK_STEP, MAX_KICK);
      } else {
        pwm_kick = max(pwm_kick - KICK_STEP, 0.0);
      }

      stall_window_ms = now;
      stall_left_start = left_count;
      stall_right_start = right_count;
    }

    float pwm = BASE_PWM * scale + pwm_kick;

    // 5) PER-SURFACE ACTIONS (each surface triggers its action once).
    if (!surface_action_done) {
      if (surface == SURF_TABLE) {
        if (!maze_started) {
          maze_started = true;
          surface_action_done = true;
          report("# maze started");
        } else {
          robot.setMotorPWM(0, 0);
          maze_finished = true;
          report("# maze finished");
          return;
        }
      } else if (surface == SURF_TINFOIL) {
        if (robot.startRotateAngle(-PI / 2.0)) {
          maze_turn_started = true;
          maze_turn_seen_running = false;
          maze_turn_start_ms = now;
          surface_action_done = true;
          report("# turning right");
        } else {
          report("# right turn failed");
          robot.setMotorPWM(0, 0);
          maze_finished = true;
        }
        return;
      } else if (surface == SURF_CARPET) {
        if (robot.startRotateAngle(PI / 2.0)) {
          maze_turn_started = true;
          maze_turn_seen_running = false;
          maze_turn_start_ms = now;
          surface_action_done = true;
          report("# turning left");
        } else {
          report("# left turn failed");
          robot.setMotorPWM(0, 0);
          maze_finished = true;
        }
        return;
      } else if (surface == SURF_CARDBOARD) {
        surface_action_done = true;
        report("# going straight");
      }
    }

    // 6) CHECK WHETHER THE ROBOT HAS ENTERED A DIFFERENT SURFACE.
    int observed_surface;
    if (ir < IR_TINFOIL_MAX) {
      observed_surface = SURF_TINFOIL;
    } else if (ir < IR_CARPET_MAX) {
      observed_surface = SURF_CARPET;
    } else if (ir < IR_CARDBOARD_MAX) {
      observed_surface = SURF_CARDBOARD;
    } else {
      observed_surface = SURF_TABLE;
    }

    if (observed_surface != surface) {
      if (observed_surface == candidate_surface) {
        candidate_surface_count++;
      } else {
        candidate_surface = observed_surface;
        candidate_surface_count = 1;
      }

      if (candidate_surface_count >= SURFACE_CHANGE_SAMPLES) {
        robot.setMotorPWM(0, 0);

        // Prepare to classify the new surface.
        surface = SURF_UNKNOWN;
        rest_ir_sum = 0;
        rest_ir_count = 0;

        candidate_surface = SURF_UNKNOWN;
        candidate_surface_count = 0;

        pwm_kick = 0;
        return;
      }
    } else {
      candidate_surface = SURF_UNKNOWN;
      candidate_surface_count = 0;
    }

    // 7) CONTINUE STRAIGHT.
    robot.setMotorPWM(pwm, pwm);
  }

  /** @brief Stands in for Controller_c::report(): prints to the console. */
  void report(String message) {
    println(message);
  }

  /** @return One line describing the controller, shown in the status panel. */
  String getStatusText() {
    String state;
    if (signal == 0) state = "waiting (press B)";
    else if (maze_finished) state = "maze finished";
    else if (maze_turn_started) state = "turning";
    else if (surface == SURF_UNKNOWN) state = "measuring surface";
    else state = "driving";
    return "Controller: " + state + "   Surface: " + SURFACE_NAMES[surface]
      + "   Kick: " + nf(pwm_kick, 0, 0);
  }

  /** @brief Placeholder: Simulator owns graphing in the simulation. */
  void publishTelemetry(Robot_c robot) {
  }
}
