import processing.net.*;
import processing.core.PApplet;
import java.net.*;

/** @brief Initial application-window width in screen pixels. */
final int WINDOW_WIDTH = 1000;
/** @brief Initial application-window height in screen pixels. */
final int WINDOW_HEIGHT = 800;
// The robot starts at the world origin (0, 0). These offsets choose where
// that origin sits on surface.png. To start at image pixel (px, py), use:
//   SURFACE_OFFSET_X_MM = imageWidth / 2.0 - px
//   SURFACE_OFFSET_Y_MM = py - imageHeight / 2.0
// Current start: pixel (450, 165) on the 500 x 875 map. That is the dark
// 909090 margin at the top right, level with the middle of the top-right
// a2a2a2 block, so driving left takes the robot across that block and
// straight onto the white area beyond it.
// In live-comparison mode, place the real robot on the printed map at the
// same spot, because its odometry also starts at (0, 0).
/** @brief Initial surface-image x offset from the world origin, in mm. */
final float SURFACE_OFFSET_X_MM = -200.0;   // 500 / 2 - 450
/** @brief Initial surface-image y offset from the world origin, in mm. */
final float SURFACE_OFFSET_Y_MM = 269.5;    // 707 - 875 / 2
/** @brief Starting heading in radians: 0 = right, PI/2 = up, PI = left, -PI/2 = down. */
final float START_THETA_RAD = PI;

/** @brief Application-level owner of the simulated robot and UI. */
Simulator simulator;

/** @brief Configures Processing's initial drawing surface. */
void settings() {
  size(WINDOW_WIDTH, WINDOW_HEIGHT);
}

/** @brief Creates the resizable simulator window and its application state. */
void setup() {
  
  // Keep a normal window (rather than fullscreen) so students can resize it
  // or place it beside their controller code. Starting at the display size
  // gives the same practical benefit as maximising it at launch.
  surface.setResizable(true);
  
  //surface.setSize(displayWidth, displayHeight);
  //surface.setLocation(0, 0);
  surface.setTitle("CW2026 Digital Twin");
  simulator = new Simulator(this, SURFACE_OFFSET_X_MM, SURFACE_OFFSET_Y_MM);

  // Place the robot at the world origin with the chosen starting heading.
  simulator.robot.resetForRun(0, 0, START_THETA_RAD, 0, 0);

}

/** @brief Advances the application state and renders one frame. */
void draw() {
  simulator.update();
  simulator.draw();
}

/** @brief Forwards keyboard input to the simulator controls.
 * @see Simulator#handleKey(char, int)
 */
void keyPressed() {
  simulator.handleKey(key, keyCode);
}
