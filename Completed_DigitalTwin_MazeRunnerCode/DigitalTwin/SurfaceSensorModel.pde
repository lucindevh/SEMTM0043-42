/** @brief Cached five-channel surface-sensor data exposed to the controller. */
class RobotSurface_c {
  float[] reading = {280, 280, 280, 280, 280};
}

/**
 * @brief Ideal image-sampling model for one downward-facing surface sensor.
 * @details Local coordinates are in mm in the robot frame. The source image
 * is treated as 1 pixel per mm, centred on its configured world offset, with
 * image y inverted relative to world y.
 */
class SurfaceSensorModel_c {
  String name;
  float localX;
  float localY;

  // Exercise 9: reference readings, owned by each sensor. With these values
  // the map tones give: 255 -> 280 (M1), 191 -> 712 (M2), 162 -> 907 (M3),
  // 144 -> 1029 (M4), 0 -> 2000 (black line).
  float whiteReference = 280;    // reading for brightness 255
  float blackReference = 2000;   // reading for brightness 0

  // Simulation-only convenience: when true, a sensor beyond the edge of the
  // image reads as black, so the controller treats the map edge as a wall.
  boolean outsideImageIsWall = true;

  /**
   * @param nameIn Human-readable channel name, such as DN1.
   * @param radiusMm Sensor distance from the robot centre, in mm.
   * @param angleDegrees Sensor angle in the robot frame, in degrees.
   */
  SurfaceSensorModel_c(String nameIn, float radiusMm, float angleDegrees) {
    name = nameIn;
    localX = radiusMm * cos(radians(angleDegrees));
    localY = radiusMm * sin(radians(angleDegrees));
  }

  /** @brief Straight line from brightness 0..255 to blackReference..whiteReference. */
  float linearReadingForBrightness(float brightnessValue) {
    return map(brightnessValue, 0, 255, blackReference, whiteReference);
  }

  /** @brief Selects the response model used by sample(...). */
  float applySensorModel(float brightnessValue) {
    return linearReadingForBrightness(brightnessValue);
  }

  /**
   * @brief Converts an image brightness sample into an ideal sensor reading.
   * @param image Surface image, centred on the supplied world offset.
   * @param surfaceOffsetX Surface-image x offset in world millimetres.
   * @param surfaceOffsetY Surface-image y offset in world millimetres.
   * @param robotX Robot x position in mm.
   * @param robotY Robot y position in mm.
   * @param robotTheta Robot heading in radians.
   */
  float sample(PImage image, float surfaceOffsetX, float surfaceOffsetY,
    float robotX, float robotY, float robotTheta) {
    float worldX = robotX + localX * cos(robotTheta) - localY * sin(robotTheta);
    float worldY = robotY + localX * sin(robotTheta) + localY * cos(robotTheta);
    int pixelX = floor(image.width / 2.0 + worldX - surfaceOffsetX);
    int pixelY = floor(image.height / 2.0 - worldY + surfaceOffsetY);
    if (pixelX < 0 || pixelX >= image.width || pixelY < 0 || pixelY >= image.height) {
      return applySensorModel(outsideImageIsWall ? 0 : 255);
    }
    return applySensorModel(brightness(image.get(pixelX, pixelY)));
  }

  /** @brief Draws a small sensor marker in the robot frame.
   * @param reading Current ideal sensor reading, used to set marker shade.
   */
  void draw(float reading) {
    pushStyle();
    fill(constrain(map(reading, whiteReference, blackReference, 220, 20), 20, 220));
    stroke(20);
    rectMode(CENTER);
    rect(localX, localY, 3, 3);
    popStyle();
  }
}
