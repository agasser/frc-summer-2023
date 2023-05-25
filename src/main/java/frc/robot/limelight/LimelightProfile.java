package frc.robot.limelight;

import static frc.robot.Constants.VisionConstants.LIMELIGHT_TO_ROBOT;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

/**
 * Limelight profiles enum that holds info about the limelight used for various targets
 */
public enum LimelightProfile {

  SCORE_CONE_TOP(0, Units.inchesToMeters(44.0), LIMELIGHT_TO_ROBOT);

  /** ID for the Limelight profile */
  public final int pipelineId;

  /** Height of the target in meters */
  public final double targetHeight;

  /** Camera to robot transform for the targeting camera */
  public final Transform3d cameraToRobot;

  /**
   * Create a limelight profile
   * @param targetHeight target height
   * @param cameraToRobot transform from the camera to the robot center
   */
  private LimelightProfile(
      int pipelineId, double targetHeight, Transform3d cameraToRobot) {
    this.pipelineId = pipelineId;
    this.targetHeight = targetHeight;
    this.cameraToRobot = cameraToRobot;
  }

}