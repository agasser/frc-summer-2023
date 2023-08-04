package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
  private final SwerveSpeedController driveController;
  private final SwerveSteerController steerController;

  public SwerveModule(SwerveSpeedController driveController, SwerveSteerController steerController) {
    this.driveController = driveController;
    this.steerController = steerController;
  }

  /**
   * Returns the drive velocity in meters per second
   * @return drive velocity in meters per second
   */
  public double getDriveVelocity() {
    return driveController.getStateVelocity();
  }

  public Rotation2d getSteerAngle() {
    return steerController.getStateRotation();
  }

  public void setDesiredState(SwerveModuleState moduleState) {
    driveController.setReferenceVelocity(moduleState.speedMetersPerSecond);
    steerController.setDesiredRotation(moduleState.angle);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), getSteerAngle());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveController.getStatePosition(), getSteerAngle());
  }

  /**
   * Sets the neutral mode for the drive and steer motors
   * @param brakeMode true to use brake mode, false for coast mode
   */
  public void setBrakeMode(boolean brakeMode) {
    steerController.setBrakeMode(brakeMode);
    driveController.setBrakeMode(brakeMode);
  }

}