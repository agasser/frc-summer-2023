package frc.robot.swerve;

import com.ctre.phoenix6.BaseStatusSignal;

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
  public double getDriveVelocity(boolean refresh) {
    return driveController.getStateVelocity(refresh);
  }

  public Rotation2d getSteerAngle(boolean refresh) {
    return steerController.getStateRotation(refresh);
  }

  public void setDesiredState(SwerveModuleState moduleState) {
    driveController.setReferenceVelocity(moduleState.speedMetersPerSecond);
    steerController.setDesiredRotation(moduleState.angle);
  }

  public SwerveModuleState getState(boolean refresh) {
    return new SwerveModuleState(getDriveVelocity(refresh), getSteerAngle(refresh));
  }

  public SwerveModulePosition getPosition(boolean refresh) {
    return new SwerveModulePosition(driveController.getStatePosition(refresh), getSteerAngle(refresh));
  }

  /**
   * Sets the neutral mode for the steer motors
   * @param brakeMode true to use brake mode, false for coast mode
   */
  public void setBrakeMode(boolean brakeMode) {
    steerController.setBrakeMode(brakeMode);
    driveController.setBrakeMode(brakeMode);
  }

  /**
   * Gets the status signals for the module
   * @return array in this order: drive position, drive velocity, steer position, steer velocity
   */
  public BaseStatusSignal[] getSignals() {
    var driveSignals = driveController.getSignals();
    var steerSignals = steerController.getSignals();
    return new BaseStatusSignal[] {driveSignals[0], driveSignals[1], steerSignals[0], driveSignals[1]};
  }

}