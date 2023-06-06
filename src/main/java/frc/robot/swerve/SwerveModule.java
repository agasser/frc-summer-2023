package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule implements SwerveModuleIO {
  private final SwerveSpeedController driveController;
  private final SwerveSteerController steerController;

  public SwerveModule(SwerveSpeedController driveController, SwerveSteerController steerController) {
    this.driveController = driveController;
    this.steerController = steerController;
  }

  @Override
  public void setDesiredState(SwerveModuleState moduleState) {
    driveController.setReferenceVelocity(moduleState.speedMetersPerSecond);
    steerController.setDesiredRotation(moduleState.angle);
  }

  /**
   * Sets the neutral mode for the drive and steer motors
   * @param neutralMode neutral mode
   */
  @Override
  public void setNeutralMode(NeutralMode neutralMode) {
    steerController.setNeutralMode(neutralMode);
    driveController.setNeutralMode(neutralMode);
  }

  /**
   * Reseeds to Talon FX motor offset from the CANCoder. Workaround for "dead wheel"
   */
  @Override
  public void reseedSteerMotorOffset() {
    steerController.configMotorOffset(false);
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    inputs.drivePositionMeters = driveController.getStatePosition();
    inputs.driveVelocityMetersPerSecond = driveController.getStateVelocity();

    inputs.steerAngleRadians = steerController.getStateRotation();
    inputs.steerAbsoluteEncoderPosition = steerController.getAbsoluteEncoderPosition();
  }

}