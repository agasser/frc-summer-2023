package frc.robot.swerve;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {

  @AutoLog
  public static class SwerveModuleIOInputs {
    public double driveVelocityMetersPerSecond;
    public double drivePositionMeters;

    public double steerAngleRadians;
    public double steerAbsoluteEncoderPosition;
  }

  public default void updateInputs(SwerveModuleIOInputs inputs) {}
  public default void setDesiredState(SwerveModuleState moduleState) {}
  public default void setNeutralMode(NeutralMode neutralMode) {}
  public default void reseedSteerMotorOffset() {}
  
}
