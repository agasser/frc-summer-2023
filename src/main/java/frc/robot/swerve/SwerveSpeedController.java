package frc.robot.swerve;

import static com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;
import static com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
import static frc.robot.Constants.DrivetrainConstants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.DrivetrainConstants.DRIVE_kD;
import static frc.robot.Constants.DrivetrainConstants.DRIVE_kI;
import static frc.robot.Constants.DrivetrainConstants.DRIVE_kP;
import static frc.robot.Constants.DrivetrainConstants.DRIVE_kS;
import static frc.robot.Constants.DrivetrainConstants.DRIVE_kV;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

public class SwerveSpeedController {

  private final TalonFX motor;

  /** How many meters per revolution of the wheel (not the motor) */
  private final double metersPerRevolution;

  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0);
  private final StatusSignal<Double> velocitySignal;
  private final StatusSignal<Double> positionSignal;

  public SwerveSpeedController(int port, ModuleConfiguration moduleConfiguration, ShuffleboardContainer container) {
    
    metersPerRevolution = Math.PI * moduleConfiguration.getWheelDiameter();

    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

    motorConfiguration.CurrentLimits.SupplyCurrentLimit = 80;
    motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfiguration.MotorOutput.Inverted =
        moduleConfiguration.isDriveInverted() ? Clockwise_Positive : CounterClockwise_Positive;
    motorConfiguration.Feedback.SensorToMechanismRatio = 1 / moduleConfiguration.getDriveReduction();

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = DRIVE_kP;
    slot0Configs.kI = DRIVE_kI;
    slot0Configs.kD = DRIVE_kD;
    slot0Configs.kS = DRIVE_kS;
    slot0Configs.kV = DRIVE_kV;

    motor = new TalonFX(port, CANIVORE_BUS_NAME);
    CtreUtils.checkCtreError(motor.getConfigurator().apply(motorConfiguration), "Failed to configure Falcon 500", port);
    CtreUtils.checkCtreError(motor.getConfigurator().apply(slot0Configs), "Failed to configure Falcon 500 slot 0", port);
    motor.setSafetyEnabled(true);
    velocitySignal = motor.getVelocity();
    positionSignal = motor.getPosition();
    
    addDashboardEntries(container);
  }

  private void addDashboardEntries(ShuffleboardContainer container) {
    if (container != null) {
      container.addNumber("Current Velocity", () -> getStateVelocity()).withPosition(0, 0);
      container.addNumber("Target Velocity", () -> velocityVoltageRequest.Velocity * metersPerRevolution)
          .withPosition(0, 1);
      container.addNumber("Current Position", () -> getStatePosition()).withPosition(0, 2);
    }
  }

  public void setReferenceVelocity(double velocity) {
    motor.setControl(velocityVoltageRequest.withVelocity(velocity / metersPerRevolution));
  }

  /**
   * Returns velocity in meters per second
   * @return drive velocity in meters per second
   */
  public double getStateVelocity() {
    return velocitySignal.refresh().getValue() * metersPerRevolution;
  }

  /**
   * Returns position in meters
   * @return position in meters
   */
  public double getStatePosition() {
    return positionSignal.refresh().getValue() * metersPerRevolution;
  }

  /**
   * Sets the neutral mode for the drive motor
   * @param brakeMode true to use brake mode, false for coast mode
   */
  public void setBrakeMode(boolean brakeMode) {
    velocityVoltageRequest.OverrideBrakeDurNeutral = brakeMode;
  }

}
