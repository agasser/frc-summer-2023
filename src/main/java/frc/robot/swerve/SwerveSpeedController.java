package frc.robot.swerve;

import static com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;
import static com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
import static frc.robot.Constants.DrivetrainConstants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.DrivetrainConstants.DRIVE_kD;
import static frc.robot.Constants.DrivetrainConstants.DRIVE_kI;
import static frc.robot.Constants.DrivetrainConstants.DRIVE_kP;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

public class SwerveSpeedController {

  private final TalonFX motor;

  /** How many meters per revolution of the wheel (not the motor) */
  private final double metersPerRevolution;

  private final VelocityTorqueCurrentFOC velocityFOCRequest = new VelocityTorqueCurrentFOC(0);
  private final StatusSignal<Double> velocitySignal;
  private final StatusSignal<Double> positionSignal;

  public SwerveSpeedController(int port, ModuleConfiguration moduleConfiguration, ShuffleboardContainer container) {
    
    metersPerRevolution = Math.PI * moduleConfiguration.getWheelDiameter();

    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

    motorConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    motorConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -40;
    motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfiguration.MotorOutput.Inverted =
        moduleConfiguration.isDriveInverted() ? Clockwise_Positive : CounterClockwise_Positive;
    motorConfiguration.Feedback.SensorToMechanismRatio = 1 / moduleConfiguration.getDriveReduction();

    Slot0Configs slot0Configs = new Slot0Configs();
    // TODO gains are from voltage, not amperage - need to tune
    slot0Configs.kP = DRIVE_kP;
    slot0Configs.kI = DRIVE_kI;
    slot0Configs.kD = DRIVE_kD;

    motor = new TalonFX(port, CANIVORE_BUS_NAME);
    CtreUtils.checkCtreError(motor.getConfigurator().apply(motorConfiguration), "Failed to configure Falcon 500", port);
    CtreUtils.checkCtreError(motor.getConfigurator().apply(slot0Configs), "Failed to configure Falcon 500 slot 0", port);
    motor.setSafetyEnabled(true);
    velocitySignal = motor.getVelocity().clone();
    positionSignal = motor.getPosition().clone();

    // Make request signal synchronous
    velocityFOCRequest.UpdateFreqHz = 0.0;
    
    addDashboardEntries(container);
  }

  private void addDashboardEntries(ShuffleboardContainer container) {
    if (container != null) {
      container.addNumber("Current Velocity", () -> getStateVelocity(false)).withPosition(0, 0);
      container.addNumber("Target Velocity", () -> velocityFOCRequest.Velocity * metersPerRevolution)
          .withPosition(0, 1);
      container.addNumber("Current Position", () -> getStatePosition(false)).withPosition(0, 2);
    }
  }

  public void setReferenceVelocity(double velocity) {
    motor.setControl(velocityFOCRequest.withVelocity(velocity / metersPerRevolution));
  }

  /**
   * Returns velocity in meters per second
   * @return drive velocity in meters per second
   */
  public double getStateVelocity(boolean refresh) {
    if (refresh) {
      velocitySignal.refresh();
    }
    return velocitySignal.getValue() * metersPerRevolution;
  }

  /**
   * Returns position in meters
   * @return position in meters
   */
  public double getStatePosition(boolean refresh) {
    if (refresh) {
      BaseStatusSignal.refreshAll(positionSignal, velocitySignal);
    }
    return BaseStatusSignal.getLatencyCompensatedValue(positionSignal, velocitySignal) * metersPerRevolution;
  }

  /**
   * Sets the neutral mode for the drive motor
   * @param brakeMode true to use brake mode, false for coast mode
   */
  public void setBrakeMode(boolean brakeMode) {
    velocityFOCRequest.OverrideCoastDurNeutral = !brakeMode;
  }

  public BaseStatusSignal[] getSignals() {
    return new BaseStatusSignal[] {positionSignal, velocitySignal};
  }

}
