package frc.robot.swerve;

import static com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;
import static com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
import static edu.wpi.first.math.util.Units.rotationsToRadians;
import static frc.robot.Constants.DrivetrainConstants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.DrivetrainConstants.MAX_STEER_ROTATIONS_PER_SECOND;
import static frc.robot.Constants.DrivetrainConstants.STEER_kD;
import static frc.robot.Constants.DrivetrainConstants.STEER_kI;
import static frc.robot.Constants.DrivetrainConstants.STEER_kP;
import static frc.robot.Constants.DrivetrainConstants.STEER_kV;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

public class SwerveSteerController {

  private final TalonFX motor;
  private final CANcoder encoder;
  
  private final MotionMagicVoltage motionMagicVoltageRequest = new MotionMagicVoltage(0);
  private final StatusSignal<Double> motorPositionSignal;
  private final StatusSignal<Double> motorVelocitySignal;

  public SwerveSteerController(
      int motorPort,
      int canCoderPort,
      double canCoderOffset,
      ShuffleboardContainer container, 
      ModuleConfiguration moduleConfiguration) {

    // Configure the encoder
    var cancoderConfig = new CANcoderConfiguration();
    var magSensorConfig = cancoderConfig.MagnetSensor;
    magSensorConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    magSensorConfig.MagnetOffset = Units.radiansToRotations(canCoderOffset);
    magSensorConfig.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    encoder = new CANcoder(canCoderPort, CANIVORE_BUS_NAME);
    CtreUtils.checkCtreError(
        encoder.getConfigurator().apply(cancoderConfig), 
        "Failed to configure CANCoder", canCoderPort);

    var motorConfiguration = new TalonFXConfiguration();
    motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    var motionMagicConfig = motorConfiguration.MotionMagic;
    motionMagicConfig.MotionMagicCruiseVelocity = MAX_STEER_ROTATIONS_PER_SECOND * .80;
    motionMagicConfig.MotionMagicAcceleration = MAX_STEER_ROTATIONS_PER_SECOND * 8;
    
    var slot0Config = motorConfiguration.Slot0;
    slot0Config.kP = STEER_kP;
    slot0Config.kI = STEER_kI;
    slot0Config.kD = STEER_kD;
    slot0Config.kV = STEER_kV;

    var currentLimitConfig = motorConfiguration.CurrentLimits;
    currentLimitConfig.SupplyCurrentLimit = 20;
    currentLimitConfig.SupplyCurrentLimitEnable = true;
    
    motorConfiguration.ClosedLoopGeneral.ContinuousWrap = true;
    motorConfiguration.Feedback.RotorToSensorRatio = 1 / moduleConfiguration.getSteerReduction();
    motorConfiguration.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    motorConfiguration.MotorOutput.Inverted = 
        moduleConfiguration.isSteerInverted() ? Clockwise_Positive : CounterClockwise_Positive;

    motor = new TalonFX(motorPort, CANIVORE_BUS_NAME);
    CtreUtils.checkCtreError(motor.getConfigurator().apply(motorConfiguration),
        "Failed to configure Falcon 500", motorPort);

    motorPositionSignal = motor.getPosition().clone();
    motorVelocitySignal = motor.getVelocity().clone();

    // Make request signal synchronous
    motionMagicVoltageRequest.UpdateFreqHz = 0.0;

    addDashboardEntries(container);
  }

  private void addDashboardEntries(ShuffleboardContainer container) {
    if (container != null) {
      container.addNumber("Motor Angle", () -> getStateRotation(false).getRadians()).withPosition(0, 3);
      container.addNumber("Target Angle", () -> rotationsToRadians(motionMagicVoltageRequest.Position))
          .withPosition(0, 4);
      container.addNumber("Encoder Angle", () -> rotationsToRadians(encoder.getAbsolutePosition().getValue()))
          .withPosition(0, 5);
    }
  }

  /**
   * Sets the rotation setpoint
   * @param desiredRotation desired rotation setpoint
   */
  public void setDesiredRotation(Rotation2d desiredRotation) {
    motor.setControl(motionMagicVoltageRequest.withPosition(desiredRotation.getRotations()));
  }

  /**
   * Gets the current wheel rotation
   * @return Range is [-.5, .5] radians
   */
  public Rotation2d getStateRotation(boolean refresh) {
    if (refresh) {
      BaseStatusSignal.refreshAll(motorPositionSignal, motorVelocitySignal);
    }
    var position = BaseStatusSignal.getLatencyCompensatedValue(motorPositionSignal, motorVelocitySignal);
    return Rotation2d.fromRotations(Math.IEEEremainder(position, 1));
  }

  /**
   * Sets the neutral mode to brake mode or coast mode
   * @param brakeMode true to use brake mode, false for coast mode
   */
  public void setBrakeMode(boolean brakeMode) {
    motionMagicVoltageRequest.OverrideBrakeDurNeutral = brakeMode;
  }

  public BaseStatusSignal[] getSignals() {
    return new BaseStatusSignal[] {motorPositionSignal, motorVelocitySignal};
  }

}