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

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
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
    
    CtreUtils.checkCtreError(
      encoder.getAbsolutePosition().setUpdateFrequency(10), 
        "Failed to configure CANCoder update rate", canCoderPort);

    var slot0Config = new Slot0Configs();
    slot0Config.kP = STEER_kP;
    slot0Config.kI = STEER_kI;
    slot0Config.kD = STEER_kD;
    slot0Config.kV = STEER_kV;
    
    var motorConfiguration = new TalonFXConfiguration();
    motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    var motionMagicConfig = motorConfiguration.MotionMagic;
    motionMagicConfig.MotionMagicCruiseVelocity = MAX_STEER_ROTATIONS_PER_SECOND * .80;
    motionMagicConfig.MotionMagicAcceleration = MAX_STEER_ROTATIONS_PER_SECOND * 8;
    
    var currentLimitConfig = motorConfiguration.CurrentLimits;
    currentLimitConfig.SupplyCurrentLimit = 20;
    currentLimitConfig.SupplyCurrentLimitEnable = true;
    
    motorConfiguration.ClosedLoopGeneral.ContinuousWrap = true;
    motorConfiguration.Feedback.SensorToMechanismRatio = 1 / moduleConfiguration.getSteerReduction();
    motorConfiguration.MotorOutput.Inverted = 
        moduleConfiguration.isSteerInverted() ? Clockwise_Positive : CounterClockwise_Positive;

    motor = new TalonFX(motorPort, CANIVORE_BUS_NAME);
    CtreUtils.checkCtreError(motor.getConfigurator().apply(motorConfiguration),
        "Failed to configure Falcon 500", motorPort);
    CtreUtils.checkCtreError(motor.getConfigurator().apply(slot0Config),
        "Failed to configure Falcon 500 slot 0", motorPort);

    configMotorOffset();
    motorPositionSignal = motor.getPosition();
    motorPositionSignal.setUpdateFrequency(100.0);
    addDashboardEntries(container);
  }

  private void addDashboardEntries(ShuffleboardContainer container) {
    if (container != null) {
      container.addNumber("Motor Angle", () -> getStateRotation().getRadians()).withPosition(0, 3);
      container.addNumber("Target Angle", () -> rotationsToRadians(motionMagicVoltageRequest.Position))
          .withPosition(0, 4);
      container.addNumber("Encoder Angle", () -> rotationsToRadians(encoder.getAbsolutePosition().getValue()))
          .withPosition(0, 5);
    }
  }

  /**
   * Configures the motor offset from the CANCoder's abosolute position.
   */
  public void configMotorOffset() {
    // With Phoenix Pro, the CAN Coder and motor encoder can be fused, so this wouldn't be needed
    var position = encoder.getAbsolutePosition().waitForUpdate(0.2);
    CtreUtils.checkCtreError(position.getError(), "Failed to read position from CANcoder", encoder.getDeviceID());
    // There's a bug in Phoenix 6, this is the mechanism position, not rotor position
    // https://api.ctr-electronics.com/changelog#known-issues-20230607
    CtreUtils.checkCtreError(
        motor.setRotorPosition(position.getValue()), "Failed to set motor position", motor.getDeviceID());
  }

  /**
   * Sets the rotation setpoint
   * @param desiredRotation desired rotation setpoint
   */
  public void setDesiredRotation(Rotation2d desiredRotation) {
    motionMagicVoltageRequest.Position = desiredRotation.getRotations();
    motor.setControl(motionMagicVoltageRequest);
  }

  /**
   * Gets the current wheel rotation
   * @return Range is [-PI, PI] radians
   */
  public Rotation2d getStateRotation() {
    var position = motorPositionSignal.refresh().getValue();
    return Rotation2d.fromRotations(Math.IEEEremainder(position, 1));
  }

  /**
   * Sets the neutral mode to brake mode or coast mode
   * @param brakeMode true to use brake mode, false for coast mode
   */
  public void setBrakeMode(boolean brakeMode) {
    motionMagicVoltageRequest.OverrideBrakeDurNeutral = brakeMode;
  }

}