package frc.robot.controls;

import static frc.robot.Constants.DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
import static frc.robot.Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;
import static frc.robot.Constants.TeleopDriveConstants.JOYSTICK_DEADBAND;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivetrainConstants;

public class JoystickControlBindings implements ControlBindings {

  private final CommandJoystick leftJoystick = new CommandJoystick(0);
  private final CommandJoystick rightJoystick = new CommandJoystick(1);

  @Override
  public Optional<Trigger> reseedSteerMotors() {
    return Optional.of(leftJoystick.button(6));
  }

  @Override
  public Optional<Trigger> resetPose() {
    return Optional.of(leftJoystick.povDown());
  }
  
  @Override
  public Optional<Trigger> wheelsToX() {
    return Optional.of(leftJoystick.button(4));
  }

  @Override
  public DoubleSupplier translationX() {
    return () -> -modifyAxis(leftJoystick.getY()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;
  }
  @Override
  public DoubleSupplier translationY() {
    return () -> -modifyAxis(leftJoystick.getX()) * MAX_VELOCITY_METERS_PER_SECOND;
  }
  
  @Override
  public DoubleSupplier omega() {
    return () -> -modifyAxis(rightJoystick.getX()) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 2;
  }

  @Override
  public BooleanSupplier driverWantsControl() {
    return () -> modifyAxis(leftJoystick.getY()) != 0.0 
        || modifyAxis(leftJoystick.getX()) != 0.0
        || modifyAxis(rightJoystick.getY()) != 0.0
        || modifyAxis(rightJoystick.getX()) != 0.0;
  }
  
  private static double modifyAxis(double value) {
    // Deadband
    value = MathUtil.applyDeadband(value, JOYSTICK_DEADBAND);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
  
}
