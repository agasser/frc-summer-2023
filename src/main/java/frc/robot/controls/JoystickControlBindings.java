package frc.robot.controls;

import static frc.robot.Constants.DrivetrainConstants.MAX_ANGULAR_VELOCITY;
import static frc.robot.Constants.DrivetrainConstants.MAX_VELOCITY;
import static frc.robot.Constants.TeleopDriveConstants.JOYSTICK_DEADBAND;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JoystickControlBindings implements ControlBindings {

  private final CommandJoystick leftJoystick = new CommandJoystick(0);
  private final CommandJoystick rightJoystick = new CommandJoystick(1);

  @Override
  public Optional<Trigger> resetPose() {
    return Optional.of(leftJoystick.povDown());
  }
  
  @Override
  public Optional<Trigger> wheelsToX() {
    return Optional.of(leftJoystick.button(4));
  }

  @Override
  public Supplier<Measure<Velocity<Distance>>> translationX() {
    return () -> MAX_VELOCITY.times(-modifyAxis(leftJoystick.getY()));
  }
  @Override
  public Supplier<Measure<Velocity<Distance>>> translationY() {
    return () -> MAX_VELOCITY.times(-modifyAxis(leftJoystick.getX()));
  }
  
  @Override
  public Supplier<Measure<Velocity<Angle>>> omega() {
    return () -> MAX_ANGULAR_VELOCITY.times(-modifyAxis(rightJoystick.getX()) / 2.0);
  }
  
  private static double modifyAxis(double value) {
    // Deadband
    value = MathUtil.applyDeadband(value, JOYSTICK_DEADBAND);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
  
}
