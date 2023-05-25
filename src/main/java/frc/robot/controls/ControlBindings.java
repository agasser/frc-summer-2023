package frc.robot.controls;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ControlBindings {
  Optional<Trigger> resetPose();
  Optional<Trigger> reseedSteerMotors();
  Optional<Trigger> wheelsToX();
  DoubleSupplier translationX();
  DoubleSupplier translationY();
  DoubleSupplier omega();
  BooleanSupplier driverWantsControl();
}
