package frc.robot.swerve;

import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj.DriverStation;

public final class CtreUtils {
  private CtreUtils() {
  }

  public static void checkCtreError(StatusCode statusCode, String message, int port) {
    if (statusCode != StatusCode.OK) {
      DriverStation.reportError(String.format("%s ID: %s StatusCode: ", message, port, statusCode.toString()), false);
    }
  }
}
