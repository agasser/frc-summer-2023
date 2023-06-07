package frc.robot.swerve;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj.DriverStation;

public final class CtreUtils {
  private CtreUtils() {
  }

  public static void checkCtreError(ErrorCode errorCode, String message) {
    if (errorCode != ErrorCode.OK) {
      DriverStation.reportError(String.format("%s: %s", message, errorCode.toString()), false);
    }
  }

  public static void checkCtreError(StatusCode statusCode, String message) {
    if (statusCode != StatusCode.OK) {
      DriverStation.reportError(String.format("%s: %s", message, statusCode.toString()), false);
    }
  }
}
