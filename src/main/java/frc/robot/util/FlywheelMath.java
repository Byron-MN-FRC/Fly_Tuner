// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Utility math helpers for flywheel unit conversions. */
public final class FlywheelMath {
  private FlywheelMath() {}

  /** Converts rotations per minute to rotations per second. */
  public static double rpmToRps(double rpm) {
    return rpm / 60.0;
  }

  /** Converts rotations per second to rotations per minute. */
  public static double rpsToRpm(double rps) {
    return rps * 60.0;
  }

  /**
   * Converts flywheel RPM to motor rotations per second.
   *
   * @param flywheelRpm flywheel speed in RPM
   * @param motorToFlywheelRatio motor rotations per flywheel rotation
   * @return motor speed in rotations per second
   */
  public static double flywheelRpmToMotorRps(double flywheelRpm, double motorToFlywheelRatio) {
    return rpmToRps(flywheelRpm) / motorToFlywheelRatio;
  }

  /**
   * Converts motor rotations per second to flywheel RPM.
   *
   * @param motorRps motor speed in rotations per second
   * @param motorToFlywheelRatio motor rotations per flywheel rotation
   * @return flywheel speed in RPM
   */
  public static double motorRpsToFlywheelRpm(double motorRps, double motorToFlywheelRatio) {
    return rpsToRpm(motorRps * motorToFlywheelRatio);
  }
}
