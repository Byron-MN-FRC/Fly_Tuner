// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Helper for formatting PID export values consistently. */
public final class FlywheelTuningFormat {
  private FlywheelTuningFormat() {}

  public static String formatPidExport(double kP, double kI, double kD, double kV, double kS) {
    return String.format("kP=%.6f, kI=%.6f, kD=%.6f, kV=%.6f, kS=%.6f", kP, kI, kD, kV, kS);
  }
}
