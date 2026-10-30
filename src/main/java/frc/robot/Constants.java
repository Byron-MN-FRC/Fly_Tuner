// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class FlywheelConstants {
    public static final int kTalonId = 24;
    public static final String kCanBus = "CANivore2";
    public static final boolean kInverted = false;
    // Motor rotations per flywheel rotation.
    public static final double kMotorToFlywheelRatio = 42.0 / 24.0;
    public static final double kFlywheelDiameterInches = 4.0;
    public static final double kVelocityToleranceRpm = 75.0;
    public static final double kStableTimeSeconds = 0.25;
    public static final double kReinitDelaySeconds = 0.5;
  }

  public static class FlywheelTuningConstants {
    public static final double kDefaultTargetRpm = 3000.0;
    public static final double kMinTargetRpm = 0.0;
    public static final double kMaxTargetRpm = 6000.0;

    // Initial PID/FF guesses. Update these during tuning.
  public static final double kDefaultKP = 0.05;
    public static final double kDefaultKI = 0.0;
    public static final double kDefaultKD = 0.0;
  public static final double kDefaultKV = 0.12;
  public static final double kDefaultKS = 0.0;

    public static final double[] kPresetRpms = {2500.0, 3500.0, 4500.0};

    public static final String kProfileAggressive = "Aggressive";
    public static final String kProfileSmooth = "Smooth";

    public static final double kAutoPidDutyCycle = 0.4;
    public static final double kAutoPidSpinupSeconds = 1.0;
    public static final double kAutoPidSampleSeconds = 0.5;
    public static final double kAutoPidAggressiveKpFactor = 0.12;
    public static final double kAutoPidSmoothKpFactor = 0.06;
    public static final int kAutoPidMaxIterations = 5;
    public static final double kAutoPidKvConvergence = 0.005;
  public static final double kAutoPidKsRampRate = 0.01;
  public static final double kAutoPidKsMaxDuty = 0.35;
  public static final double kAutoPidKsVelocityThreshold = 0.5;
  public static final double kAutoPidKsSustainSeconds = 1.0;
    public static final double kAutoPidMinRpm = 1500.0;
    public static final double kAutoPidMaxRpm = 4500.0;
  public static final double kAutoPidPrestepSeconds = 0.4;
    public static final double kAutoPidStepSeconds = 0.6;
    public static final double kAutoPidHoldSeconds = 1.0;
    public static final double kAutoPidKiScale = 0.4;
    public static final double kAutoPidKdScale = 0.02;
    public static final double kAutoPidKiMin = 0.0;
    public static final double kAutoPidKiMax = 0.5;
    public static final double kAutoPidKdMin = 0.0;
    public static final double kAutoPidKdMax = 0.2;
  }
}
