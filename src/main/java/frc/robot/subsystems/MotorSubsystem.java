// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simple motor subsystem wrapper that allows dependency injection of the motor
 * controller from {@code RobotContainer}.
 */
public class MotorSubsystem extends SubsystemBase {
  private final MotorController motor;
  private double lastSpeed = 0.0;

  /** Creates a new MotorSubsystem. */
  public MotorSubsystem(MotorController motor) {
    this.motor = motor;
  }

  /**
   * Sets motor output speed.
   *
   * @param speed motor output in the range [-1.0, 1.0]
   */
  public void setSpeed(double speed) {
    lastSpeed = speed;
    motor.set(speed);
  }

  /** Stops the motor. */
  public void stop() {
    setSpeed(0.0);
  }

  /**
   * Returns the last speed commanded to the motor.
   *
   * @return last commanded motor speed
   */
  public double getLastSpeed() {
    return lastSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
