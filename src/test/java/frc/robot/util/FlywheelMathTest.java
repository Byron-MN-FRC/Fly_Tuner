package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class FlywheelMathTest {
  @Test
  void conversionsAreConsistent() {
    double ratio = 42.0 / 24.0;
    double flywheelRpm = 4200.0;

    double motorRps = FlywheelMath.flywheelRpmToMotorRps(flywheelRpm, ratio);
    double backToFlywheel = FlywheelMath.motorRpsToFlywheelRpm(motorRps, ratio);

    assertEquals(flywheelRpm, backToFlywheel, 1e-6);
  }
}
