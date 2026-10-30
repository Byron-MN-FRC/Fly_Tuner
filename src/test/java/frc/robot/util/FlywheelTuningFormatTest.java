package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class FlywheelTuningFormatTest {
  @Test
  void formatsPidExportWithFixedPrecision() {
    String result = FlywheelTuningFormat.formatPidExport(0.1, 0.02, 0.003, 0.0004, 0.00005);

    assertEquals("kP=0.100000, kI=0.020000, kD=0.003000, kV=0.000400, kS=0.000050", result);
  }
}
