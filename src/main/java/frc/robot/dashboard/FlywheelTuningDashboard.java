// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.dashboard;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.FlywheelTuningConstants;
import frc.robot.util.FlywheelTuningFormat;
import java.util.Map;

/** Shuffleboard UI helper for flywheel tuning and phase testing. */
public class FlywheelTuningDashboard {
  private final ShuffleboardTab tab;
  private final ShuffleboardTab motorTab;
  private final ShuffleboardTab resultsTab;

  private final GenericEntry targetRpmEntry;
  private final GenericEntry targetRpmSliderEntry;
  private final GenericEntry enableFlywheelEntry;
  private final GenericEntry applyPresetEntry;
  private final GenericEntry motorIdEntry;
  private final GenericEntry applyMotorIdEntry;
  private final GenericEntry canBusEntry;
  private final GenericEntry applyCanBusEntry;
  private final GenericEntry activeMotorEntry;
  private final GenericEntry testCommEntry;
  private final GenericEntry commStatusEntry;
  private final GenericEntry commStatusTextEntry;
  private final SendableChooser<Double> presetChooser = new SendableChooser<>();

  private final GenericEntry kPEntry;
  private final GenericEntry kIEntry;
  private final GenericEntry kDEntry;
  private final GenericEntry kVEntry;
  private final GenericEntry kSEntry;
  private final GenericEntry applyPidEntry;
  private final GenericEntry pidExportEntry;
  private final GenericEntry autoPidEntry;
  private final GenericEntry autoPidStatusEntry;
  private final GenericEntry autoPidMinRpmEntry;
  private final GenericEntry autoPidMaxRpmEntry;
  private final GenericEntry resetPidEntry;
  private final SendableChooser<String> profileChooser = new SendableChooser<>();

  private final GenericEntry resultsProfileEntry;
  private final GenericEntry resultsKvEntry;
  private final GenericEntry resultsKsEntry;
  private final GenericEntry resultsKpEntry;
  private final GenericEntry resultsKiEntry;
  private final GenericEntry resultsKdEntry;
  private final GenericEntry resultsIterEntry;
  private final GenericEntry resultsStatusEntry;
  private final GenericEntry resultsKiRawEntry;
  private final GenericEntry resultsKdRawEntry;

  private final GenericEntry measuredRpmEntry;
  private final GenericEntry atSpeedEntry;

  private final GenericEntry phase2EnabledEntry;
  private final GenericEntry phase2ReadyEntry;
  private final GenericEntry phase2PromptEntry;

  private double lastSliderValue;
  private boolean lastApplyPreset;
  private boolean lastApplyPid;
  private boolean lastApplyMotorId;
  private boolean lastApplyCanBus;
  private boolean lastTestComm;
  private boolean lastAutoPid;
  private boolean lastResetPid;

  /** Creates the Shuffleboard tab and widgets for flywheel tuning. */
  public FlywheelTuningDashboard() {
    tab = Shuffleboard.getTab("Flywheel Tuner");
    motorTab = Shuffleboard.getTab("Motor Def");
  resultsTab = Shuffleboard.getTab("Results");

    ShuffleboardLayout targetLayout =
        tab.getLayout("Target RPM", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 3);

    targetRpmEntry =
        targetLayout
            .add("Target RPM", FlywheelTuningConstants.kDefaultTargetRpm)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();

    targetRpmSliderEntry =
        targetLayout
            .add("Target RPM Slider", FlywheelTuningConstants.kDefaultTargetRpm)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(
                Map.of(
                    "min",
                    FlywheelTuningConstants.kMinTargetRpm,
                    "max",
                    FlywheelTuningConstants.kMaxTargetRpm))
            .getEntry();

    enableFlywheelEntry =
        targetLayout
            .add("Enable Flywheel", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .getEntry();

    applyPresetEntry =
        targetLayout
            .add("Apply Preset", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .getEntry();

  ShuffleboardLayout motorLayout =
    motorTab.getLayout("Motor Identity", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4);

  activeMotorEntry =
    motorLayout
      .add("Active Motor", FlywheelConstants.kTalonId + " @ " + FlywheelConstants.kCanBus)
      .withWidget(BuiltInWidgets.kTextView)
      .getEntry();

  motorIdEntry =
    motorLayout
      .add("Motor CAN ID", FlywheelConstants.kTalonId)
      .withWidget(BuiltInWidgets.kTextView)
      .getEntry();

  applyMotorIdEntry =
    motorLayout
      .add("Apply Motor ID", false)
      .withWidget(BuiltInWidgets.kToggleButton)
      .getEntry();

  canBusEntry =
    motorLayout
      .add("CAN Bus", FlywheelConstants.kCanBus)
      .withWidget(BuiltInWidgets.kTextView)
      .getEntry();

  applyCanBusEntry =
    motorLayout
      .add("Apply CAN Bus", false)
      .withWidget(BuiltInWidgets.kToggleButton)
      .getEntry();

  ShuffleboardLayout commLayout =
    motorTab.getLayout("Comms Test", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 4);

  testCommEntry =
    commLayout
      .add("Test Motor Comms", false)
      .withWidget(BuiltInWidgets.kToggleButton)
      .getEntry();

  commStatusEntry =
    commLayout
      .add("Comms OK", false)
      .withWidget(BuiltInWidgets.kBooleanBox)
      .getEntry();

  commStatusTextEntry =
    commLayout
      .add("Comms Status", "Not tested")
      .withWidget(BuiltInWidgets.kTextView)
      .getEntry();

    presetChooser.setDefaultOption(
        "Default", FlywheelTuningConstants.kDefaultTargetRpm);
    for (int i = 0; i < FlywheelTuningConstants.kPresetRpms.length; i++) {
      double preset = FlywheelTuningConstants.kPresetRpms[i];
      presetChooser.addOption("Preset " + (i + 1) + " (" + preset + " rpm)", preset);
    }
    targetLayout.add("Presets", presetChooser);

  ShuffleboardLayout pidLayout =
    tab.getLayout("PID Gains", BuiltInLayouts.kGrid).withPosition(2, 0).withSize(4, 3);

  kPEntry =
    pidLayout
      .add("kP", FlywheelTuningConstants.kDefaultKP)
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(0, 0)
      .getEntry();
  kIEntry =
    pidLayout
      .add("kI", FlywheelTuningConstants.kDefaultKI)
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(1, 0)
      .getEntry();
  kDEntry =
    pidLayout
      .add("kD", FlywheelTuningConstants.kDefaultKD)
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(2, 0)
      .getEntry();
  kSEntry =
    pidLayout
      .add("kS", FlywheelTuningConstants.kDefaultKS)
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(0, 1)
      .getEntry();
  kVEntry =
    pidLayout
      .add("kV", FlywheelTuningConstants.kDefaultKV)
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(1, 1)
      .getEntry();

  applyPidEntry =
    pidLayout
      .add("Apply PID", false)
      .withWidget(BuiltInWidgets.kToggleButton)
      .withPosition(2, 1)
      .getEntry();

  resetPidEntry =
    pidLayout
      .add("Reset PID", false)
      .withWidget(BuiltInWidgets.kToggleButton)
      .withPosition(3, 1)
      .getEntry();

  autoPidMinRpmEntry =
    pidLayout
      .add("Auto PID Min RPM", FlywheelTuningConstants.kAutoPidMinRpm)
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(0, 2)
      .getEntry();

  autoPidMaxRpmEntry =
    pidLayout
      .add("Auto PID Max RPM", FlywheelTuningConstants.kAutoPidMaxRpm)
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(1, 2)
      .getEntry();

  profileChooser.setDefaultOption("Aggressive", FlywheelTuningConstants.kProfileAggressive);
  profileChooser.addOption("Smooth", FlywheelTuningConstants.kProfileSmooth);
  pidLayout.add("Profile", profileChooser).withPosition(2, 2).withSize(1, 1);

  autoPidEntry =
    pidLayout
      .add("Auto PID", false)
      .withWidget(BuiltInWidgets.kToggleButton)
      .withPosition(3, 2)
      .getEntry();

  pidExportEntry =
    pidLayout
      .add("PID Export", "")
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(0, 3)
      .withSize(4, 1)
      .getEntry();

  autoPidStatusEntry =
    pidLayout
      .add("Auto PID Status", "Idle")
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(0, 4)
      .withSize(4, 1)
      .getEntry();

  ShuffleboardLayout resultsLayout =
    resultsTab.getLayout("Auto PID Results", BuiltInLayouts.kList).withPosition(0, 0).withSize(3, 6);

  resultsProfileEntry =
    resultsLayout
      .add("Profile", "")
      .withWidget(BuiltInWidgets.kTextView)
      .getEntry();
  resultsKvEntry =
    resultsLayout
      .add("kV", 0.0)
      .withWidget(BuiltInWidgets.kTextView)
      .getEntry();
  resultsKsEntry =
    resultsLayout
      .add("kS", 0.0)
      .withWidget(BuiltInWidgets.kTextView)
      .getEntry();
  resultsKpEntry =
    resultsLayout
      .add("kP", 0.0)
      .withWidget(BuiltInWidgets.kTextView)
      .getEntry();
  resultsKiEntry =
    resultsLayout
      .add("kI", 0.0)
      .withWidget(BuiltInWidgets.kTextView)
      .getEntry();
  resultsKdEntry =
    resultsLayout
      .add("kD", 0.0)
      .withWidget(BuiltInWidgets.kTextView)
      .getEntry();
  resultsIterEntry =
    resultsLayout
      .add("Iterations", 0)
      .withWidget(BuiltInWidgets.kTextView)
      .getEntry();
  resultsStatusEntry =
    resultsLayout
      .add("Status", "Idle")
      .withWidget(BuiltInWidgets.kTextView)
      .getEntry();
  resultsKiRawEntry =
    resultsLayout
      .add("kI Raw", 0.0)
      .withWidget(BuiltInWidgets.kTextView)
      .getEntry();
  resultsKdRawEntry =
    resultsLayout
      .add("kD Raw", 0.0)
      .withWidget(BuiltInWidgets.kTextView)
      .getEntry();

    ShuffleboardLayout statusLayout =
        tab.getLayout("Status", BuiltInLayouts.kList).withPosition(0, 3).withSize(2, 3);

    measuredRpmEntry =
        statusLayout
            .add("Measured RPM", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();

    atSpeedEntry =
        statusLayout
            .add("At Speed", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .getEntry();

    ShuffleboardLayout phase2Layout =
        tab.getLayout("Phase 2", BuiltInLayouts.kList).withPosition(2, 3).withSize(4, 3);

    phase2EnabledEntry =
        phase2Layout
            .add("Enable Phase 2", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .getEntry();

    phase2ReadyEntry =
        phase2Layout
            .add("Ready to Feed Ball", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("colorWhenFalse", "red", "colorWhenTrue", "green"))
            .getEntry();

    phase2PromptEntry =
        phase2Layout
            .add("Prompt", "Phase 2 disabled")
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();

    lastSliderValue = FlywheelTuningConstants.kDefaultTargetRpm;
  }

  /** Returns the latest target RPM value, syncing slider and text entries. */
  public double getTargetRpm() {
    double sliderValue = targetRpmSliderEntry.getDouble(FlywheelTuningConstants.kDefaultTargetRpm);
    double textValue = targetRpmEntry.getDouble(FlywheelTuningConstants.kDefaultTargetRpm);

    if (Math.abs(sliderValue - lastSliderValue) > 1e-3
        && Math.abs(sliderValue - textValue) > 1e-3) {
      targetRpmEntry.setDouble(sliderValue);
      textValue = sliderValue;
    } else if (Math.abs(textValue - sliderValue) > 1e-3) {
      targetRpmSliderEntry.setDouble(textValue);
    }

    lastSliderValue = sliderValue;
    return textValue;
  }

  public void setTargetRpm(double rpm) {
    targetRpmEntry.setDouble(rpm);
    targetRpmSliderEntry.setDouble(rpm);
  }

  public int getMotorId() {
    return (int) Math.round(motorIdEntry.getDouble(FlywheelConstants.kTalonId));
  }

  public void setMotorId(int motorId) {
    motorIdEntry.setDouble(motorId);
  }

  public String getCanBus() {
    return canBusEntry.getString(FlywheelConstants.kCanBus);
  }

  public void setCanBus(String canBus) {
    canBusEntry.setString(canBus);
  }

  public void setActiveMotor(String label) {
    activeMotorEntry.setString(label);
  }

  public boolean consumeTestMotorComm() {
    boolean current = testCommEntry.getBoolean(false);
    boolean triggered = current && !lastTestComm;
    lastTestComm = current;
    if (triggered) {
      testCommEntry.setBoolean(false);
    }
    return triggered;
  }

  public void setMotorCommStatus(boolean ok, String message) {
    commStatusEntry.setBoolean(ok);
    commStatusTextEntry.setString(message);
  }

  public boolean isFlywheelEnabled() {
    return enableFlywheelEntry.getBoolean(false);
  }

  public boolean isPhase2Enabled() {
    return phase2EnabledEntry.getBoolean(false);
  }

  public void setPhase2Ready(boolean ready) {
    phase2ReadyEntry.setBoolean(ready);
  }

  public void setPhase2Prompt(String prompt) {
    phase2PromptEntry.setString(prompt);
  }

  public void setMeasuredRpm(double rpm) {
    measuredRpmEntry.setDouble(rpm);
  }

  public void setAtSpeed(boolean atSpeed) {
    atSpeedEntry.setBoolean(atSpeed);
  }

  public void setPidExport(String exportText) {
    pidExportEntry.setString(exportText);
  }

  public void setAutoPidStatus(String status) {
    autoPidStatusEntry.setString(status);
  }

  public void setAutoPidResults(
      String profile,
      PidGains gains,
      int iterations,
      String status,
      double rawKi,
      double rawKd) {
    resultsProfileEntry.setString(profile);
    resultsKvEntry.setDouble(gains.kV());
    resultsKsEntry.setDouble(gains.kS());
    resultsKpEntry.setDouble(gains.kP());
    resultsKiEntry.setDouble(gains.kI());
    resultsKdEntry.setDouble(gains.kD());
    resultsIterEntry.setDouble(iterations);
    resultsStatusEntry.setString(status);
    resultsKiRawEntry.setDouble(rawKi);
    resultsKdRawEntry.setDouble(rawKd);
  }

  public boolean consumeApplyPreset() {
    boolean current = applyPresetEntry.getBoolean(false);
    boolean triggered = current && !lastApplyPreset;
    lastApplyPreset = current;
    if (triggered) {
      applyPresetEntry.setBoolean(false);
    }
    return triggered;
  }

  public double getSelectedPresetRpm() {
    Double selected = presetChooser.getSelected();
    return selected != null ? selected : FlywheelTuningConstants.kDefaultTargetRpm;
  }

  public boolean consumeApplyPid() {
    boolean current = applyPidEntry.getBoolean(false);
    boolean triggered = current && !lastApplyPid;
    lastApplyPid = current;
    if (triggered) {
      applyPidEntry.setBoolean(false);
    }
    return triggered;
  }

  public boolean consumeAutoPid() {
    boolean current = autoPidEntry.getBoolean(false);
    boolean triggered = current && !lastAutoPid;
    lastAutoPid = current;
    if (triggered) {
      autoPidEntry.setBoolean(false);
    }
    return triggered;
  }

  public boolean consumeResetPid() {
    boolean current = resetPidEntry.getBoolean(false);
    boolean triggered = current && !lastResetPid;
    lastResetPid = current;
    if (triggered) {
      resetPidEntry.setBoolean(false);
    }
    return triggered;
  }

  public String getSelectedProfile() {
    String selected = profileChooser.getSelected();
    return selected != null ? selected : FlywheelTuningConstants.kProfileAggressive;
  }

  public double getAutoPidMinRpm() {
    return autoPidMinRpmEntry.getDouble(FlywheelTuningConstants.kAutoPidMinRpm);
  }

  public double getAutoPidMaxRpm() {
    return autoPidMaxRpmEntry.getDouble(FlywheelTuningConstants.kAutoPidMaxRpm);
  }

  public boolean consumeApplyMotorId() {
    boolean current = applyMotorIdEntry.getBoolean(false);
    boolean triggered = current && !lastApplyMotorId;
    lastApplyMotorId = current;
    if (triggered) {
      applyMotorIdEntry.setBoolean(false);
    }
    return triggered;
  }

  public boolean consumeApplyCanBus() {
    boolean current = applyCanBusEntry.getBoolean(false);
    boolean triggered = current && !lastApplyCanBus;
    lastApplyCanBus = current;
    if (triggered) {
      applyCanBusEntry.setBoolean(false);
    }
    return triggered;
  }

  public PidGains getPidGains() {
    return new PidGains(
        kPEntry.getDouble(FlywheelTuningConstants.kDefaultKP),
        kIEntry.getDouble(FlywheelTuningConstants.kDefaultKI),
        kDEntry.getDouble(FlywheelTuningConstants.kDefaultKD),
        kVEntry.getDouble(FlywheelTuningConstants.kDefaultKV),
        kSEntry.getDouble(FlywheelTuningConstants.kDefaultKS));
  }

  public void setPidGains(PidGains gains) {
    kPEntry.setDouble(gains.kP());
    kIEntry.setDouble(gains.kI());
    kDEntry.setDouble(gains.kD());
    kVEntry.setDouble(gains.kV());
    kSEntry.setDouble(gains.kS());
  }

  /** Simple record for PID gain values. */
  public record PidGains(double kP, double kI, double kD, double kV, double kS) {}

  public String formatPidExport(PidGains gains) {
    return FlywheelTuningFormat.formatPidExport(
        gains.kP(), gains.kI(), gains.kD(), gains.kV(), gains.kS());
  }

  public double getVelocityToleranceRpm() {
    return FlywheelConstants.kVelocityToleranceRpm;
  }
}
