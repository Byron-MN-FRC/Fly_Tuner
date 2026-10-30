// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.FlywheelTuningConstants;
import frc.robot.dashboard.FlywheelTuningDashboard;
import frc.robot.dashboard.FlywheelTuningDashboard.PidGains;
import frc.robot.util.FlywheelMath;

/** Flywheel subsystem with onboard TalonFX PID velocity control. */
public class FlywheelSubsystem extends SubsystemBase {
  private TalonFX motor;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);
  private final FlywheelTuningDashboard dashboard;
  private final Timer atSpeedTimer = new Timer();
  private final Timer autoPidTimer = new Timer();

  private String canBus;
  private final boolean inverted;
  private int deviceId;
  private double reinitDelayUntilSeconds = 0.0;

  private double targetRpm = 0.0;
  private PidGains lastGains;
  private AutoPidState autoPidState = AutoPidState.IDLE;
  private double autoPidVelocitySum = 0.0;
  private double autoPidVoltageSum = 0.0;
  private int autoPidSamples = 0;
  private String autoPidProfile = FlywheelTuningConstants.kProfileAggressive;
  private int autoPidIteration = 0;
  private double autoPidLastKv = 0.0;
  private double autoPidKs = 0.0;
  private double autoPidBaseKv = 0.0;
  private double autoPidBaseKp = 0.0;
  private double autoPidMinRpm = FlywheelTuningConstants.kAutoPidMinRpm;
  private double autoPidMaxRpm = FlywheelTuningConstants.kAutoPidMaxRpm;
  private double autoPidMaxErrorSlope = 0.0;
  private double autoPidErrorSum = 0.0;
  private int autoPidErrorSamples = 0;
  private double autoPidLastError = 0.0;
  private double autoPidLastTimestamp = 0.0;
  private double autoPidRawKi = 0.0;
  private double autoPidRawKd = 0.0;
  private PidGains aggressiveGains;
  private PidGains smoothGains;
  private String lastProfileSelected = FlywheelTuningConstants.kProfileAggressive;
  private double autoPidKsSustainStart = -1.0;

  /** Creates a new FlywheelSubsystem. */
  public FlywheelSubsystem(
      FlywheelTuningDashboard dashboard, int deviceId, String canBus, boolean inverted) {
    this.dashboard = dashboard;
    this.deviceId = deviceId;
    this.canBus = canBus;
    this.inverted = inverted;
    motor = new TalonFX(Constants.FlywheelConstants.kTalonId, Constants.FlywheelConstants.kCanBus);

    dashboard.setMotorId(deviceId);
  dashboard.setCanBus(canBus);
  dashboard.setActiveMotor(formatActiveMotorLabel());
  dashboard.setAutoPidStatus("Idle");

    applyBaseConfiguration(inverted);
    applyPidGains(dashboard.getPidGains());

    atSpeedTimer.reset();
  }

  public void setTargetRpm(double flywheelRpm) {
    targetRpm = flywheelRpm;
    double motorRps =
        FlywheelMath.flywheelRpmToMotorRps(
            flywheelRpm, FlywheelConstants.kMotorToFlywheelRatio);
    motor.setControl(velocityRequest.withVelocity(motorRps));
  }

  public void stop() {
    targetRpm = 0.0;
    motor.setControl(velocityRequest.withVelocity(0.0));
  }

  public double getFlywheelRpm() {
    double motorRps = motor.getVelocity().getValueAsDouble();
    return FlywheelMath.motorRpsToFlywheelRpm(
        motorRps, FlywheelConstants.kMotorToFlywheelRatio);
  }

  public boolean atSetpoint() {
    return Math.abs(getFlywheelRpm() - targetRpm) <= FlywheelConstants.kVelocityToleranceRpm;
  }

  private void applyBaseConfiguration(boolean inverted) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted =
        inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    motor.getConfigurator().apply(config);
  }

  private void applyPidGains(PidGains gains) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    motor.getConfigurator().refresh(config);
    config.Slot0.kP = gains.kP();
    config.Slot0.kI = gains.kI();
    config.Slot0.kD = gains.kD();
    config.Slot0.kV = gains.kV();
    config.Slot0.kS = gains.kS();
    motor.getConfigurator().apply(config);
    lastGains = gains;
    dashboard.setPidExport(dashboard.formatPidExport(gains));
  }

  private void resetPidGains() {
    PidGains zeroGains = new PidGains(0.0, 0.0, 0.0, 0.0, 0.0);
    dashboard.setPidGains(zeroGains);
    applyPidGains(zeroGains);
    dashboard.setAutoPidResults("", zeroGains, 0, "Reset", 0.0, 0.0);
  }

  private boolean shouldApplyGains(PidGains gains) {
    if (lastGains == null) {
      return true;
    }
    return lastGains.kP() != gains.kP()
        || lastGains.kI() != gains.kI()
        || lastGains.kD() != gains.kD()
        || lastGains.kV() != gains.kV()
        || lastGains.kS() != gains.kS();
  }

  @Override
  public void periodic() {
    double requestedRpm = dashboard.getTargetRpm();

    if (dashboard.consumeApplyPreset()) {
      double preset = dashboard.getSelectedPresetRpm();
      dashboard.setTargetRpm(preset);
      requestedRpm = preset;
    }

    if (dashboard.consumeApplyMotorId()) {
      updateMotorId(dashboard.getMotorId());
    }

    if (dashboard.consumeApplyCanBus()) {
      updateCanBus(dashboard.getCanBus());
    }

    if (dashboard.consumeTestMotorComm()) {
      testMotorComms();
    }

    if (dashboard.consumeAutoPid()) {
      startAutoPid(dashboard.getSelectedProfile());
    }

    String selectedProfile = dashboard.getSelectedProfile();
    if (!selectedProfile.equals(lastProfileSelected) && autoPidState == AutoPidState.IDLE) {
      applyProfileGains(selectedProfile);
      lastProfileSelected = selectedProfile;
    }

    if (autoPidState != AutoPidState.IDLE) {
      runAutoPid();
      return;
    }

    if (Timer.getFPGATimestamp() < reinitDelayUntilSeconds) {
      stop();
      dashboard.setMotorCommStatus(false, "Reinitializing motor...");
      return;
    }

    if (dashboard.isFlywheelEnabled()) {
      setTargetRpm(requestedRpm);
    } else {
      stop();
    }

    PidGains gains = dashboard.getPidGains();
    if (dashboard.consumeApplyPid() || shouldApplyGains(gains)) {
      applyPidGains(gains);
    }

    if (dashboard.consumeResetPid()) {
      resetPidGains();
    }

    double measuredRpm = getFlywheelRpm();
    boolean atSpeed = atSetpoint();

    dashboard.setMeasuredRpm(measuredRpm);
    dashboard.setAtSpeed(atSpeed);
    dashboard.setActiveMotor(formatActiveMotorLabel());

    updatePhase2State(atSpeed);
  }

  private void updatePhase2State(boolean atSpeed) {
    if (!dashboard.isPhase2Enabled()) {
      atSpeedTimer.reset();
      atSpeedTimer.stop();
      dashboard.setPhase2Ready(false);
      dashboard.setPhase2Prompt("Phase 2 disabled");
      return;
    }

    if (atSpeed) {
      if (!atSpeedTimer.isRunning()) {
        atSpeedTimer.restart();
      }
    } else {
      atSpeedTimer.reset();
      atSpeedTimer.stop();
    }

    boolean ready = atSpeedTimer.hasElapsed(FlywheelConstants.kStableTimeSeconds);
    dashboard.setPhase2Ready(ready);
    dashboard.setPhase2Prompt(
        ready
            ? "Feed ball now"
            : "Spinning up to "
                + String.format("%.0f", targetRpm)
        + " RPM");
  }

  private void updateMotorId(int newId) {
    if (newId == deviceId) {
      return;
    }

    stop();
    deviceId = newId;
    motor = new TalonFX(deviceId, canBus);
    applyBaseConfiguration(inverted);
    applyPidGains(lastGains != null ? lastGains : dashboard.getPidGains());
    reinitDelayUntilSeconds = Timer.getFPGATimestamp() + FlywheelConstants.kReinitDelaySeconds;
  }

  private void updateCanBus(String newCanBus) {
    if (newCanBus == null || newCanBus.isBlank() || newCanBus.equals(canBus)) {
      return;
    }

    stop();
    canBus = newCanBus;
    motor = new TalonFX(deviceId, canBus);
    applyBaseConfiguration(inverted);
    applyPidGains(lastGains != null ? lastGains : dashboard.getPidGains());
    reinitDelayUntilSeconds = Timer.getFPGATimestamp() + FlywheelConstants.kReinitDelaySeconds;
  }

  private void testMotorComms() {
    StatusSignal<?> tempSignal = motor.getDeviceTemp();
    tempSignal.refresh();
    StatusCode status = tempSignal.getStatus();
    boolean ok = status.isOK();
    dashboard.setMotorCommStatus(ok, ok ? "OK" : status.toString());
  }

  private void startAutoPid(String profile) {
    autoPidProfile = profile;
    autoPidState = AutoPidState.KS_RAMP;
    autoPidVelocitySum = 0.0;
    autoPidVoltageSum = 0.0;
    autoPidSamples = 0;
    autoPidIteration = 1;
    autoPidLastKv = 0.0;
    autoPidKs = 0.0;
    autoPidBaseKv = 0.0;
    autoPidBaseKp = 0.0;
    autoPidMinRpm = dashboard.getAutoPidMinRpm();
    autoPidMaxRpm = dashboard.getAutoPidMaxRpm();
  autoPidKsSustainStart = -1.0;
    dashboard.setAutoPidResults(autoPidProfile, new PidGains(0.0, 0.0, 0.0, 0.0, 0.0), 0, "Running", 0.0, 0.0);
    autoPidTimer.reset();
    autoPidTimer.start();
    dashboard.setAutoPidStatus("Auto PID: finding kS");
  }

  private void runAutoPid() {
    if (autoPidState == AutoPidState.KS_RAMP) {
      runKsRamp();
      return;
    }

    if (autoPidState == AutoPidState.SPINUP || autoPidState == AutoPidState.SAMPLE) {
      motor.setControl(dutyCycleRequest.withOutput(FlywheelTuningConstants.kAutoPidDutyCycle));
    }

    if (autoPidState == AutoPidState.SPINUP
        && autoPidTimer.hasElapsed(FlywheelTuningConstants.kAutoPidSpinupSeconds)) {
      autoPidState = AutoPidState.SAMPLE;
      autoPidTimer.reset();
      autoPidVelocitySum = 0.0;
      autoPidVoltageSum = 0.0;
      autoPidSamples = 0;
      dashboard.setAutoPidStatus("Auto PID: sampling (" + autoPidIteration + ")");
    }

    if (autoPidState == AutoPidState.SAMPLE) {
      StatusSignal<?> velocitySignal = motor.getVelocity();
      velocitySignal.refresh();
      StatusSignal<?> voltageSignal = motor.getMotorVoltage();
      voltageSignal.refresh();

      autoPidVelocitySum += velocitySignal.getValueAsDouble();
      autoPidVoltageSum += voltageSignal.getValueAsDouble();
      autoPidSamples++;

      if (autoPidTimer.hasElapsed(FlywheelTuningConstants.kAutoPidSampleSeconds)) {
        finalizeAutoPid();
      }
    }

    if (autoPidState == AutoPidState.PRESTEP) {
      setTargetRpm(autoPidMinRpm);
      if (autoPidTimer.hasElapsed(FlywheelTuningConstants.kAutoPidPrestepSeconds)) {
        beginStepResponse();
      }
    }

    if (autoPidState == AutoPidState.STEP_RESPONSE) {
      runStepResponse();
    }

    if (autoPidState == AutoPidState.HOLD) {
      runHoldPhase();
    }
  }

  private void finalizeAutoPid() {
    if (autoPidSamples <= 0) {
      dashboard.setAutoPidStatus("Auto PID: no samples");
      autoPidState = AutoPidState.IDLE;
      return;
    }

    double avgVelocity = autoPidVelocitySum / autoPidSamples;
    double avgVoltage = autoPidVoltageSum / autoPidSamples;
    double kV = avgVelocity != 0.0 ? avgVoltage / avgVelocity : 0.0;

    double kPFactor =
        FlywheelTuningConstants.kProfileSmooth.equals(autoPidProfile)
            ? FlywheelTuningConstants.kAutoPidSmoothKpFactor
            : FlywheelTuningConstants.kAutoPidAggressiveKpFactor;
    double kP = kV * kPFactor;

    autoPidBaseKv = kV;
    autoPidBaseKp = kP;
    PidGains gains = new PidGains(kP, 0.0, 0.0, kV, autoPidKs);
    dashboard.setPidGains(gains);
    applyPidGains(gains);

    autoPidState = AutoPidState.PRESTEP;
    autoPidTimer.reset();
    autoPidTimer.start();
    dashboard.setAutoPidStatus("Auto PID: prestep (" + autoPidIteration + ")");
  }

  private void beginStepResponse() {
    autoPidState = AutoPidState.STEP_RESPONSE;
    autoPidTimer.reset();
    autoPidTimer.start();
    autoPidMaxErrorSlope = 0.0;
    autoPidErrorSum = 0.0;
    autoPidErrorSamples = 0;
    autoPidLastError = autoPidMaxRpm - getFlywheelRpm();
    autoPidLastTimestamp = Timer.getFPGATimestamp();
    dashboard.setAutoPidStatus("Auto PID: step response (" + autoPidIteration + ")");
  }

  private void runStepResponse() {
    setTargetRpm(autoPidMaxRpm);
    double now = Timer.getFPGATimestamp();
    double error = autoPidMaxRpm - getFlywheelRpm();
    double dt = now - autoPidLastTimestamp;
    if (dt > 1e-3) {
      double slope = (error - autoPidLastError) / dt;
      autoPidMaxErrorSlope = Math.max(autoPidMaxErrorSlope, Math.abs(slope));
      autoPidLastError = error;
      autoPidLastTimestamp = now;
    }

    if (autoPidTimer.hasElapsed(FlywheelTuningConstants.kAutoPidStepSeconds)) {
      autoPidState = AutoPidState.HOLD;
      autoPidTimer.reset();
      autoPidTimer.start();
      autoPidErrorSum = 0.0;
      autoPidErrorSamples = 0;
      dashboard.setAutoPidStatus("Auto PID: hold error (" + autoPidIteration + ")");
    }
  }

  private void runHoldPhase() {
    setTargetRpm(autoPidMaxRpm);
    double error = autoPidMaxRpm - getFlywheelRpm();
    autoPidErrorSum += error;
    autoPidErrorSamples++;

    if (autoPidTimer.hasElapsed(FlywheelTuningConstants.kAutoPidHoldSeconds)) {
      finalizeKiKd();
    }
  }

  private void finalizeKiKd() {
    double avgError = autoPidErrorSamples > 0 ? autoPidErrorSum / autoPidErrorSamples : 0.0;
    double normalizedError = autoPidMaxRpm != 0.0 ? avgError / autoPidMaxRpm : 0.0;
    double normalizedSlope = autoPidMaxRpm != 0.0 ? autoPidMaxErrorSlope / autoPidMaxRpm : 0.0;

  autoPidRawKi = autoPidBaseKp * FlywheelTuningConstants.kAutoPidKiScale * normalizedError;
  autoPidRawKd = autoPidBaseKp * FlywheelTuningConstants.kAutoPidKdScale * normalizedSlope;

  double kI =
    Math.min(
      FlywheelTuningConstants.kAutoPidKiMax,
      Math.max(FlywheelTuningConstants.kAutoPidKiMin, autoPidRawKi));
  double kD =
    Math.min(
      FlywheelTuningConstants.kAutoPidKdMax,
      Math.max(FlywheelTuningConstants.kAutoPidKdMin, autoPidRawKd));

    PidGains gains = new PidGains(autoPidBaseKp, kI, kD, autoPidBaseKv, autoPidKs);
    dashboard.setPidGains(gains);
    applyPidGains(gains);
  storeProfileGains(autoPidProfile, gains);
  dashboard.setAutoPidResults(
    autoPidProfile,
    gains,
    autoPidIteration,
    "Completed",
    autoPidRawKi,
    autoPidRawKd);

    boolean converged =
        autoPidIteration > 1
            && Math.abs(autoPidBaseKv - autoPidLastKv)
                <= FlywheelTuningConstants.kAutoPidKvConvergence;
    autoPidLastKv = autoPidBaseKv;

    if (converged || autoPidIteration >= FlywheelTuningConstants.kAutoPidMaxIterations) {
      dashboard.setAutoPidStatus(
          "Auto PID done (" + autoPidIteration + "): "
              + (FlywheelTuningConstants.kProfileSmooth.equals(autoPidProfile)
                  ? "Smooth"
                  : "Aggressive"));
      autoPidState = AutoPidState.IDLE;
      return;
    }

    autoPidIteration++;
    autoPidState = AutoPidState.KS_RAMP;
    autoPidTimer.reset();
    autoPidTimer.start();
    dashboard.setAutoPidStatus("Auto PID: finding kS (" + autoPidIteration + ")");
  }

  private void storeProfileGains(String profile, PidGains gains) {
    if (FlywheelTuningConstants.kProfileSmooth.equals(profile)) {
      smoothGains = gains;
    } else {
      aggressiveGains = gains;
    }
  }

  private void applyProfileGains(String profile) {
    PidGains gains =
        FlywheelTuningConstants.kProfileSmooth.equals(profile) ? smoothGains : aggressiveGains;
    if (gains == null) {
      return;
    }
    dashboard.setPidGains(gains);
    applyPidGains(gains);
    dashboard.setAutoPidResults(profile, gains, autoPidIteration, "Profile applied", autoPidRawKi, autoPidRawKd);
  }

  private void runKsRamp() {
    double duty = autoPidTimer.get() * FlywheelTuningConstants.kAutoPidKsRampRate;
    duty = Math.min(duty, FlywheelTuningConstants.kAutoPidKsMaxDuty);

    motor.setControl(dutyCycleRequest.withOutput(duty));

    StatusSignal<?> velocitySignal = motor.getVelocity();
    velocitySignal.refresh();
    double velocity = velocitySignal.getValueAsDouble();

    if (Math.abs(velocity) >= FlywheelTuningConstants.kAutoPidKsVelocityThreshold) {
      if (autoPidKsSustainStart < 0.0) {
        autoPidKsSustainStart = Timer.getFPGATimestamp();
      }

      if (Timer.getFPGATimestamp() - autoPidKsSustainStart
          >= FlywheelTuningConstants.kAutoPidKsSustainSeconds) {
        autoPidKs = duty * 12.0;
        autoPidState = AutoPidState.SPINUP;
        autoPidTimer.reset();
        autoPidTimer.start();
        dashboard.setAutoPidStatus("Auto PID: spinning up (" + autoPidIteration + ")");
        return;
      }
    } else {
      autoPidKsSustainStart = -1.0;
    }

    if (duty >= FlywheelTuningConstants.kAutoPidKsMaxDuty) {
      autoPidKs = duty * 12.0;
      autoPidState = AutoPidState.SPINUP;
      autoPidTimer.reset();
      autoPidTimer.start();
      dashboard.setAutoPidStatus("Auto PID: spinning up (" + autoPidIteration + ")");
    } else {
      dashboard.setAutoPidStatus("Auto PID: finding kS (" + autoPidIteration + ")");
    }
  }

  private enum AutoPidState {
    IDLE,
    KS_RAMP,
    SPINUP,
    SAMPLE,
    PRESTEP,
    STEP_RESPONSE,
    HOLD
  }

  private String formatActiveMotorLabel() {
    return deviceId + " @ " + canBus;
  }
}
