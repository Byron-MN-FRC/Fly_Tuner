# Flywheel Tuner

This project provides a Shuffleboard-driven flywheel tuning workflow for a Talon FX (Kraken) motor using onboard velocity PID.

## Quick start
1. Deploy the code to the robot.
2. Open Shuffleboard and select the **"Flywheel Tuner"** tab.
3. Set a target RPM (text box or slider).
4. Toggle **Enable Flywheel** to spin up.

## Flywheel Tuner tab
### Target RPM controls
- **Target RPM** text box (authoritative)
- **Target RPM Slider** (tracks the text value)
- **Enable Flywheel** toggle
- **Presets** chooser + **Apply Preset** button

### PID Gains panel
- Gains are shown in logical order: **kP**, **kI**, **kD**, **kS**, **kV**
- **Apply PID** pushes the values to the motor controller
- **PID Export** shows a copyable string of the current gains
- **Profile** chooser selects the Auto PID strategy
- **Auto PID** runs the automatic tuning routine
- **Auto PID Status** displays progress and completion

## Motor Def tab
Use this tab to set up the motor hardware and test communications.

- **Motor CAN ID** + **Apply Motor ID**
- **CAN Bus** + **Apply CAN Bus**
- **Active Motor** shows the active configuration (format: `ID @ bus`)
- **Test Motor Comms** verifies the motor responds
- **Comms OK** and **Comms Status** show the result

## Phase 2 Test
- Toggle **Enable Phase 2** to start the recovery test.
- The **Ready to Feed Ball** box stays red while spinning up, then turns green once the flywheel is at speed.
- When green, manually feed a ball to observe recovery behavior.

## Notes
- CAN ID defaults to 23 on bus "rio"; adjust in `Constants.FlywheelConstants` if needed.
- Phoenix 6 is managed via vendordeps (currently 26.1.1); update vendordeps if you need a newer release.

## Testing
Run the unit tests with Gradle:

```powershell
./gradlew.bat test
```

Current tests cover math conversions and PID export formatting.

## Troubleshooting
- If CTRE classes fail to resolve, confirm the Phoenix 6 vendordep is installed.
- Ensure the Talon FX is on the configured CAN bus and ID.

## How Auto PID works (kP, kI, kD, kV, kS)
Auto PID uses a simple iterative routine to estimate feedforward and apply a profile-specific proportional gain:

1. **Spin-up**: The motor runs at a fixed duty cycle for a short time.
2. **Sampling**: The routine samples **motor velocity** and **motor voltage** for a brief window.
3. **kV estimate**: $k_V$ is estimated as $\frac{\text{avg voltage}}{\text{avg velocity}}$.
4. **kP selection**: A profile factor scales $k_V$ to get $k_P$.
	- **Aggressive** uses a larger factor (faster response)
	- **Smooth** uses a smaller factor (slower, steadier response)
5. **kI, kD, kS**: Currently set to 0 as a safe starting point.
6. **Iterate**: The routine repeats until $k_V$ stabilizes (converges) or the max iteration count is reached.

After Auto PID completes, the gains are written to the controller and displayed in the PID panel.
