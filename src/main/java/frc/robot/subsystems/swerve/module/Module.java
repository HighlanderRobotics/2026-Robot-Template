package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

// A single module
public class Module {
  // Represents module specific constants
  public record ModuleConstants(
      int id, String prefix, int driveID, int turnID, int cancoderID, Rotation2d cancoderOffset) {}

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final ModuleConstants constants;

  // Connection alerts
  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderDisconnectedAlert;

  // Connected debouncers
  private final Debouncer driveMotorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnMotorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnEncoderConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public Module(ModuleIO io) {
    this.io = io;
    this.constants = io.getModuleConstants();

    driveDisconnectedAlert =
        new Alert("Disconnected drive motor on " + constants.prefix + " module!", AlertType.kError);
    turnDisconnectedAlert =
        new Alert("Disconnected turn motor on " + constants.prefix + " module!", AlertType.kError);
    turnEncoderDisconnectedAlert =
        new Alert(
            "Disconnected turn encoder on " + constants.prefix + " module!", AlertType.kError);
  }

  // Updates and logs the IO layer inputs
  // This class isn't a Subsystem, so periodic() needs to be called in a subsystem periodic to run
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Swerve/" + constants.prefix + " Module", inputs);

    // Update alerts
    driveDisconnectedAlert.set(!driveMotorConnectedDebouncer.calculate(inputs.driveConnected));
    turnDisconnectedAlert.set(!turnMotorConnectedDebouncer.calculate(inputs.turnConnected));
    turnEncoderDisconnectedAlert.set(
        !turnEncoderConnectedDebouncer.calculate(inputs.cancoderConnected));
  }

  public void stop() {
    io.setDriveVoltage(0);
  }

  /**
   * Runs closed-loop to the specified state. Used for automated actions, i.e. auto, autoalign etc.
   *
   * @param state setpoint state
   * @return the optimized state
   */
  public SwerveModuleState runClosedLoop(SwerveModuleState state) {
    state.optimize(getAngle());

    io.setTurnPositionSetpoint(state.angle);
    io.setDriveVelocitySetpoint(state.speedMetersPerSecond);

    return state;
  }

  /**
   * Runs open loop to the specified state, i.e. for teleop
   *
   * @param state the setpoint
   * @param focEnabled
   * @return the optimized state
   */
  public SwerveModuleState runOpenLoop(SwerveModuleState state, boolean focEnabled) {
    state.optimize(getAngle());

    // Converts speed to voltage
    // Essentially, finds what proportion of the maximum speed the setpoint is, then, applies that
    // same proportion of the max supply voltage (12) to the motor
    // i.e. if the max speed is 10 m/s, and we want to go 5 m/s, that's 1/2 of the max speed. So we
    // apply 1/2 of the max supply voltage to the motor: 6V
    double volts =
        state.speedMetersPerSecond * 12 / SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed();

    runVoltageSetpoint(volts, state.angle, focEnabled);

    return state;
  }

  private void runVoltageSetpoint(double volts, Rotation2d targetAngle, boolean focEnabled) {
    io.setTurnPositionSetpoint(targetAngle);
    io.setDriveVoltage(
        // I think this is supposed avoid moving the drive too much when the turn motor is out of
        // position
        volts * Math.cos(targetAngle.minus(inputs.turnPosition).getRadians()), focEnabled);
  }

  /** Returns the current turn angle of the module at normal sampling frequency. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters at normal sampling frequency. */
  public double getPositionMeters() {
    return inputs.drivePositionMeters;
  }

  /** Returns this modules prefix ie "Back Left" */
  public String getPrefix() {
    return constants.prefix;
  }

  /**
   * Returns the current drive velocity of the module in meters per second at normal sampling
   * frequency.
   */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityMetersPerSec;
  }

  /** Returns the module position (turn angle and drive position) at normal sampling frequency. */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity) at normal sampling frequency. */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  public void setTurnSetpoint(Rotation2d rotation) {
    io.setTurnPositionSetpoint(rotation);
  }
}
