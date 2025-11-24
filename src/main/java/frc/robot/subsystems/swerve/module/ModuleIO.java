package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.module.Module.ModuleConstants;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {

  @AutoLog
  public static class ModuleIOInputs {
    // For the drive motor
    // The conversions from rotations to meters are done in the SensorToMechanismRatio config for
    // the motors
    public boolean driveConnected = false;
    public double driveVelocityMetersPerSec = 0.0;
    public double drivePositionMeters = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveTempC = 0.0;
    public double driveStatorCurrentAmps = 0.0;
    public double driveSupplyCurrentAmps = 0.0;

    // For the turn motor
    public boolean turnConnected = false;
    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnTempC = 0.0;
    public double turnStatorCurrentAmps = 0.0;
    public double turnSupplyCurrentAmps = 0.0;

    public boolean cancoderConnected = false;
    public Rotation2d cancoderAbsolutePosition = new Rotation2d();
  }

  public void updateInputs(ModuleIOInputs inputs);

  public void setDriveVoltage(double volts, boolean withFoc);

  public default void setDriveVoltage(double volts) {
    setDriveVoltage(volts, true);
  }

  public void setDriveVelocitySetpoint(double setpointMetersPerSecond);

  public void setTurnVoltage(double volts);

  public void setTurnPositionSetpoint(Rotation2d setpoint);

  public ModuleConstants getModuleConstants();
}
