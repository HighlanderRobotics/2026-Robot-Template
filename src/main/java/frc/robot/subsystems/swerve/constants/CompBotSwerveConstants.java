// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Mass;
import frc.robot.components.camera.Camera.CameraConstants;
import frc.robot.subsystems.swerve.module.Module.ModuleConstants;

/** Add your docs here. */
public class CompBotSwerveConstants extends SwerveConstants {

  @Override
  public CameraConstants[] getCameraConstants() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getCameraConstants'");
  }

  @Override
  public String getName() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getName'");
  }

  @Override
  public double getTrackWidthX() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getTrackWidthX'");
  }

  @Override
  public double getTrackWidthY() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getTrackWidthY'");
  }

  @Override
  public double getBumperWidth() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getBumperWidth'");
  }

  @Override
  public double getBumperLength() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getBumperLength'");
  }

  @Override
  public double getMaxLinearSpeed() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getMaxLinearSpeed'");
  }

  @Override
  public double getMaxLinearAcceleration() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getMaxLinearAcceleration'");
  }

  @Override
  public double getDriveGearRatio() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getDriveGearRatio'");
  }

  @Override
  public double getTurnGearRatio() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getTurnGearRatio'");
  }

  @Override
  public Mass getMass() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getMass'");
  }

  @Override
  public ModuleConstants getFrontLeftModuleConstants() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getFrontLeftModuleConstants'");
  }

  @Override
  public ModuleConstants getFrontRightModuleConstants() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getFrontRightModuleConstants'");
  }

  @Override
  public ModuleConstants getBackLeftModuleConstants() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getBackLeftModuleConstants'");
  }

  @Override
  public ModuleConstants getBackRightModuleConstants() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getBackRightModuleConstants'");
  }

  @Override
  public int getGyroID() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getGyroID'");
  }

  @Override
  public Pigeon2Configuration getGyroConfig() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getGyroConfig'");
  }

  @Override
  public TalonFXConfiguration getDriveConfiguration() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getDriveConfiguration'");
  }

  @Override
  public TalonFXConfiguration getTurnConfiguration(int cancoderID) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getTurnConfiguration'");
  }

  @Override
  public CANcoderConfiguration getCancoderConfiguration(Rotation2d cancoderOffset) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getCancoderConfiguration'");
  }

  @Override
  public double getHeadingVelocityKP() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getHeadingVelocityKP'");
  }
}
