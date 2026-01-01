// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components.cancoder;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

// for cancoders that aren't on a swerve module (eg arm, intake)
public interface CANcoderIO {
  // TODO wherever you use this, create an alert for connected or not
  @AutoLog
  public static class CANcoderIOInputs {
    public boolean connected = false;
    public Rotation2d cancoderPositionRotations = new Rotation2d();
  }

  public void updateInputs(CANcoderIOInputs inputs);
}
