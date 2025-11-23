// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components.cancoder;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface CANcoderIO {
  @AutoLog
  public static class CANcoderIOInputs {
    public Rotation2d cancoderPositionRotations = new Rotation2d();
  }

  public void updateInputs(CANcoderIOInputs inputs);
}
