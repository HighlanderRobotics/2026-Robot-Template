// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components.candle;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface CANdleIO {
  @AutoLog
  public static class CANdleIOInputs {
    public boolean connected = false;
  }

  public void updateInputs(CANdleIOInputs inputs);
}
