// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components.canrange;

import org.littletonrobotics.junction.AutoLog;

public interface CANrangeIO {

  @AutoLog
  public static class CANrangeIOInputs {
    public double distanceMeters = 0.0;
    public boolean isDetected = false;
  }

  public void updateInputs(CANrangeIOInputs inputs);
}
