// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components.canrange;

import org.littletonrobotics.junction.AutoLog;

public interface CANrangeIO {
  // TODO wherever you use this, create an alert for connected or not

  @AutoLog
  public static class CANrangeIOInputs {
    public boolean connected = false;
    public double distanceMeters = 0.0;
    public boolean isDetected = false;
  }

  public void updateInputs(CANrangeIOInputs inputs);
}
