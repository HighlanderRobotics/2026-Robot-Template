// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components.canrange;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;

public class CANrangeIOReal implements CANrangeIO {
  private final CANrange canrange;

  private final StatusSignal<Distance> distance;
  private final StatusSignal<Boolean> isDetected;

  public CANrangeIOReal(int CANrangeID) {
    canrange = new CANrange(CANrangeID, "*");
    distance = canrange.getDistance();
    isDetected = canrange.getIsDetected();

    final CANrangeConfiguration config = new CANrangeConfiguration();
    config.ToFParams.UpdateFrequency = 50; // update frequency in Hz
    config.ProximityParams.ProximityThreshold = 0.05;

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, distance, isDetected);

    canrange.getConfigurator().apply(config);
    canrange.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(CANrangeIOInputs inputs) {
    BaseStatusSignal.refreshAll(distance, isDetected);

    inputs.distanceMeters = distance.getValueAsDouble();
    inputs.isDetected = isDetected.getValue();
  }
}
