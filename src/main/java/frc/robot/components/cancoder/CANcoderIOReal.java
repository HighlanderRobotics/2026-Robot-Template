// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components.cancoder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

public class CANcoderIOReal implements CANcoderIO {
  private final CANcoder cancoder;

  private final StatusSignal<Angle> cancoderAbsolutePositionRotations;

  public CANcoderIOReal(int cancoderID, CANcoderConfiguration config) {
    cancoder = new CANcoder(cancoderID, "*");
    cancoderAbsolutePositionRotations = cancoder.getAbsolutePosition();
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, cancoderAbsolutePositionRotations);
    cancoder.getConfigurator().apply(config);
    cancoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(CANcoderIOInputs inputs) {
    BaseStatusSignal.refreshAll(cancoderAbsolutePositionRotations);

    inputs.cancoderPositionRotations =
        Rotation2d.fromRotations(cancoderAbsolutePositionRotations.getValueAsDouble());
  }
}
