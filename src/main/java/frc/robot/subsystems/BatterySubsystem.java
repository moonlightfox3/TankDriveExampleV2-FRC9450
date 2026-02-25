// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.playingwithfusion.BattFuelGauge;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BatterySubsystem extends SubsystemBase {
  private static BatterySubsystem INSTANCE;

  private final BattFuelGauge BFG = new BattFuelGauge(80);

  /** Creates a new BatterySubsystem. */
  public BatterySubsystem() {
  }
  public static BatterySubsystem getInstance() {
    if (INSTANCE == null) INSTANCE = new BatterySubsystem();
    return INSTANCE;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("TankDrive/BatterySubsystem/Info/SerialNumber", BFG.getSerialNumber());
  }
}
