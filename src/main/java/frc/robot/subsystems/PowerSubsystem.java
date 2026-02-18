// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO: Put logs in coordsub of main robot
public class PowerSubsystem extends SubsystemBase { // TODO: Test
  private static PowerSubsystem INSTANCE;

  PowerDistribution power = new PowerDistribution(-1, ModuleType.kRev); // TODO: CAN ID (Not -1)

  /** Creates a new PowerSubsystem. */
  public PowerSubsystem() {
  }
  public static PowerSubsystem getInstance() {
    if (INSTANCE == null) INSTANCE = new PowerSubsystem();
    return INSTANCE;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("TankDrive/PowerSubsystem/BatteryVoltage", getBatteryVoltage());
    Logger.recordOutput("TankDrive/PowerSubsystem/TotalAmpPull", getTotalCurrentAmpPull());
    Logger.recordOutput("TankDrive/PowerSubsystem/PowerTempC", getPowerTempCelsiusCTRE());
  }

  public double getBatteryVoltage() {
    return power.getVoltage();
  }
  public double getTotalCurrentAmpPull() {
    return power.getTotalCurrent();
  }
  public double getCurrentAmpPull(int channel) {
    return power.getCurrent(channel);
  }

  /** CTRE only, always returns 0 for REV */
  public double getPowerTempCelsiusCTRE() {
    return power.getTemperature();
  }
}
