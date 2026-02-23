// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.hal.PowerDistributionVersion;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerSubsystem extends SubsystemBase {
  private static PowerSubsystem INSTANCE;

  // Defaults to CAN ID 0 for CTRE, 1 for REV (should auto-detect type and id)
  private final PowerDistribution power = new PowerDistribution();
  public final int CAN_ID = power.getModule();
  public final ModuleType TYPE = power.getType();
  public final PowerDistributionVersion VERSION_NUMBERS = power.getVersion();

  /** Creates a new PowerSubsystem. */
  public PowerSubsystem() {
    Logger.recordOutput("TankDrive/PowerSubsystem/Info/CanId", CAN_ID);
    Logger.recordOutput("TankDrive/PowerSubsystem/Info/Type", TYPE);

    Logger.recordOutput("TankDrive/PowerSubsystem/Info/Version/FirmwareMajor", VERSION_NUMBERS.firmwareMajor);
    Logger.recordOutput("TankDrive/PowerSubsystem/Info/Version/FirmwareMinor", VERSION_NUMBERS.firmwareMinor);
    Logger.recordOutput("TankDrive/PowerSubsystem/Info/Version/FirmwareFix", VERSION_NUMBERS.firmwareFix);
    Logger.recordOutput("TankDrive/PowerSubsystem/Info/Version/HardwareMajor", VERSION_NUMBERS.hardwareMajor);
    Logger.recordOutput("TankDrive/PowerSubsystem/Info/Version/HardwareMinor", VERSION_NUMBERS.hardwareMinor);
    Logger.recordOutput("TankDrive/PowerSubsystem/Info/Version/UniqueId", VERSION_NUMBERS.uniqueId);
  }
  public static PowerSubsystem getInstance() {
    if (INSTANCE == null) INSTANCE = new PowerSubsystem();
    return INSTANCE;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("TankDrive/PowerSubsystem/BatteryVoltage", getBatteryVoltage());
    Logger.recordOutput("TankDrive/PowerSubsystem/TotalAmpPull", getTotalCurrentAmpPull());
    if (TYPE == ModuleType.kRev) {
      for (int i = 0; i <= 23; i++) Logger.recordOutput("TankDrive/PowerSubsystem/AmpPull/" + i, getCurrentAmpPull(i));
    } else if (TYPE == ModuleType.kCTRE) {
      for (int i = 0; i <= 15; i++) Logger.recordOutput("TankDrive/PowerSubsystem/AmpPull/" + i, getCurrentAmpPull(i));
    }

    Logger.recordOutput("TankDrive/PowerSubsystem/CTRE/PowerTempC", getPowerTempCelsiusCTRE());
    Logger.recordOutput("TankDrive/PowerSubsystem/CTRE/PowerUsed", getTotalPowerUsedCTRE());
    Logger.recordOutput("TankDrive/PowerSubsystem/CTRE/EnergyUsedResettable", getTotalEnergyUsedCTRE());
  }

  public double getBatteryVoltage() {
    return power.getVoltage();
  }
  public double getTotalCurrentAmpPull() {
    return power.getTotalCurrent();
  }

  /** REV: Channels 0-23, CTRE: Channels 0-15 */
  public double getCurrentAmpPull(int channel) {
    return power.getCurrent(channel);
  }

  /** CTRE only, always returns 0 for REV */
  public double getPowerTempCelsiusCTRE() {
    return power.getTemperature();
  }
  /** CTRE only, always returns 0 for REV */
  public double getTotalPowerUsedCTRE() {
    return power.getTotalPower();
  }
  /** CTRE only, always returns 0 for REV */
  public double getTotalEnergyUsedCTRE() {
    return power.getTotalEnergy();
  }
  /** CTRE only, does nothing for REV */
  public void resetTotalEnergyUsedCTRE() {
    power.resetTotalEnergy();
  }
}
