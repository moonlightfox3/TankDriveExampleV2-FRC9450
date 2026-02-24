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
  private final int CAN_ID = power.getModule();
  private final ModuleType TYPE = power.getType();
  private final PowerDistributionVersion VERSION_NUMBERS = power.getVersion();

  /** Creates a new PowerSubsystem. */
  public PowerSubsystem() {
    Logger.recordOutput("TankDrive/PowerSubsystem/Info/CanId", getCanId());
    Logger.recordOutput("TankDrive/PowerSubsystem/Info/IsREV", isREV());
    Logger.recordOutput("TankDrive/PowerSubsystem/Info/NumChannels", getNumChannels());

    Logger.recordOutput("TankDrive/PowerSubsystem/Info/Version/FirmwareMajor", getVersion().firmwareMajor);
    Logger.recordOutput("TankDrive/PowerSubsystem/Info/Version/FirmwareMinor", getVersion().firmwareMinor);
    Logger.recordOutput("TankDrive/PowerSubsystem/Info/Version/FirmwareFix", getVersion().firmwareFix);
    Logger.recordOutput("TankDrive/PowerSubsystem/Info/Version/HardwareMajor", getVersion().hardwareMajor);
    Logger.recordOutput("TankDrive/PowerSubsystem/Info/Version/HardwareMinor", getVersion().hardwareMinor);
    Logger.recordOutput("TankDrive/PowerSubsystem/Info/Version/UniqueId", getVersion().uniqueId);
  }
  public static PowerSubsystem getInstance() {
    if (INSTANCE == null) INSTANCE = new PowerSubsystem();
    return INSTANCE;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("TankDrive/PowerSubsystem/BatteryVoltage", getBatteryVoltage());
    Logger.recordOutput("TankDrive/PowerSubsystem/ReportedTotalAmpPull", getReportedTotalCurrentAmpPull());
    Logger.recordOutput("TankDrive/PowerSubsystem/CalculatedTotalAmpPull", getCalculatedTotalCurrentAmpPull());
    for (int i = 0; i < getNumChannels(); i++) Logger.recordOutput("TankDrive/PowerSubsystem/AmpPull/" + i, getCurrentAmpPull(i));

    Logger.recordOutput("TankDrive/PowerSubsystem/CTRE/PowerTempC", getPowerTempCelsiusCTRE());
    Logger.recordOutput("TankDrive/PowerSubsystem/CTRE/PowerUsed", getTotalPowerUsedCTRE());
    Logger.recordOutput("TankDrive/PowerSubsystem/CTRE/EnergyUsedResettable", getTotalEnergyUsedCTRE());
  }

  public int getCanId() {
    return CAN_ID;
  }
  public PowerDistributionVersion getVersion() {
    return VERSION_NUMBERS;
  }
  public boolean isREV() {
    return TYPE == ModuleType.kRev;
  }
  public int getNumChannels() {
    if (isREV()) return 24;
    else return 16;
  }

  public double getBatteryVoltage() {
    return power.getVoltage();
  }
  public double getReportedTotalCurrentAmpPull() {
    return power.getTotalCurrent();
  }
  public double getCalculatedTotalCurrentAmpPull() {
    double sum = 0.0;
    for (int i = 0; i < getNumChannels(); i++) sum += getCurrentAmpPull(i);
    return sum;
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
