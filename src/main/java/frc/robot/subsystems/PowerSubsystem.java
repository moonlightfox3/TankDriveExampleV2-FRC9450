// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

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
  
  private final LoggedNetworkBoolean logDebuggingToggle = new LoggedNetworkBoolean("Tuning/Debugging Toggles/Power", false);
  private final LoggedNetworkBoolean logDebuggingToggleSecondary = new LoggedNetworkBoolean("Tuning/Debugging Toggles/Power Secondary", false); // For less important values (like firmware versions)

  /** Creates a new PowerSubsystem. */
  public PowerSubsystem() {
  }
  public static PowerSubsystem getInstance() {
    if (INSTANCE == null) INSTANCE = new PowerSubsystem();
    return INSTANCE;
  }

  @Override
  public void periodic() {
    if (logDebuggingToggle.get()) {
      Logger.recordOutput("TankDrive/PowerSubsystem/BatteryVoltage", getBatteryVoltage());

      // Doesn't use the method to calculate total current draw because this reduces vendordep method calls (already getting all channels)
      double[] currents = getAllCurrentAmpPulls(); double currentsSum = 0.0;
      for (double current : currents) currentsSum += current;
      Logger.recordOutput("TankDrive/PowerSubsystem/AmpPulls", currents);
      Logger.recordOutput("TankDrive/PowerSubsystem/CalculatedTotalAmpPull", currentsSum);
    }
    if (logDebuggingToggleSecondary.get()) {
      Logger.recordOutput("TankDrive/PowerSubsystem/Info/CanId", getCanId());
      Logger.recordOutput("TankDrive/PowerSubsystem/Info/IsREV", isREV());
      Logger.recordOutput("TankDrive/PowerSubsystem/Info/NumChannels", getNumChannels());

      PowerDistributionVersion version = getVersion();
      Logger.recordOutput("TankDrive/PowerSubsystem/Info/Version/FirmwareMajor", version.firmwareMajor);
      Logger.recordOutput("TankDrive/PowerSubsystem/Info/Version/FirmwareMinor", version.firmwareMinor);
      Logger.recordOutput("TankDrive/PowerSubsystem/Info/Version/FirmwareFix", version.firmwareFix);
      Logger.recordOutput("TankDrive/PowerSubsystem/Info/Version/HardwareMajor", version.hardwareMajor);
      Logger.recordOutput("TankDrive/PowerSubsystem/Info/Version/HardwareMinor", version.hardwareMinor);
      Logger.recordOutput("TankDrive/PowerSubsystem/Info/Version/UniqueId", version.uniqueId);
    }
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
  /** Warning: The returned number is rounded to an integer. */
  public double getReportedTotalCurrentAmpPull() {
    return power.getTotalCurrent();
  }
  public double getCalculatedTotalCurrentAmpPull() {
    double[] currents = getAllCurrentAmpPulls(); double currentsSum = 0.0;
    for (double current : currents) currentsSum += current;
    return currentsSum;
  }

  /** REV: Channels 0-23, CTRE: Channels 0-15 */
  public double getCurrentAmpPull(int channel) {
    return power.getCurrent(channel);
  }
  /** REV: Channels 0-23, CTRE: Channels 0-15 */
  public double[] getAllCurrentAmpPulls() {
    return power.getAllCurrents();
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
