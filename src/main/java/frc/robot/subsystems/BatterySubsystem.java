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
  public void periodic() { // No indent = Not shown on BFG screen. One indent = Partially shown on BFG screen. Two indents = Shown on BFG screen.
        Logger.recordOutput("TankDrive/BatterySubsystem/Nickname", BFG.getNickname());
    Logger.recordOutput("TankDrive/BatterySubsystem/SerialNumber", BFG.getSerialNumber());
    Logger.recordOutput("TankDrive/BatterySubsystem/FirmwareVersion", BFG.getFirmwareVersion());
        Logger.recordOutput("TankDrive/BatterySubsystem/Manufacturer", BFG.getManufacturer());
        float capacityAh = BFG.getCapacityAh();
        Logger.recordOutput("TankDrive/BatterySubsystem/Capacity_Ah", capacityAh);
        Logger.recordOutput("TankDrive/BatterySubsystem/RatedCapacity_Ah", BFG.getRatedCapacityAh());
        Logger.recordOutput("TankDrive/BatterySubsystem/Age_ShouldBeDaysButIsntIdkWhy", BFG.getBatteryAgeDays());
        Logger.recordOutput("TankDrive/BatterySubsystem/NumCycles", BFG.getNumCycles());

        Logger.recordOutput("TankDrive/BatterySubsystem/Voltage", BFG.getVoltage());
        Logger.recordOutput("TankDrive/BatterySubsystem/Current", BFG.getCurrent());
      Logger.recordOutput("TankDrive/BatterySubsystem/State", BFG.getBatteryState());
        float effectiveCapacityAh = BFG.getEffectiveCapacityAh();
        Logger.recordOutput("TankDrive/BatterySubsystem/EffectiveCapacity_Ah", effectiveCapacityAh);
      float effectiveCapacityPct = effectiveCapacityAh / capacityAh * 100.0F;
      Logger.recordOutput("TankDrive/BatterySubsystem/EffectiveCapacity_Pct", effectiveCapacityPct);
    float depthOfDischargeAh = BFG.getDodAh();
    Logger.recordOutput("TankDrive/BatterySubsystem/DepthOfDischarge_Ah", depthOfDischargeAh);
    Logger.recordOutput("TankDrive/BatterySubsystem/DepthOfDischarge_Wh", BFG.getDodWh());
    float stateOfChargeAh = effectiveCapacityAh - depthOfDischargeAh;
    Logger.recordOutput("TankDrive/BatterySubsystem/StateOfCharge_Ah", stateOfChargeAh);
        float remainingCharge_Ah = stateOfChargeAh * effectiveCapacityAh; // TODO: Test (Also is this a duplicate of another value?)
        Logger.recordOutput("TankDrive/BatterySubsystem/RemainingCharge_Ah", remainingCharge_Ah); // TODO: (Above)
    Logger.recordOutput("TankDrive/BatterySubsystem/RemainingCharge_Pct", BFG.getRemainingChargePct()); // TODO: How does this compare to the remaining charge in amp-hours?


        Logger.recordOutput("TankDrive/BatterySubsystem/CurrentCycle/MinVoltage", BFG.getCycleMinVoltage());
    Logger.recordOutput("TankDrive/BatterySubsystem/CurrentCycle/MaxVoltage", BFG.getCycleMaxVoltage());
    Logger.recordOutput("TankDrive/BatterySubsystem/CurrentCycle/MinCurrent", BFG.getCycleMinCurrent());
        Logger.recordOutput("TankDrive/BatterySubsystem/CurrentCycle/MaxCurrent", BFG.getCycleMaxCurrent());
    Logger.recordOutput("TankDrive/BatterySubsystem/CurrentCycle/StartingVoltage", BFG.getCycleStartingVoltage());
        Logger.recordOutput("TankDrive/BatterySubsystem/CurrentCycle/RmsDischargeCurrent", BFG.getCycleRmsDischargeCurrent());
    Logger.recordOutput("TankDrive/BatterySubsystem/CurrentCycle/MinDepthOfDischarge_Ah", BFG.getCycleMinDodAh());
    Logger.recordOutput("TankDrive/BatterySubsystem/CurrentCycle/MinDepthOfDischarge_Wh", BFG.getCycleMinDodWh());
    Logger.recordOutput("TankDrive/BatterySubsystem/CurrentCycle/MaxDepthOfDischarge_Ah", BFG.getCycleMaxDodAh());
    Logger.recordOutput("TankDrive/BatterySubsystem/CurrentCycle/MaxDepthOfDischarge_Wh", BFG.getCycleMaxDodWh());
      Logger.recordOutput("TankDrive/BatterySubsystem/CurrentCycle/ChargeTime_S", BFG.getCycleChargeTime());
      Logger.recordOutput("TankDrive/BatterySubsystem/CurrentCycle/DischargeTime_S", BFG.getCycleDischargeTime());


        Logger.recordOutput("TankDrive/BatterySubsystem/LastMatch/MinVoltage", BFG.getMatchMinVoltage());
    Logger.recordOutput("TankDrive/BatterySubsystem/LastMatch/MaxVoltage", BFG.getMatchMaxVoltage());
    Logger.recordOutput("TankDrive/BatterySubsystem/LastMatch/MinCurrent", BFG.getMatchMinCurrent());
        Logger.recordOutput("TankDrive/BatterySubsystem/LastMatch/MaxCurrent", BFG.getMatchMaxCurrent());
        Logger.recordOutput("TankDrive/BatterySubsystem/LastMatch/StartingVoltage", BFG.getMatchStartingVoltage());
        Logger.recordOutput("TankDrive/BatterySubsystem/LastMatch/RmsDischargeCurrent", BFG.getMatchRmsDischargeCurrent());
    Logger.recordOutput("TankDrive/BatterySubsystem/LastMatch/MinDepthOfDischarge_Ah", BFG.getMatchMinDodAh());
    Logger.recordOutput("TankDrive/BatterySubsystem/LastMatch/MaxDepthOfDischarge_Ah", BFG.getMatchMaxDodAh());
    Logger.recordOutput("TankDrive/BatterySubsystem/LastMatch/ChargeTime_S", BFG.getMatchChargeTime());
    Logger.recordOutput("TankDrive/BatterySubsystem/LastMatch/DischargeTime_S", BFG.getMatchDischargeTime());
        Logger.recordOutput("TankDrive/BatterySubsystem/LastMatch/Duration_S", BFG.getMatchDuration());
    Logger.recordOutput("TankDrive/BatterySubsystem/LastMatch/StartingCharge_Ah", BFG.getMatchStartingCharge());
    Logger.recordOutput("TankDrive/BatterySubsystem/LastMatch/EndingCharge_Ah", BFG.getMatchEndingCharge());
    Logger.recordOutput("TankDrive/BatterySubsystem/LastMatch/StartingEnergy_Wh", BFG.getMatchStartingEnergy());
    Logger.recordOutput("TankDrive/BatterySubsystem/LastMatch/EndingEnergy_Wh", BFG.getMatchEndingEnergy());
  }
}
