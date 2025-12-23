// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private static ArmSubsystem instance;

  private static final double CLOSE_LIMIT = 2.0;
  private static final double FAR_LIMIT = 17.5;
  
  private final PIDController speedPID = new PIDController(1.0, 0.0, 0.0);
  
  private final LoggedNetworkNumber speedPIDLogP = new LoggedNetworkNumber("TankDrive/ArmSubsystem/Speed/PID/P", 0.0);
  private final LoggedNetworkNumber speedPIDLogI = new LoggedNetworkNumber("TankDrive/ArmSubsystem/Speed/PID/I", 0.0);
  private final LoggedNetworkNumber speedPIDLogD = new LoggedNetworkNumber("TankDrive/ArmSubsystem/Speed/PID/D", 0.0);

  private final TalonFX motor = new TalonFX(5, "CantDrive");
  private final MotionMagicVoltage request = new MotionMagicVoltage(0.0).withSlot(0);
  private final TalonFXConfiguration motorConfig;
  private final Slot0Configs slot0;
  private double setpoint = 2.0;
  private double oldVolts = 0.0;
  public boolean usingMotionMagic = true;

  private final LoggedNetworkNumber requestLogKS = new LoggedNetworkNumber("TankDrive/ArmSubsystem/Speed/MotionMagic/kS", 0.0);
  private final LoggedNetworkNumber requestLogKV = new LoggedNetworkNumber("TankDrive/ArmSubsystem/Speed/MotionMagic/kV", 0.0);
  private final LoggedNetworkNumber requestLogKA = new LoggedNetworkNumber("TankDrive/ArmSubsystem/Speed/MotionMagic/kA", 0.0);
  private final LoggedNetworkNumber requestLogKP = new LoggedNetworkNumber("TankDrive/ArmSubsystem/Speed/MotionMagic/kP", 0.0);
  private final LoggedNetworkNumber requestLogKI = new LoggedNetworkNumber("TankDrive/ArmSubsystem/Speed/MotionMagic/kI", 0.0);
  private final LoggedNetworkNumber requestLogKD = new LoggedNetworkNumber("TankDrive/ArmSubsystem/Speed/MotionMagic/kD", 0.0);
  private final LoggedNetworkNumber requestLogKG = new LoggedNetworkNumber("TankDrive/ArmSubsystem/Speed/MotionMagic/kG", 0.0);
  private final LoggedNetworkNumber requestLogVel = new LoggedNetworkNumber("TankDrive/ArmSubsystem/Speed/MotionMagic/Velocity", 0.0);
  private final LoggedNetworkNumber requestLogAcc = new LoggedNetworkNumber("TankDrive/ArmSubsystem/Speed/MotionMagic/Acceleration", 0.0);
  private final LoggedNetworkNumber requestLogJerk = new LoggedNetworkNumber("TankDrive/ArmSubsystem/Speed/MotionMagic/Jerk", 0.0);

  /** Creates a new Arm. */
  public ArmSubsystem() {
    motorConfig = new TalonFXConfiguration();
    slot0 = motorConfig.Slot0;
    slot0.kS = 0.1; // Overcome static friction
    slot0.kV = 0.1; // Target velocity
    slot0.kA = 0.0; // Target acceleration
    slot0.kP = 8.0; // Error in position
    slot0.kI = 0.0; // Integrated error in position
    slot0.kD = 0.0; // Error in velocity
    slot0.kG = 0.2; // Overcome gravity

    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    motorConfig.Slot0 = slot0;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = 40.0;
    motorConfig.MotionMagic.MotionMagicAcceleration = 100.0;
    motorConfig.MotionMagic.MotionMagicJerk = 1000.0;
    applyConfig();
    
    speedPIDLogP.set(speedPID.getP());
    speedPIDLogI.set(speedPID.getI());
    speedPIDLogD.set(speedPID.getD());

    requestLogKS.set(slot0.kS);
    requestLogKV.set(slot0.kV);
    requestLogKA.set(slot0.kA);
    requestLogKP.set(slot0.kP);
    requestLogKI.set(slot0.kI);
    requestLogKD.set(slot0.kD);
    requestLogKG.set(slot0.kG);
    requestLogVel.set(motorConfig.MotionMagic.MotionMagicCruiseVelocity);
    requestLogAcc.set(motorConfig.MotionMagic.MotionMagicAcceleration);
    requestLogJerk.set(motorConfig.MotionMagic.MotionMagicJerk);

    resetPosition();
  }
  public static ArmSubsystem getInstance() {
    if (instance == null) instance = new ArmSubsystem();
    return instance;
  }

  private void applyConfig() {
    motor.getConfigurator().apply(motorConfig);
  }

  @Override
  public void periodic() {
    double speedPValue = speedPIDLogP.get();
    if (speedPValue != speedPID.getP()) speedPID.setP(speedPValue);
    double speedIValue = speedPIDLogI.get();
    if (speedIValue != speedPID.getI()) speedPID.setI(speedIValue);
    double speedDValue = speedPIDLogD.get();
    if (speedDValue != speedPID.getD()) speedPID.setD(speedDValue);

    double requestKSValue = requestLogKS.get();
    if (requestKSValue != slot0.kS) { slot0.kS = requestKSValue; applyConfig(); }
    double requestKVValue = requestLogKV.get();
    if (requestKVValue != slot0.kV) { slot0.kV = requestKVValue; applyConfig(); }
    double requestKAValue = requestLogKA.get();
    if (requestKAValue != slot0.kA) { slot0.kA = requestKAValue; applyConfig(); }
    double requestKPValue = requestLogKP.get();
    if (requestKPValue != slot0.kP) { slot0.kP = requestKPValue; applyConfig(); }
    double requestKIValue = requestLogKI.get();
    if (requestKIValue != slot0.kI) { slot0.kI = requestKIValue; applyConfig(); }
    double requestKDValue = requestLogKD.get();
    if (requestKDValue != slot0.kD) { slot0.kD = requestKDValue; applyConfig(); }
    double requestKGValue = requestLogKG.get();
    if (requestKGValue != slot0.kG) { slot0.kG = requestKGValue; applyConfig(); }
    double requestVelValue = requestLogVel.get();
    if (requestVelValue != motorConfig.MotionMagic.MotionMagicCruiseVelocity) { motorConfig.MotionMagic.MotionMagicCruiseVelocity = requestVelValue; applyConfig(); }
    double requestAccValue = requestLogAcc.get();
    if (requestAccValue != motorConfig.MotionMagic.MotionMagicAcceleration) { motorConfig.MotionMagic.MotionMagicAcceleration = requestAccValue; applyConfig(); }
    double requestJerkValue = requestLogJerk.get();
    if (requestJerkValue != motorConfig.MotionMagic.MotionMagicJerk) { motorConfig.MotionMagic.MotionMagicJerk = requestJerkValue; applyConfig(); }

    Logger.recordOutput("TankDrive/ArmSubsystem/MotionMagic/Enabled", usingMotionMagic);
    Logger.recordOutput("TankDrive/ArmSubsystem/MotionMagic/Setpoint", setpoint);
    if (usingMotionMagic) motor.setControl(request.withPosition(setpoint));
    Logger.recordOutput("TankDrive/ArmSubsystem/MotionMagic/ActualSpeed", motor.getMotorVoltage().getValueAsDouble());
  }

  public double getPosition() {
    return motor.getPosition().getValueAsDouble();
  }
  public void resetPosition() {
    motor.setPosition(0.0);
  }

  public void setSetpoint(double position) {
    if (!usingMotionMagic) return;

    if (position < CLOSE_LIMIT) position = CLOSE_LIMIT;
    if (position > FAR_LIMIT) position = FAR_LIMIT;
    
    setpoint = position;
  }
  public void setMotor(double volts) {
    if (usingMotionMagic) return;
    
    double position = getPosition();
    if ((position <= CLOSE_LIMIT && volts < 0) || (position >= FAR_LIMIT && volts > 0)) volts = 0;
    double newVolts = oldVolts + speedPID.calculate(oldVolts - volts);

    Logger.recordOutput("TankDrive/ArmSubsystem/Speed/Input", oldVolts);
    Logger.recordOutput("TankDrive/ArmSubsystem/Speed/Target", volts);
    Logger.recordOutput("TankDrive/ArmSubsystem/Speed/Output", newVolts);

    motor.setVoltage(newVolts);
    oldVolts = newVolts;
  }
}
