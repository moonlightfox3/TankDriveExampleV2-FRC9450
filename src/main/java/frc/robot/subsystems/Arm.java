// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private static Arm instance;

  private static final double CLOSE_LIMIT = 2.0;
  private static final double FAR_LIMIT = 17.5;
  
  private final PIDController speedPID = new PIDController(0.6, 0.0, 0.0);
  private final PIDController positionPID = new PIDController(0.7, 0.0, 0.03);
  
  private final LoggedNetworkNumber speedPIDLogP = new LoggedNetworkNumber("TankDrive/ArmSubsystem/PID/Speed/P", 0.0);
  private final LoggedNetworkNumber speedPIDLogI = new LoggedNetworkNumber("TankDrive/ArmSubsystem/PID/Speed/I", 0.0);
  private final LoggedNetworkNumber speedPIDLogD = new LoggedNetworkNumber("TankDrive/ArmSubsystem/PID/Speed/D", 0.0);
  private final LoggedNetworkNumber positionPIDLogP = new LoggedNetworkNumber("TankDrive/ArmSubsystem/PID/Position/P", 0.0);
  private final LoggedNetworkNumber positionPIDLogI = new LoggedNetworkNumber("TankDrive/ArmSubsystem/PID/Position/I", 0.0);
  private final LoggedNetworkNumber positionPIDLogD = new LoggedNetworkNumber("TankDrive/ArmSubsystem/PID/Position/D", 0.0);

  private final TalonFX motor = new TalonFX(5, "CantDrive");

  private double oldVolts = 0.0;

  /** Creates a new Arm. */
  public Arm() {
    speedPIDLogP.set(speedPID.getP());
    speedPIDLogI.set(speedPID.getI());
    speedPIDLogD.set(speedPID.getD());
    positionPIDLogP.set(positionPID.getP());
    positionPIDLogI.set(positionPID.getI());
    positionPIDLogD.set(positionPID.getD());

    resetPosition();
  }
  public static Arm getInstance() {
    if (instance == null) instance = new Arm();
    return instance;
  }

  @Override
  public void periodic() {
    double speedPValue = speedPIDLogP.get();
    if (speedPValue != speedPID.getP()) speedPID.setP(speedPValue);
    double speedIValue = speedPIDLogI.get();
    if (speedIValue != speedPID.getI()) speedPID.setI(speedIValue);
    double speedDValue = speedPIDLogD.get();
    if (speedDValue != speedPID.getD()) speedPID.setD(speedDValue);
    
    double positionPValue = positionPIDLogP.get();
    if (positionPValue != positionPID.getP()) positionPID.setP(positionPValue);
    double positionIValue = positionPIDLogI.get();
    if (positionIValue != positionPID.getI()) positionPID.setI(positionIValue);
    double positionDValue = positionPIDLogD.get();
    if (positionDValue != positionPID.getD()) positionPID.setD(positionDValue);
  }

  public double getPosition() {
    return motor.getPosition().getValueAsDouble();
  }
  public void resetPosition() {
    motor.setPosition(0.0);
  }

  public void setMotor(double volts) {
    double position = getPosition();
    if ((position <= CLOSE_LIMIT && volts < 0) || (position >= FAR_LIMIT && volts > 0)) volts = 0;
    double newVolts = oldVolts + speedPID.calculate(oldVolts - volts);

    Logger.recordOutput("TankDrive/ArmSubsystem/Speed/Input", oldVolts);
    Logger.recordOutput("TankDrive/ArmSubsystem/Speed/Target", volts);
    Logger.recordOutput("TankDrive/ArmSubsystem/Speed/Output", newVolts);

    motor.setVoltage(newVolts);
    oldVolts = newVolts;
  }
  public void goToPosition(double position) {
    if (position < CLOSE_LIMIT) position = CLOSE_LIMIT;
    if (position > FAR_LIMIT) position = FAR_LIMIT;

    double volts = positionPID.calculate(getPosition() - position);
    setMotor(volts);
  }
}
