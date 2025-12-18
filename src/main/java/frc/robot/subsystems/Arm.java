// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private static Arm instance;

  private final TalonFX motor = new TalonFX(5, "CantDrive");

  /** Creates a new Arm. */
  public Arm() {
    resetDistance();
  }
  public static Arm getInstance() {
    if (instance == null) instance = new Arm();
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getPosition() {
    return motor.getPosition().getValueAsDouble();
  }
  public void resetDistance() {
    motor.setPosition(0.0);
  }

  public void setMotorRaw(double volts) { // TODO: test this, add pid
    Logger.recordOutput("TankDrive/ArmSubsystem/Speed/Target", volts);

    motor.setVoltage(volts);
  }
  // TODO: add method to move to position
}
