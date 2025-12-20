// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private static Arm instance;

  private static final double CLOSE_LIMIT = 2.0;
  private static final double FAR_LIMIT = 17.5;
  
  private final PIDController speedPID = new PIDController(0.3, 0.0, 0.0);
  private final PIDController positionPID = new PIDController(0.8, 0.0, 0.0);

  private final TalonFX motor = new TalonFX(5, "CantDrive");

  private double oldVolts = 0.0;

  /** Creates a new Arm. */
  public Arm() {
    resetPosition();
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
