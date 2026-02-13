// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoSubsystem extends SubsystemBase {
  private static ServoSubsystem INSTANCE;

  private Servo servo = new Servo(0);
  
  /** Creates a new ServoSubsystem. */
  public ServoSubsystem() {
    setTargetPos(0.5);
  }
  public static ServoSubsystem getInstance() {
    if (INSTANCE == null) INSTANCE = new ServoSubsystem();
    return INSTANCE;
  }

  @Override
  public void periodic() {    
    Logger.recordOutput("TankDrive/ServoSubsystem/TargetPos", getTargetPos());
    Logger.recordOutput("TankDrive/ServoSubsystem/TargetAngle", getTargetAngle());
  }

  public void setTargetPos(double value) {
    servo.set(value);
  }
  public void setTargetAngle(double degrees) {
    servo.setAngle(degrees);
  }
  public double getTargetPos() {
    return servo.get();
  }
  public double getTargetAngle() {
    return servo.getAngle();
  }
}
