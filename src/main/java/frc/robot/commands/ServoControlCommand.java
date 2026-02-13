// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ServoSubsystem;

/* Command to control the servo. */
public class ServoControlCommand extends Command {
  private ServoSubsystem m_servo;
  private CommandXboxController m_controller;

  /** Creates a new ServoControlCommand. */
  public ServoControlCommand(ServoSubsystem servo, CommandXboxController controller) {
    addRequirements(servo);
    m_servo = servo;
    m_controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_servo.setTargetPos(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_controller.leftBumper().getAsBoolean()) m_servo.setTargetPos(m_controller.getLeftTriggerAxis());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_servo.setTargetPos(0.5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
