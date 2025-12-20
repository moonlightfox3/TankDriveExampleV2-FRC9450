// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.Arm;

/* Default command to move the arm with a joystick. */
public class MoveArmButtonCommand extends Command {
  private Arm m_arm;
  private CommandPS4Controller m_controller;
  
  private static final double SPEED = 2.0;

  /** Creates a new MoveArmJoystickCommand. */
  public MoveArmButtonCommand(Arm arm, CommandPS4Controller controller) {
    addRequirements(arm);
    m_arm = arm;
    m_controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean moveUp = m_controller.povUp().getAsBoolean();
    boolean moveDown = m_controller.povDown().getAsBoolean();
    if (moveUp) m_arm.setMotor(SPEED);
    else if (moveDown) m_arm.setMotor(-SPEED);
    else m_arm.setMotor(0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setMotor(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
