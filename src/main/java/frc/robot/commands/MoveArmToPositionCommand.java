// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

/* Moves the arm to a position. */
public class MoveArmToPositionCommand extends Command {
  private Arm m_arm;

  private static final double END_DISTANCE = 0.3;

  private double position;

  /** Creates a new MoveArmToPositionCommand. */
  public MoveArmToPositionCommand(Arm arm, double position) {
    addRequirements(arm);
    m_arm = arm;
    this.position = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.goToPosition(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setMotor(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_arm.getPosition() - position) <= END_DISTANCE;
  }
}
