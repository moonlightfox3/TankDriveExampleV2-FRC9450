// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

/* Moves the robot forward for a distance. */
public class AutoForwardCommand extends Command {
  private DrivetrainSubsystem m_drivetrain;

  private static final double MOVE_VOLTS = 1.2;
  private static final double MOVE_DIST_FEET = 2.0;
  private static final double END_OFFSET_FEET = 1.0 / 2.0;

  /** Creates a new AutoForwardCommand. */
  public AutoForwardCommand(DrivetrainSubsystem drivetrain) {
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.resetAllDistance();
    m_drivetrain.setAllMotors(MOVE_VOLTS);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setAllMotors(0.0);
    m_drivetrain.resetAllDistance();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return MOVE_DIST_FEET - (m_drivetrain.getAverageDistanceInch() / 12.0) <= END_OFFSET_FEET;
  }
}
