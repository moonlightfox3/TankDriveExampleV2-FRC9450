// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.Drivetrain;

/* Default command to move with a joystick. */
public class MoveJoystickCommand extends Command {
  private Drivetrain m_drivetrain;
  private CommandPS4Controller m_controller;

  private static final double DEADBAND_SIZE = 0.03;
  private static final double SPEED_MUL = 0.1;

  /** Creates a new MoveJoystickCommand. */
  public MoveJoystickCommand(Drivetrain drivetrain, CommandPS4Controller controller) {
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
    m_controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = -m_controller.getLeftY();
    if (Math.abs(speed) < DEADBAND_SIZE) {
      m_drivetrain.setAllMotors(0.0);
      return;
    }
    double lSpeed = speed; double rSpeed = speed;

    double rStick = m_controller.getRightX();
    lSpeed *= 1 - -Math.min(rStick, 0.0);
    rSpeed *= 1 -  Math.max(rStick, 0.0);

    m_drivetrain.setMotors(lSpeed * 12.0 * SPEED_MUL, rSpeed * 12.0 * SPEED_MUL);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setAllMotors(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
