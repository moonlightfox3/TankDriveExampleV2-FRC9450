// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoForwardCommand;
import frc.robot.commands.MoveArmButtonCommand;
import frc.robot.commands.MoveArmToPositionCommand;
import frc.robot.commands.MoveJoystickCommand;
import frc.robot.commands.ZeroArmCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer { // TODO: Use LoggedNetworkNumbers for PID values?
  // The robot's subsystems and commands are defined here...
  public final Drivetrain m_drivetrain = Drivetrain.getInstance();
  public final Arm m_arm = Arm.getInstance();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final CommandPS4Controller m_driverController = new CommandPS4Controller(0);

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_drivetrain.setDefaultCommand(new MoveJoystickCommand(m_drivetrain, m_driverController));
    m_arm.setDefaultCommand(new MoveArmButtonCommand(m_arm, m_driverController));

    m_driverController.options().onTrue(new ZeroArmCommand(m_arm));

    m_driverController.circle().onTrue(new MoveArmToPositionCommand(m_arm, 2.0));
    m_driverController.triangle().onTrue(new MoveArmToPositionCommand(m_arm, 9.75));
    m_driverController.square().onTrue(new MoveArmToPositionCommand(m_arm, 17.5));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // A command that will be run in autonomous
    return new AutoForwardCommand(m_drivetrain);
  }
}
