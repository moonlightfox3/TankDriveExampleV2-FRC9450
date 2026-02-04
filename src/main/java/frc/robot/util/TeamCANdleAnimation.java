// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

/* Displays a custom CANdle animation. */
public class TeamCANdleAnimation extends Command {
  private LEDSubsystem m_leds;

  public static final RGBWColor BLUE = RGBWColor.fromHSV(239.0, 1.0, 0.11);
  public static final RGBWColor ORANGE = RGBWColor.fromHSV(4.0, 1.0, 1.0);
  public static final RGBWColor BLUE_GRADIENT = new RGBWColor((BLUE.Red + ORANGE.Red) / 3, (BLUE.Green + ORANGE.Green) / 3, (BLUE.Blue + ORANGE.Blue) / 3);
  public static final RGBWColor ORANGE_GRADIENT = new RGBWColor((BLUE.Red + ORANGE.Red) / 3 * 2, (BLUE.Green + ORANGE.Green) / 3 * 2, (BLUE.Blue + ORANGE.Blue) / 3 * 2);

  /** Creates a new TeamCANdleAnimation. */
  public TeamCANdleAnimation(LEDSubsystem leds) {
    addRequirements(leds);
    m_leds = leds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
