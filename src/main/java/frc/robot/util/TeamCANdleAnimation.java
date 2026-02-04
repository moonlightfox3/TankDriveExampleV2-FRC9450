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

  private static final double BLUE_H = 239.0;
  private static final double BLUE_S = 1.0;
  private static final double BLUE_V = 0.11;
  private static final double BLUE_B = 1.0;
  private static final double ORANGE_H = 4.0;
  private static final double ORANGE_S = 1.0;
  private static final double ORANGE_V = 1.0;
  private static final double ORANGE_B = 1.0;
  private static double BLUEGRADIENT_H;
  private static double BLUEGRADIENT_S;
  private static double BLUEGRADIENT_V;
  private static double BLUEGRADIENT_B;
  private static double ORANGEGRADIENT_H;
  private static double ORANGEGRADIENT_S;
  private static double ORANGEGRADIENT_V;
  private static double ORANGEGRADIENT_B;

  /** Creates a new TeamCANdleAnimation. */
  public TeamCANdleAnimation(LEDSubsystem leds) {
    addRequirements(leds);
    m_leds = leds;

    RGBWColor blue = RGBWColor.fromHSV(BLUE_H, BLUE_S, BLUE_V).scaleBrightness(BLUE_B);
    RGBWColor orange = RGBWColor.fromHSV(ORANGE_H, ORANGE_S, ORANGE_V).scaleBrightness(ORANGE_B);
    // TODO: Gradient colors (below) - separate (2 between colors)
    RGBWColor blueGradient = new RGBWColor((blue.Red + orange.Red) / 2, (blue.Green + orange.Green) / 2, (blue.Blue + orange.Blue) / 2);
    RGBWColor orangeGradient = new RGBWColor((blue.Red + orange.Red) / 2, (blue.Green + orange.Green) / 2, (blue.Blue + orange.Blue) / 2);
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
