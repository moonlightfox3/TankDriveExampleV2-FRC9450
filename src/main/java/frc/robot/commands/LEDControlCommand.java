// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.util.TeamCANdleAnimation;

/* Default command to control the LEDs. */
public class LEDControlCommand extends Command {
  private LEDSubsystem m_leds;
  private CommandXboxController m_controller;

  private double hue = 0.0;
  private double saturation = 1.0;
  private double value = 1.0;
  private double brightness = 1.0;
  private int changeType = 0;
  private int endIndex = 7;

  private boolean rightPressed = false;
  private boolean leftPressed = false;

  private boolean useAnimation = false;
  private boolean animationStarted = false;
  private TeamCANdleAnimation animation;

  /** Creates a new LEDControlCommand. */
  public LEDControlCommand(LEDSubsystem leds, CommandXboxController controller) {
    addRequirements(leds);
    m_leds = leds;
    m_controller = controller;

    animation = new TeamCANdleAnimation(m_leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_leds.setRange(0, 7);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_controller.rightTrigger().getAsBoolean()) useAnimation = true;
    if (m_controller.leftTrigger().getAsBoolean()) useAnimation = false;

    if (useAnimation) {
      if (!animationStarted) animation.initialize();
      animationStarted = true;

      animation.execute();
      return;
    } else {
      if (animationStarted) animation.end();
      animationStarted = false;

      m_leds.setRange(0, endIndex);
    }

    if (m_controller.back().getAsBoolean()) {
      m_leds.setRange(8, LEDSubsystem.NUM_LEDS);
      m_leds.setColor(LEDSubsystem.EMPTY_COLOR);
      m_leds.applyControl();

      m_leds.setRange(0, 7);
      endIndex = 7;
      hue = 333; saturation = 1; value = 1; brightness = 0.05;
    }

    if (m_controller.rightBumper().getAsBoolean()) {
      if (!rightPressed) {
        rightPressed = true;
        if (endIndex < LEDSubsystem.NUM_LEDS) {
          if (m_controller.y().getAsBoolean()) endIndex++;
          else {
            endIndex += 5;
            if (endIndex > LEDSubsystem.NUM_LEDS) endIndex = LEDSubsystem.NUM_LEDS;
          }
          m_leds.setEnd(endIndex);
        }
      }
    } else rightPressed = false;
    if (m_controller.leftBumper().getAsBoolean()) {
      if (!leftPressed) {
        leftPressed = true;
        if (endIndex > 0) {
          if (m_controller.y().getAsBoolean()) m_leds.setStart(endIndex);
          else {
            int moveEndIndex = endIndex - 4;
            if (moveEndIndex < 1) moveEndIndex = 1;
            m_leds.setStart(moveEndIndex);
          }
          m_leds.setColor(LEDSubsystem.EMPTY_COLOR);
          m_leds.applyControl();
          m_leds.setStart(0);

          if (m_controller.y().getAsBoolean()) endIndex--;
          else {
            endIndex -= 5;
            if (endIndex < 0) endIndex = 0;
          }
          m_leds.setEnd(endIndex);
        }
      }
    } else leftPressed = false;

    if (m_controller.x().getAsBoolean()) changeType = 0;
    if (m_controller.a().getAsBoolean()) changeType = 1;
    if (m_controller.b().getAsBoolean()) changeType = 2;

    if (changeType == 0) {
      if (m_controller.y().getAsBoolean()) {
        if (m_controller.povUp().getAsBoolean()) hue += 0.1; if (hue > 360.0) hue = 0.0;
        if (m_controller.povDown().getAsBoolean()) hue -= 0.1; if (hue < 0.0) hue = 360.0;
      } else {
        if (m_controller.povUp().getAsBoolean()) hue++; if (hue > 360.0) hue = 0.0;
        if (m_controller.povDown().getAsBoolean()) hue--; if (hue < 0.0) hue = 360.0;
      }
    } else if (changeType == 1) {
      if (m_controller.y().getAsBoolean()) {
        if (m_controller.povUp().getAsBoolean()) saturation += 0.001; if (saturation > 1.0) saturation = 1.0;
        if (m_controller.povDown().getAsBoolean()) saturation -= 0.001; if (saturation < 0.0) saturation = 0.0;
      } else {
        if (m_controller.povUp().getAsBoolean()) saturation += 0.01; if (saturation > 1.0) saturation = 1.0;
        if (m_controller.povDown().getAsBoolean()) saturation -= 0.01; if (saturation < 0.0) saturation = 0.0;
      }
    } else if (changeType == 2) {
      if (m_controller.y().getAsBoolean()) {
        if (m_controller.povUp().getAsBoolean()) value += 0.001; if (value > 1.0) value = 1.0;
        if (m_controller.povDown().getAsBoolean()) value -= 0.001; if (value < 0.0) value = 0.0;
      } else {
        if (m_controller.povUp().getAsBoolean()) value += 0.01; if (value > 1.0) value = 1.0;
        if (m_controller.povDown().getAsBoolean()) value -= 0.01; if (value < 0.0) value = 0.0;
      }
    }

    if (m_controller.y().getAsBoolean()) {
      if (m_controller.povRight().getAsBoolean()) brightness += 0.001; if (brightness > 1.0) brightness = 1.0;
      if (m_controller.povLeft().getAsBoolean()) brightness -= 0.001; if (brightness < 0.0) brightness = 0.0;
    } else {
      if (m_controller.povRight().getAsBoolean()) brightness += 0.01; if (brightness > 1.0) brightness = 1.0;
      if (m_controller.povLeft().getAsBoolean()) brightness -= 0.01; if (brightness < 0.0) brightness = 0.0;
    }

    if (m_controller.povUp().getAsBoolean() || m_controller.povDown().getAsBoolean() || m_controller.povRight().getAsBoolean() || m_controller.povLeft().getAsBoolean()) System.out.println("H: " + hue + ", S: " + saturation + ", V: " + value + ", B: " + brightness);
    m_leds.setColor(hue, saturation, value, brightness);
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
