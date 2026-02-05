// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.LEDSubsystem;

/* Displays a custom CANdle animation. */
public class TeamCANdleAnimation {
  private LEDSubsystem m_leds;

  public static final double FRAMERATE = 6.0;
  private final double BRIGHTNESS = 1.0;

  public static final RGBWColor BLUE = new RGBWColor(0, 0, 40);
  public static final RGBWColor ORANGE = new RGBWColor(255, 17, 0);
  public static final RGBWColor BLUE_GRADIENT = new RGBWColor(0, 26, 165);
  public static final RGBWColor ORANGE_GRADIENT = new RGBWColor(255, 41, 0);
  private RGBWColor[] COLORS = new RGBWColor[6];
  private RGBWColor[] COLORS_CANDLE = new RGBWColor[8];

  private final Timer timer = new Timer();
  private boolean hasStarted = false;
  private int offset = 0;
  private int offsetCandle = 0;

  /** Creates a new TeamCANdleAnimation. */
  public TeamCANdleAnimation(LEDSubsystem leds) {
    m_leds = leds;

    // Intentionally reversed indexes
    COLORS[5] = BLUE.scaleBrightness(BRIGHTNESS);
    COLORS[4] = BLUE_GRADIENT.scaleBrightness(BRIGHTNESS);
    COLORS[3] = ORANGE_GRADIENT.scaleBrightness(BRIGHTNESS);
    COLORS[2] = ORANGE.scaleBrightness(BRIGHTNESS);
    COLORS[1] = ORANGE_GRADIENT.scaleBrightness(BRIGHTNESS);
    COLORS[0] = BLUE_GRADIENT.scaleBrightness(BRIGHTNESS);

    // Intentionally reversed indexes
    COLORS_CANDLE[7] = BLUE.scaleBrightness(BRIGHTNESS);
    COLORS_CANDLE[6] = BLUE.scaleBrightness(BRIGHTNESS);
    COLORS_CANDLE[5] = BLUE.scaleBrightness(BRIGHTNESS);
    COLORS_CANDLE[4] = BLUE.scaleBrightness(BRIGHTNESS);
    COLORS_CANDLE[3] = ORANGE.scaleBrightness(BRIGHTNESS);
    COLORS_CANDLE[2] = ORANGE.scaleBrightness(BRIGHTNESS);
    COLORS_CANDLE[1] = ORANGE.scaleBrightness(BRIGHTNESS);
    COLORS_CANDLE[0] = ORANGE.scaleBrightness(BRIGHTNESS);
  }

  // Called when the command is initially scheduled.
  public void initialize() {
    m_leds.setRange(0, LEDSubsystem.HIGHEST_INDEX);
    m_leds.setColor(LEDSubsystem.EMPTY_COLOR);
    m_leds.applyControl();

    hasStarted = false;
    offset = 0;
    offsetCandle = 0;
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    if (timer.advanceIfElapsed(1.0 / FRAMERATE) || !hasStarted) {
      hasStarted = true;

      int ledOffset = COLORS.length - ((LEDSubsystem.HIGHEST_INDEX + 1 - 8) % COLORS.length);
      for (int led = LEDSubsystem.HIGHEST_INDEX; led >= 8; led--) {
        int colorIndex = (offset + ledOffset) % COLORS.length;
        ledOffset++;

        m_leds.setRange(led, led);
        m_leds.setColor(COLORS[colorIndex]);
        m_leds.applyControl();
      }
      offset = (offset + 1) % COLORS.length;

      int ledOffsetCandle = COLORS_CANDLE.length - (8 % COLORS_CANDLE.length);
      for (int led = 7; led >= 0; led--) {
        int colorIndexCandle = (offsetCandle + ledOffsetCandle) % COLORS_CANDLE.length;
        ledOffsetCandle++;

        m_leds.setRange(led, led);
        m_leds.setColor(COLORS_CANDLE[colorIndexCandle]);
        m_leds.applyControl();
      }
      offsetCandle = (offsetCandle + 1) % COLORS_CANDLE.length;
    }
  }

  // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {
    timer.stop();
    m_leds.setRange(0, LEDSubsystem.HIGHEST_INDEX);
    m_leds.setColor(LEDSubsystem.EMPTY_COLOR);
    m_leds.applyControl();
  }

  // Returns true when the command should end.
  public boolean isFinished() {
    return false;
  }
}
