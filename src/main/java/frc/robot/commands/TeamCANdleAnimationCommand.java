// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

/* Displays a custom CANdle animation. */
public class TeamCANdleAnimationCommand extends Command {
  // Subsystems
  private LEDSubsystem m_leds;

  // Config
  public static final double FRAMERATE = 1.0;//6.0;
  private final double BASE_BRIGHTNESS = 1.0;

  // Color data
  public static final double[] BLUE_HSV = {240.0, 1.0, 0.16}; // RGB 0, 0, 40
  public static final double[] ORANGE_HSV = {4.0, 1.0, 1.0}; // RGB 255, 17, 0

  // Computed colors
  public static final RGBWColor BLUE = RGBWColor.fromHSV(BLUE_HSV[0], BLUE_HSV[1], BLUE_HSV[2]);
  public static final RGBWColor ORANGE = RGBWColor.fromHSV(ORANGE_HSV[0], ORANGE_HSV[1], ORANGE_HSV[2]);
  // public static final RGBWColor BLUE_GRADIENT = new RGBWColor(0, 26, 165);
  // public static final RGBWColor ORANGE_GRADIENT = new RGBWColor(255, 41, 0);

  // Color arrays
  private ArrayList<RGBWColor> COLORS = new ArrayList<>();
  private ArrayList<RGBWColor> COLORS_CANDLE = new ArrayList<>();

  // Vars
  private final Timer timer = new Timer();
  private boolean hasStarted = false;
  private int offset = 0;
  private int offsetCandle = 0;

  /** Creates a new TeamCANdleAnimation. */
  public TeamCANdleAnimationCommand(LEDSubsystem leds) {
    addRequirements(leds);
    m_leds = leds;

    COLORS.add(BLUE);
    addBlendedColorsToArray(COLORS, BLUE, new RGBWColor(10, 20, 20), 3);
    addBlendedColorsToArray(COLORS, new RGBWColor(35, 15, 2), ORANGE, 3);
    COLORS.add(ORANGE);
    
    int sizeCOLORS = COLORS.size();
    for (int i = sizeCOLORS - 2; i >= 1; i--) COLORS.add(COLORS.get(i));

    COLORS_CANDLE.add(BLUE);
    COLORS_CANDLE.add(BLUE);
    COLORS_CANDLE.add(BLUE);
    COLORS_CANDLE.add(BLUE);
    COLORS_CANDLE.add(ORANGE);
    COLORS_CANDLE.add(ORANGE);
    COLORS_CANDLE.add(ORANGE);
    COLORS_CANDLE.add(ORANGE);

    for (int i = 0; i < COLORS.size() / 2; i++) {
      RGBWColor temp = COLORS.get(i);
      COLORS.set(i, COLORS.get(COLORS.size() - 1 - i));
      COLORS.set(COLORS.size() - 1 - i, temp);
    }
    for (int i = 0; i < COLORS_CANDLE.size() / 2; i++) {
      RGBWColor temp = COLORS_CANDLE.get(i);
      COLORS_CANDLE.set(i, COLORS_CANDLE.get(COLORS_CANDLE.size() - 1 - i));
      COLORS_CANDLE.set(COLORS_CANDLE.size() - 1 - i, temp);
    }

    for (int i = 0; i < COLORS.size(); i++) COLORS.set(i, COLORS.get(i).scaleBrightness(BASE_BRIGHTNESS));
    for (int i = 0; i < COLORS_CANDLE.size(); i++) COLORS_CANDLE.set(i, COLORS_CANDLE.get(i).scaleBrightness(BASE_BRIGHTNESS));
  }
  private void addBlendedColorsToArray(ArrayList<RGBWColor> arr, RGBWColor color1, RGBWColor color2, int totalCount) {
    double posAdd = 1.0 / (totalCount + 1);
    for (int i = 0; i < totalCount; i++) {
      double t = posAdd * (i + 1);
      arr.add(new RGBWColor(
        (int) ((1 - t) * color1.Red + t * color2.Red),
        (int) ((1 - t) * color1.Green + t * color2.Green),
        (int) ((1 - t) * color1.Blue + t * color2.Blue)
      ));
    }
  }

  // Called when the command is initially scheduled.
  @Override
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
  @Override
  public void execute() {
    if (timer.advanceIfElapsed(1.0 / FRAMERATE) || !hasStarted) {
      hasStarted = true;

      int ledOffset = COLORS.size() - ((LEDSubsystem.HIGHEST_INDEX + 1 - 8) % COLORS.size());
      for (int led = LEDSubsystem.HIGHEST_INDEX; led >= 8; led--) {
        int colorIndex = (offset + ledOffset) % COLORS.size();
        ledOffset++;

        m_leds.setRange(led, led);
        m_leds.setColor(COLORS.get(colorIndex));
        m_leds.applyControl();
      }
      offset = (offset + 1) % COLORS.size();

      int ledOffsetCandle = COLORS_CANDLE.size() - (8 % COLORS_CANDLE.size());
      for (int led = 7; led >= 0; led--) {
        int colorIndexCandle = (offsetCandle + ledOffsetCandle) % COLORS_CANDLE.size();
        ledOffsetCandle++;

        m_leds.setRange(led, led);
        m_leds.setColor(COLORS_CANDLE.get(colorIndexCandle));
        m_leds.applyControl();
      }
      offsetCandle = (offsetCandle + 1) % COLORS_CANDLE.size();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    m_leds.setRange(0, LEDSubsystem.HIGHEST_INDEX);
    m_leds.setColor(LEDSubsystem.EMPTY_COLOR);
    m_leds.applyControl();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
