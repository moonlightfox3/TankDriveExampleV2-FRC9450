// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private static Drivetrain instance = null;

  public static final double WHEEL_DIAMETER_INCH = 6;
  public static final double WHEEL_CIRCUMFERENCE_INCH = Math.PI * WHEEL_DIAMETER_INCH;
  private static final double POS_CONVERT_FACTOR = 1.0 / 8.0; // Gearbox

  private final PIDController speedPID = new PIDController(0.2, 0, 0);

  private final SparkMaxConfig config = new SparkMaxConfig();
  private final SparkMax leftFrontMotor = new SparkMax(1, MotorType.kBrushless);
  private final SparkMax leftBackMotor = new SparkMax(4, MotorType.kBrushless);
  private final SparkMax rightFrontMotor = new SparkMax(2, MotorType.kBrushless);
  private final SparkMax rightBackMotor = new SparkMax(3, MotorType.kBrushless);

  private final RelativeEncoder leftFrontEncoder = leftFrontMotor.getEncoder();
  private final RelativeEncoder rightFrontEncoder = rightFrontMotor.getEncoder();

  private double leftVolts = 0.0;
  private double rightVolts = 0.0;

  /** Creates a new Drivetrain. */
  private Drivetrain() {
    config.encoder.positionConversionFactor(POS_CONVERT_FACTOR);

    config.inverted(false);
    leftFrontMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftBackMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.inverted(true);
    rightFrontMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightBackMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  public static Drivetrain getInstance() {
    if (instance == null) instance = new Drivetrain();
    return instance;
  }

  public double getLeftDistanceInch() {
    return leftFrontEncoder.getPosition() * WHEEL_CIRCUMFERENCE_INCH;
  }
  public double getRightDistanceInch() {
    return rightFrontEncoder.getPosition() * WHEEL_CIRCUMFERENCE_INCH;
  }
  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }
  public void resetLeftDistance() {
    leftFrontEncoder.setPosition(0.0);
  }
  public void resetRightDistance() {
    rightFrontEncoder.setPosition(0.0);
  }
  public void resetAllDistance() {
    resetLeftDistance();
    resetRightDistance();
  }

  public void setLeftMotors (double volts) {
    double newVolts = leftVolts + speedPID.calculate(leftVolts - volts);

    Logger.recordOutput("TankDrive/DrivetrainSubsystem/Speed/LeftInput", leftVolts);
    Logger.recordOutput("TankDrive/DrivetrainSubsystem/Speed/LeftTarget", volts);
    Logger.recordOutput("TankDrive/DrivetrainSubsystem/Speed/LeftOutput", newVolts);

    leftFrontMotor.setVoltage(newVolts);
    leftBackMotor.setVoltage(newVolts);
    leftVolts = newVolts;
  }
  public void setRightMotors (double volts) {
    double newVolts = rightVolts + speedPID.calculate(rightVolts - volts);

    Logger.recordOutput("TankDrive/DrivetrainSubsystem/Speed/RightInput", rightVolts);
    Logger.recordOutput("TankDrive/DrivetrainSubsystem/Speed/RightTarget", volts);
    Logger.recordOutput("TankDrive/DrivetrainSubsystem/Speed/RightOutput", newVolts);

    rightFrontMotor.setVoltage(newVolts);
    rightBackMotor.setVoltage(newVolts);
    rightVolts = newVolts;
  }
  public void setMotors (double lVolts, double rVolts) {
    setLeftMotors(lVolts);
    setRightMotors(rVolts);
  }
  public void setAllMotors (double volts) {
    setMotors(volts, volts);
  }
}
