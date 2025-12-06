// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

  private SparkMaxConfig config = new SparkMaxConfig();
  private SparkMax leftFrontMotor = new SparkMax(1, MotorType.kBrushless);
  private SparkMax leftBackMotor = new SparkMax(4, MotorType.kBrushless);
  private SparkMax rightFrontMotor = new SparkMax(2, MotorType.kBrushless);
  private SparkMax rightBackMotor = new SparkMax(3, MotorType.kBrushless);

  private RelativeEncoder leftFrontEncoder = leftFrontMotor.getEncoder();
  private RelativeEncoder rightFrontEncoder = rightFrontMotor.getEncoder();

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

  public void setLeftMotors (double speed) {
    double currentSpeed = leftFrontMotor.get();
    double newSpeed = currentSpeed + speedPID.calculate(currentSpeed - speed);

    leftFrontMotor.set(newSpeed);
    leftBackMotor.set(newSpeed);
  }
  public void setRightMotors (double speed) {
    double currentSpeed = rightFrontMotor.get();
    double newSpeed = currentSpeed + speedPID.calculate(currentSpeed - speed);

    rightFrontMotor.set(newSpeed);
    rightBackMotor.set(newSpeed);
  }
  public void setMotors (double lSpeed, double rSpeed) {
    setLeftMotors(lSpeed);
    setRightMotors(rSpeed);
  }
  public void setAllMotors (double speed) {
    setMotors(speed, speed);
  }
}
