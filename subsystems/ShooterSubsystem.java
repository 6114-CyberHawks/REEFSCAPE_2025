// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DIOPortIDs;
import frc.robot.Constants.NeoMotorConstants;
import frc.robot.Constants.SparkMaxIDs;

public class ShooterSubsystem extends SubsystemBase {
  private final DigitalInput coralDetector = new DigitalInput(DIOPortIDs.shooterCoralDetector);
  private SparkMax shooterLeftMototController = new SparkMax(SparkMaxIDs.shooterLeftSparkID, MotorType.kBrushless);
  private SparkMax shooterRightMototController = new SparkMax(SparkMaxIDs.shooterRightSparkID, MotorType.kBrushless);
  public static double leftSpeed;
  public static double rightSpeed;
  private final double shooterLoadSpeed = 0.3; // this value should match the Intake Subsystem's intakeSpeed

  public void ConfigSparks() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .smartCurrentLimit(NeoMotorConstants.smallMotorCurrentLimit).idleMode(IdleMode.kBrake).closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    // persist config parameters over powercycle
    shooterLeftMototController.configure(config, SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    shooterRightMototController.configure(config, SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public ShooterSubsystem() {
    ConfigSparks();
    StopShooter();
  }

  @Override
  public void periodic() {}

  public Boolean GetCoralDetector() {
    return coralDetector.get();
  }

  public void LoadShooter() {
    shooterLeftMototController.set(shooterLoadSpeed);
    shooterRightMototController.set(-shooterLoadSpeed);
  }

  public void StopShooter() {
    shooterLeftMototController.set(0);
    shooterRightMototController.set(0);
  }

  public void SetShooterMotors() {
    // These values are constantly updated based on the Lift subsystem
    shooterLeftMototController.set(leftSpeed);
    shooterRightMototController.set(-rightSpeed); // this accounts for the reverse
  }
}
