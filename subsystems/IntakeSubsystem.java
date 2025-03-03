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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NeoMotorConstants;
import frc.robot.Constants.SparkMaxIDs;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax intakeLeftMotorController = new SparkMax(SparkMaxIDs.intakeLeftSparkID, MotorType.kBrushless);
  private final SparkMax intakeRightMotorController = new SparkMax(SparkMaxIDs.intakeRightSparkID, MotorType.kBrushless);
  private final double intakeSpeed = 0.3;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    ConfigSparks();
  }

  public void ConfigSparks() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .smartCurrentLimit(NeoMotorConstants.smallMotorCurrentLimit).idleMode(IdleMode.kBrake).closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    // persist config parameters over powercycle
    intakeLeftMotorController.configure(config, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeRightMotorController.configure(config, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void StopIntake() {
    intakeLeftMotorController.set(0);
    intakeRightMotorController.set(0);
  }

  public void IntakeCollect() {
    intakeLeftMotorController.set(intakeSpeed);
    intakeRightMotorController.set(-intakeSpeed);
  }

  public void IntakeReverse() {
    intakeLeftMotorController.set(-intakeSpeed);
    intakeRightMotorController.set(intakeSpeed);
  }
}
