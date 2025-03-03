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

public class ClimberSubsystem extends SubsystemBase {
  private final SparkMax climberMotorController = new SparkMax(SparkMaxIDs.climbSparkID, MotorType.kBrushless);
  private final double climberSpeed = 0.3;

  /** Creates a new ClimbSubsystem. */
  public ClimberSubsystem() {
    ConfigSpark();
  }

  public void ConfigSpark() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .smartCurrentLimit(NeoMotorConstants.bigMotorCurrentLimit).idleMode(IdleMode.kBrake).closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    // persist config parameters over powercycle
    climberMotorController.configure(config, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void RunClimb() {
    climberMotorController.set(climberSpeed);
  }

  public void ReverseClimb() {
    climberMotorController.set(-climberSpeed);
  }

  public void StopClimb() {
    climberMotorController.set(0);
  }

}
