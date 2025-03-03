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

public class GrabberSubsystem extends SubsystemBase {
  private final SparkMax grabberArmMotorController = new SparkMax(SparkMaxIDs.grabberArmSparkID, MotorType.kBrushless);
  private final SparkMax grabberMotorController = new SparkMax(SparkMaxIDs.grabberSparkID, MotorType.kBrushless);
  private final DigitalInput grabberInLimit = new DigitalInput(DIOPortIDs.grabberArmInLimit);
  private final DigitalInput grabberOutLimit = new DigitalInput(DIOPortIDs.grabberArmOutLimit);
  private final double grabberMotorSpeed = 0.5;

  /** Creates a new GrabberSubsystem. */
  public GrabberSubsystem() {
    ConfigSparks();
  }

  public void ConfigSparks() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .smartCurrentLimit(NeoMotorConstants.bigMotorCurrentLimit).idleMode(IdleMode.kBrake).closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    // persist config parameters over powercycle
    grabberArmMotorController.configure(config, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    grabberMotorController.configure(config, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void SetGrabberArmMotor(double inputSpeed) {
    grabberArmMotorController.set(inputSpeed);
  }

  public void StopGrabberArm() {
    grabberArmMotorController.set(0.0);
  }

  public void CollectGrabberMotor() {
    grabberMotorController.set(grabberMotorSpeed); //TODO test motor direction and speed
  }

  public void StopGrabber() {
    grabberMotorController.set(0.0);
  }

  public void ReleaseGrabberMotor() {
    grabberMotorController.set(-grabberMotorSpeed); //TODO test motor direction and speed
  }

  public Boolean GetGrabberInLimit() {
    return grabberInLimit.get();
  }

  public Boolean GetGrabberOutLimit() {
    return grabberOutLimit.get();
  }
}
