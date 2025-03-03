// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DIOPortIDs;
import frc.robot.Constants.NeoMotorConstants;
import frc.robot.Constants.SparkMaxIDs;

public class LiftSubsystem extends SubsystemBase {
  // Subsystem Constants
  private final SparkMax liftMotorController = new SparkMax(SparkMaxIDs.liftSparkID, MotorType.kBrushless);
  private final RelativeEncoder encoder = liftMotorController.getEncoder();
  private final DigitalInput bottomLimitSwitch = new DigitalInput(DIOPortIDs.liftBottomLimit);
  
  private final double topManualSpeed = 0.3;
  private final int L4IntakeSpeedThreshold = 55; //TODO test these values
  private final int L3IntakeSpeedThreshold = 35;
  private final int L2IntakeSpeedThreshold = 15;

  // Subsystem Variables
  private String liftZone;

  public void ConfigSpark() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .smartCurrentLimit(NeoMotorConstants.bigMotorCurrentLimit).idleMode(IdleMode.kBrake).closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    // persist config parameters over powercycle
    liftMotorController.configure(config, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // The main constructor that initializes the subsystem
  public LiftSubsystem() {
    liftZone = "L1"; // The Lift should start at L1
    ConfigSpark();
    Stop(); // initialize stopped
  }

  public void SetSpeed(double speed) {
    if (bottomLimitSwitch.get()) {
      liftMotorController.set(speed);
    } else {
      Stop();
    }

  }

  public Command Forward() {
    return this.runOnce(() -> {
      liftMotorController.set(-topManualSpeed);
    });
  }

  public Command Reverse() {
    return this.runOnce(() -> {
      if (bottomLimitSwitch.get()) {
        liftMotorController.set(topManualSpeed);
      } else {
        Stop();
      }
    });
  }

  public Command CommandStop() {
    return this.runOnce(() -> Stop());
  }

  public void Stop() {
    liftMotorController.set(0.0);
  }

  public Command ResetEncoder() {
    return this.runOnce(() -> encoder.setPosition(0.0));
  }

  public double GetEncoderPosition() {
    return encoder.getPosition() * -1;
  }

  public void SlowDown(double startingSpeed) {
    for (int i = 1; i <= 4; i++) {
      liftMotorController.set(startingSpeed / i);
      DelayMS(50);
    }
  }

  public void DelayMS(int input) {
    try {
      Thread.sleep(input); // Sleep for 500 milliseconds (half a second)
    } catch (InterruptedException e) {
      // Handle the exception (e.g., log it or re-throw it)
      e.printStackTrace();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (GetEncoderPosition() >= L4IntakeSpeedThreshold) {
      ShooterSubsystem.leftSpeed = .2;
      ShooterSubsystem.rightSpeed = .2;
      liftZone = "L4";
    } else if (GetEncoderPosition() >= L3IntakeSpeedThreshold) {
      ShooterSubsystem.leftSpeed = .3;
      ShooterSubsystem.rightSpeed = .3;
      liftZone = "L3";
    } else if (GetEncoderPosition() >= L2IntakeSpeedThreshold) {
      ShooterSubsystem.leftSpeed = .3;
      ShooterSubsystem.rightSpeed = .4;
      liftZone = "L2";
    } else {
      ShooterSubsystem.leftSpeed = .3;
      ShooterSubsystem.rightSpeed = .5;
      liftZone = "L1";
    }

    // Dashboard Outputs
    SmartDashboard.putString("Lift Zone", liftZone);
    SmartDashboard.putNumber("Lift Encoder: ", GetEncoderPosition());
    SmartDashboard.putBoolean("Lift Lower Limit Switch: ", bottomLimitSwitch.get());
  }
}
