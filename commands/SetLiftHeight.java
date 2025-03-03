// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftSubsystem;

public class SetLiftHeight extends Command {
  private final LiftSubsystem encoderSubsystem;
  private final double topSpeed = 0.3;
  @SuppressWarnings("unused") private boolean movingForwards;
  @SuppressWarnings("unused") private boolean movingBackwards;
  int desiredPosition;
  double positionTolerance = 2; // Can't be 0! May need to change based on speed
  double currentPosition;

  /** Creates a new SetPosition. */
  public SetLiftHeight(LiftSubsystem subsystem, int inputPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    encoderSubsystem = subsystem;
    desiredPosition = inputPosition;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    movingForwards = false;
    movingBackwards = false;
    currentPosition = encoderSubsystem.GetEncoderPosition();
    if(currentPosition < (desiredPosition - positionTolerance)) {
      movingForwards = true;
    } else if (currentPosition > (desiredPosition + positionTolerance)) {
      movingBackwards = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPosition = encoderSubsystem.GetEncoderPosition();
    if(currentPosition < (desiredPosition - positionTolerance)) {
      encoderSubsystem.SetSpeed(-topSpeed);
    } else if (currentPosition > (desiredPosition- positionTolerance)) {
      encoderSubsystem.SetSpeed(topSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // TODO if needed add slowdown back in
    // if(movingForwards) {
    //   encoderSubsystem.SlowDown(topSpeed);
    // } else if (movingBackwards) {
    //   encoderSubsystem.SlowDown(-topSpeed);
    // }
    encoderSubsystem.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (currentPosition <= (desiredPosition + positionTolerance)) && (currentPosition >= (desiredPosition - positionTolerance));
  }
}
