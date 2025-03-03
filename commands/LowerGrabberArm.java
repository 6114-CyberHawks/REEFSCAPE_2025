// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GrabberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LowerGrabberArm extends Command {
  private final GrabberSubsystem grabberSubsystem;

  /** Creates a new LowerGrabberArm. */
  public LowerGrabberArm(GrabberSubsystem s_GrabberSubsystem) {
    grabberSubsystem = s_GrabberSubsystem;
    addRequirements(grabberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    grabberSubsystem.SetGrabberArmMotor(-0.1); //TODO check direction and speed
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    grabberSubsystem.SetGrabberArmMotor(0.0); // Stops grabber arm after command finished
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return grabberSubsystem.GetGrabberOutLimit() == true; // TODO check limit switch
  }
}
