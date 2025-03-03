// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EngageClimber extends Command {
  private final ClimberSubsystem climbSubsystem;

  /** Creates a new ClimRun. */
  public EngageClimber(ClimberSubsystem s_ClimberSubsystem) {
    climbSubsystem = s_ClimberSubsystem;
    addRequirements(climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbSubsystem.RunClimb();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbSubsystem.StopClimb();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; //TODO is there supposed to be a limit?
  }
}
