// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LoadShooter extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  /** Creates a new LoadShooter. */
  public LoadShooter(ShooterSubsystem s_ShooterSubsystem, IntakeSubsystem s_IntakeSubsystem) {
    shooterSubsystem = s_ShooterSubsystem;
    intakeSubsystem = s_IntakeSubsystem;
    addRequirements(shooterSubsystem);
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.IntakeCollect();
    shooterSubsystem.LoadShooter();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.StopIntake();
    shooterSubsystem.StopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterSubsystem.GetCoralDetector() == true; // TODO test this
  }
}
