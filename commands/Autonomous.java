
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Autonomous extends Command {
  private final DriveSubsystem driveSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final LiftSubsystem liftSubsystem;
  private final Timer timer;

  /** Creates a new AutonomousC. */
  public Autonomous(DriveSubsystem s_DriveSubsystem, ShooterSubsystem s_ShooterSubsystem, IntakeSubsystem s_IntakeSubsystem, LiftSubsystem s_LiftSubsystem) {
    timer = new Timer();
    driveSubsystem = s_DriveSubsystem;
    shooterSubsystem = s_ShooterSubsystem;
    intakeSubsystem = s_IntakeSubsystem;
    liftSubsystem = s_LiftSubsystem;
    addRequirements(driveSubsystem);
    addRequirements(shooterSubsystem);
    addRequirements(intakeSubsystem);
    addRequirements(liftSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(driveSubsystem.getPose().getX() + " " + driveSubsystem.getPose().getY());

    if (timer.get() < 1 && timer.get() > 0) {
      driveSubsystem.StopAtPosition(3.0, 0);
    } else if (timer.get() < 2 && timer.get() > 1) {
      driveSubsystem.StopAtAngle(60);
    } else if (timer.get() < 3 && timer.get() > 2) {
      driveSubsystem.StopAtPosition(4.0, -2.0);
    }
    
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (driveSubsystem.rotateBool == true) {
      return true;
    } else {
      return false;
    }
  }
}