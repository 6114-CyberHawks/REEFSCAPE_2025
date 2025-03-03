// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import com.pathplanner.lib.auto.NamedCommands;
//import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ButtonBoxButtonIDs;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ZeroHeading;
import frc.robot.commands.Autonomous;
import frc.robot.commands.CollectAlgae;
import frc.robot.commands.DisengageClimber;
import frc.robot.commands.EngageClimber;
import frc.robot.commands.StopClimber;
import frc.robot.commands.StopGrabber;
import frc.robot.commands.StopLift;
import frc.robot.commands.StopShooter;
import frc.robot.commands.RunAgitator;
import frc.robot.commands.ShootCoral;
import frc.robot.commands.StopAgitator;
import frc.robot.commands.SetLiftHeight;
import frc.robot.commands.LoadShooter;
import frc.robot.commands.LowerGrabberArm;
import frc.robot.commands.RaiseGrabberArm;
import frc.robot.commands.ReleaseAlgae;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems : naming convention is s_ for subsystem
  public final static DriveSubsystem s_DriveSubsystem = new DriveSubsystem();
  private final LiftSubsystem s_LiftSubsystem = new LiftSubsystem();
  public final ClimberSubsystem s_ClimbSubsystem = new ClimberSubsystem();
  private final ShooterSubsystem s_ShooterSubsystem = new ShooterSubsystem();
  private final AgitatorSubsystem s_AgitatorSubsystem = new AgitatorSubsystem();
  private final GrabberSubsystem s_GrabberSubsystem = new GrabberSubsystem();
  private final IntakeSubsystem s_IntakeSubsystem = new IntakeSubsystem();

  // Commands : naming convention is c_ for command
  private final Autonomous c_Autonomous;
  private final CollectAlgae c_CollectAlgae;
  private final DisengageClimber c_DisengageClimber;
  private final EngageClimber c_EngageClimber;
  private final SetLiftHeight c_SetLiftPosition_L1;
  private final SetLiftHeight c_SetLiftPosition_L2;
  private final SetLiftHeight c_SetLiftPosition_L3;
  private final SetLiftHeight c_SetLiftPosition_L4;
  private final LoadShooter c_LoadShooter;
  private final LowerGrabberArm c_LowerGrabberArm;
  private final RaiseGrabberArm c_RaiseGrabberArm;
  private final ReleaseAlgae c_ReleaseAlgae;
  private final RunAgitator c_RunAgitator;
  private final ShootCoral c_ShootCoral;
  private final StopAgitator c_StopAgitator;
  private final StopClimber c_StopClimber;
  private final StopGrabber c_StopGrabber;
  private final StopLift c_StopLift;
  private final StopShooter c_StopShooter;
  private final ZeroHeading c_ZeroHeading;

  // Other Tools
  private final Timer timer;

  // The driver's controller
  public final static XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  // The operator's button box
  public final static GenericHID m_operatorButtonBox = new GenericHID(OIConstants.kOperatorControllerPort);
  // this is if the operator will use a controller
  //public static XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Initialize Commands
    c_Autonomous = new Autonomous(s_DriveSubsystem, s_ShooterSubsystem, s_IntakeSubsystem, s_LiftSubsystem);
    c_CollectAlgae = new CollectAlgae(s_GrabberSubsystem);
    c_DisengageClimber = new DisengageClimber(s_ClimbSubsystem);
    c_EngageClimber = new EngageClimber(s_ClimbSubsystem);
    // This is where the Lift heights are set // TODO Calibrate each height
    c_SetLiftPosition_L1 = new SetLiftHeight(s_LiftSubsystem, 0);
    c_SetLiftPosition_L2 = new SetLiftHeight(s_LiftSubsystem, 20);
    c_SetLiftPosition_L3 = new SetLiftHeight(s_LiftSubsystem, 40);
    c_SetLiftPosition_L4 = new SetLiftHeight(s_LiftSubsystem, 60);
    c_LoadShooter = new LoadShooter(s_ShooterSubsystem, s_IntakeSubsystem);
    c_LowerGrabberArm = new LowerGrabberArm(s_GrabberSubsystem);
    c_RaiseGrabberArm = new RaiseGrabberArm(s_GrabberSubsystem);
    c_ReleaseAlgae = new ReleaseAlgae(s_GrabberSubsystem);
    c_RunAgitator = new RunAgitator(s_AgitatorSubsystem);
    c_ShootCoral = new ShootCoral(s_ShooterSubsystem);
    c_StopAgitator = new StopAgitator(s_AgitatorSubsystem);
    c_StopClimber = new StopClimber(s_ClimbSubsystem);
    c_StopGrabber = new StopGrabber(s_GrabberSubsystem);
    c_StopLift = new StopLift(s_LiftSubsystem);
    c_StopShooter = new StopShooter(s_ShooterSubsystem);
    c_ZeroHeading = new ZeroHeading(s_DriveSubsystem);

    // Create and Reset Timer
    timer = new Timer();
    timer.reset();

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    try {
      s_DriveSubsystem.setDefaultCommand(
          // The left stick controls translation of the robot.
          // Turning is controlled by the X axis of the right stick.

          Commands.run(
              () -> s_DriveSubsystem.drive(
                  -MathUtil.applyDeadband(m_driverController.getLeftY(),
                      OIConstants.kDriveDeadband),
                  -MathUtil.applyDeadband(m_driverController.getLeftX(),
                      OIConstants.kDriveDeadband),
                  -MathUtil.applyDeadband(m_driverController.getRightX(),
                      OIConstants.kDriveDeadband),
                  true, true),
              s_DriveSubsystem));
    } catch (Exception e) {
      System.out.println("drive command in robotContainer failed, exception: " + e);
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    // Driver Controller Mapping
    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(c_ZeroHeading);

    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(c_ShootCoral).onFalse(c_StopShooter);

    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .whileTrue(c_LoadShooter).onFalse(c_StopShooter); //TODO this might not need stop shooter cmd

    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .whileTrue(c_RunAgitator).onFalse(c_StopAgitator);

    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .whileTrue(c_CollectAlgae).onFalse(c_StopGrabber);

    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .whileTrue(c_ReleaseAlgae).onFalse(c_StopGrabber);

    // Operator Mapping
    Trigger liftL1Trigger = new JoystickButton(m_operatorButtonBox, ButtonBoxButtonIDs.LiftL1);
    liftL1Trigger.onTrue(c_SetLiftPosition_L1);

    Trigger liftL2Trigger = new JoystickButton(m_operatorButtonBox, ButtonBoxButtonIDs.LiftL2);
    liftL2Trigger.onTrue(c_SetLiftPosition_L2);

    Trigger liftL3Trigger = new JoystickButton(m_operatorButtonBox, ButtonBoxButtonIDs.LiftL3);
    liftL3Trigger.onTrue(c_SetLiftPosition_L3);

    Trigger liftL4Trigger = new JoystickButton(m_operatorButtonBox, ButtonBoxButtonIDs.LiftL4);
    liftL4Trigger.onTrue(c_SetLiftPosition_L4);

    Trigger liftUpTrigger = new JoystickButton(m_operatorButtonBox, ButtonBoxButtonIDs.LiftUp);
    liftUpTrigger.whileTrue(s_LiftSubsystem.Forward()).onFalse(c_StopLift);

    Trigger liftDownTrigger = new JoystickButton(m_operatorButtonBox, ButtonBoxButtonIDs.LiftDown);
    liftDownTrigger.whileTrue(s_LiftSubsystem.Reverse()).onFalse(c_StopLift);

    Trigger climberDisengageTrigger = new JoystickButton(m_operatorButtonBox, ButtonBoxButtonIDs.ClimberDisengage);
    climberDisengageTrigger.whileTrue(c_DisengageClimber).onFalse(c_StopClimber);

    Trigger climberEngageTrigger = new JoystickButton(m_operatorButtonBox, ButtonBoxButtonIDs.ClimberEngage);
    climberEngageTrigger.whileTrue(c_EngageClimber).onFalse(c_StopClimber);

    Trigger grabberInTrigger = new JoystickButton(m_operatorButtonBox, ButtonBoxButtonIDs.GrabberIn);
    grabberInTrigger.onTrue(c_RaiseGrabberArm);

    Trigger grabberOutTrigger = new JoystickButton(m_operatorButtonBox, ButtonBoxButtonIDs.GrabberOut);
    grabberOutTrigger.onTrue(c_LowerGrabberArm);
  }

  /*
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return c_Autonomous;
  }

}
