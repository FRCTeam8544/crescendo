// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AmpScore.HandoffCommand;
import frc.robot.commands.Autos.AutoCommands.IntakeExtendAuto;
import frc.robot.commands.Autos.AutoCommands.IntakeRetractAuto;
import frc.robot.commands.Autos.AutoCommands.SpeakerAuto;
import frc.robot.commands.Autos.AutoCommands.IntakeRollersAuto;
import frc.robot.commands.Autos.AutoCommands.IntakeRollersStopAuto;
import frc.robot.commands.Autos.AutoSequences.FinishHangAuto;
import frc.robot.commands.Autos.AutoSequences.IntakeAuto;
import frc.robot.commands.Autos.AutoSequences.IntakeStopAuto;
import frc.robot.commands.Autos.AutoSequences.PrepareHangAuto;
import frc.robot.commands.Autos.AutoSequences.ShootAndMove;
import frc.robot.commands.Autos.AutoSequences.ShootAuto;
import frc.robot.commands.Autos.AutoSequences.testAuto;
import frc.robot.commands.Intake.SourceIntake;
import frc.robot.commands.SpeakerScore.SpeakerCommand;
import frc.robot.subsystems.ClimberElevator;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.ShooterElevator;
import frc.robot.vision.Cameras;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand; 
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShootSubsystem m_shooter = new ShootSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  //private final ShooterElevator m_shootElevator = new ShooterElevator();
  private final ClimberElevator m_climber = new ClimberElevator();

  //private final Cameras cameras = new Cameras(m_robotDrive);


  private final testAuto m_testAuto = new testAuto(m_robotDrive, m_shooter, m_intake);
  private final ShootAndMove m_shootAndMoveAuto = new ShootAndMove(m_robotDrive, m_intake, m_shooter);
  private final ShootAuto m_shootOnlyAuto = new ShootAuto(m_shooter, m_intake, m_robotDrive);
  private final PathPlannerAuto m_realestTestAuto = new PathPlannerAuto("PrimaryAuto");


  

  // romeo and juliet, this is where our humble tale begins 
  XboxController m_romeo = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_juliet = new XboxController(1);
  //XboxController m_romeo = m_juliet;

  private final IntakeAuto intakeAuto = new IntakeAuto(m_intake);
  private final IntakeStopAuto intakeStopAuto = new IntakeStopAuto(m_intake);

  private final PrepareHangAuto prepareHangAuto = new PrepareHangAuto(m_intake, m_climber, m_juliet);
  private final FinishHangAuto finishHangAuto = new FinishHangAuto(m_intake, m_climber);

  BooleanSupplier init = () -> {
    return (m_romeo.getLeftTriggerAxis() > 0.5);
  };

  BooleanSupplier run = () -> {
    return (m_romeo.getRightTriggerAxis() > 0.5);
  };

  BooleanSupplier initDone = () -> {
    return (m_romeo.getLeftTriggerAxis() <= 0.1);
  };

  BooleanSupplier runDone = () -> {
    return (m_romeo.getRightTriggerAxis() <= 0.1);
  };

  private SendableChooser<Command> toggle = new SendableChooser<>();


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    
    toggle.setDefaultOption("pick this one", m_realestTestAuto);//intercontinental ballistic missile gambit
    toggle.addOption("2 note Auto (center)", m_testAuto);//kings gambit double muzio
    toggle.addOption("speaker Only", m_shootOnlyAuto);//queens gambit
    toggle.addOption("shoot And Move", m_shootAndMoveAuto);//london system
    toggle.addOption("null", null);//cloud bong
    //toggle.addOption("Those who danced were seen as crazy by those who couldnt hear the music", m_fightGod);
    SmartDashboard.putData("Select Autonomous", toggle);//the puppet master

    /*autoChooser = AutoBuilder.buildAutoChooser(); - for when we fully convert to AutoBuilder
    SmartDashboard.putData("Auto Chooser", autoChooser);*/

    //declare commands for path software, this needs to be before configureButtonBindings();
    NamedCommands.registerCommand("IntakeExtendAuto", new IntakeExtendAuto(m_intake));
    NamedCommands.registerCommand("IntakeRetractAuto", new IntakeRetractAuto(m_intake));
    NamedCommands.registerCommand("IntakeRollersAuto", new IntakeRollersAuto(m_intake));
    NamedCommands.registerCommand("IntakeRollersStopAuto", new IntakeRollersStopAuto(m_intake));
    NamedCommands.registerCommand("IntakeStopAuto", new IntakeStopAuto(m_intake));
    NamedCommands.registerCommand("SpeakerAuto", new SpeakerAuto(m_shooter, m_intake));

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(//I drive
                -MathUtil.applyDeadband(m_romeo.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_romeo.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_romeo.getRightX(), OIConstants.kDriveDeadband) * -1,
                true, true),
            m_robotDrive));
  }

  public Command getAutonomousCommand() {

    //return autoChooser.getSelected(); - for when we fully commit to AutoBuilder
    return toggle.getSelected();//lucas got bored and is next to me send help
    //what the clutter
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
  private void configureButtonBindings() { //ALL BUTTON BINDINGS ARE SUBJECT TO CHANGE :3

    new JoystickButton(m_romeo, Button.kStart.value)//romulus and remus
        .whileTrue(new RunCommand(
        () -> m_robotDrive.zeroHeading(), m_robotDrive));
    
    new JoystickButton(m_juliet, Button.kB.value)
        .onTrue(intakeAuto).whileFalse(intakeStopAuto);

    new JoystickButton(m_juliet, Button.kX.value)
        .onTrue(prepareHangAuto.andThen(finishHangAuto));
    
    new JoystickButton(m_juliet, Button.kRightBumper.value)
        .onTrue(new SpeakerCommand(m_shooter, m_intake, m_juliet));

    new JoystickButton(m_juliet, Button.kLeftBumper.value)
        .whileTrue(new HandoffCommand(m_intake, m_shooter));

    new JoystickButton(m_juliet, Button.kStart.value)
        .whileTrue(new SourceIntake(m_intake, m_shooter));
  }
}