// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.AmpScore.HandoffCommand;
import frc.robot.commands.Autos.AutoSequences.DriveAndShootAuto;
import frc.robot.commands.Autos.AutoSequences.FixedShoot;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.function.BooleanSupplier;

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
  private final ShooterElevator m_shootElevator = new ShooterElevator();
  private final ClimberElevator m_climber = new ClimberElevator();

  // The robot's autonomousies
  private final testAuto m_testAuto = new testAuto(m_robotDrive, m_shooter, m_intake);
  private final ShootAndMove m_shootAndMoveAuto = new ShootAndMove(m_robotDrive, m_intake, m_shooter);
  private final ShootAuto m_shootOnlyAuto = new ShootAuto(m_shooter, m_intake, m_robotDrive);
  private final FixedShoot m_fixedShooter = new FixedShoot(m_shooter, m_intake);
  private final DriveAndShootAuto m_twoNoteAuto = new DriveAndShootAuto(m_robotDrive, m_intake, m_shooter);

  // romeo and juliet, this is where our humble tale begins 
  XboxController m_romeo = new XboxController(IOConstants.kDriverControllerPort);
  XboxController m_juliet = new XboxController(IOConstants.kOperatorControllerPort);
  GenericHID stinkyPooPoo = new GenericHID(IOConstants.k1800lemonlaw);

  // relax and rewind
  private Pose2d zeroPose2d = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

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

    toggle.setDefaultOption("fixed shoot only", m_fixedShooter);
    toggle.addOption("speaker only", m_shootOnlyAuto);//queens gambit
    toggle.addOption("shoot and move", m_shootAndMoveAuto);//london system
    toggle.addOption("sitting duck", null);//cloud bong
    toggle.addOption("2 not auto (center)", m_testAuto);
    toggle.addOption("2 note auto (fixed)", m_twoNoteAuto);
    SmartDashboard.putData("Select Autonomous", toggle);//the puppet master
    

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(//I drive
                -MathUtil.applyDeadband(m_romeo.getLeftY(), IOConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_romeo.getLeftX(), IOConstants.kDriveDeadband),
                MathUtil.applyDeadband(m_romeo.getRightX(), IOConstants.kDriveDeadband),
                IOConstants.fieldRelative, true),
            m_robotDrive));

    m_shootElevator.setDefaultCommand(
        new RunCommand(
            () -> m_shootElevator.teleopElevator(
                m_juliet.getPOV() == 0, m_juliet.getPOV() == 180), 
             m_shootElevator));
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

    // Domain expansion (robot is trapped in a box)
    new JoystickButton(m_romeo, Button.kX.value)//youll never guess what button you gotta press to do this
        .onTrue(new RunCommand(//thats right its a button combo not gonna tell you how I did it its a secret
        () -> m_robotDrive.toggleBox(), m_robotDrive));

    // Reset robot gyro (because the gyro was installed backward)
    new JoystickButton(m_romeo, Button.kStart.value)//romulus and remus
        .whileTrue(new RunCommand(
        () -> m_robotDrive.zeroHeading(), m_robotDrive));

    // Reset robot odometry 
    new JoystickButton(m_romeo, Button.kBack.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.resetOdometry(zeroPose2d) , m_robotDrive));

    // Intake from ground
    new JoystickButton(m_juliet, Button.kB.value)
        .toggleOnTrue(new IntakeAuto(m_intake, m_juliet, m_romeo))
        .whileFalse(new IntakeStopAuto(m_intake));

    // Prepare for hanging
    new JoystickButton(m_juliet, Button.kX.value)
        .toggleOnTrue(new PrepareHangAuto(m_intake, m_climber, m_juliet));

    // Shoot the stored note
    new JoystickButton(m_juliet, Button.kRightBumper.value)
        .whileTrue(new SpeakerCommand(m_shooter, m_intake, m_juliet));

    // Intake from source directly
    new JoystickButton(m_juliet, Button.kStart.value)
        .whileTrue(new SourceIntake(m_intake, m_shooter));

    // Handoff intake -> shooter to prep for amp
    new JoystickButton(m_juliet, Button.kLeftBumper.value)
        .onTrue(new HandoffCommand(m_intake, m_shooter, m_juliet));

    // Move pivot up
    new JoystickButton(stinkyPooPoo, IOConstants.movePivotUp)
        .whileTrue(new RunCommand( 
            () -> m_shootElevator.movePivot(false), m_shootElevator)).onFalse(
                new RunCommand(() -> m_shootElevator.stopPivot(), m_shootElevator));

    // Move pivot down
    new JoystickButton(stinkyPooPoo, IOConstants.movePivotDown)
        .whileTrue(new RunCommand( 
            () -> m_shootElevator.movePivot(true), m_shootElevator)).onFalse(
                new RunCommand(() -> m_shootElevator.stopPivot(), m_shootElevator));

    // Lower elevator
    new JoystickButton(stinkyPooPoo, IOConstants.moveElevatorDown).whileTrue(
        new RunCommand(() -> m_shootElevator.moveElevator(false), m_shootElevator)).onFalse(
            new RunCommand(() -> m_shootElevator.stopElevator(), m_shootElevator));

    // Raise elevator
    new JoystickButton(stinkyPooPoo, IOConstants.moveElevatorUp).whileTrue(
        new RunCommand(() -> m_shootElevator.moveElevator(true), m_shootElevator)).onFalse(
            new RunCommand(() -> m_shootElevator.stopElevator(), m_shootElevator));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   return toggle.getSelected();
  }
}