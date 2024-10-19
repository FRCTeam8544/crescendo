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
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AmpScore.HandoffCommand;
import frc.robot.commands.AmpScore.MovePivotIn;
import frc.robot.commands.AmpScore.MovePivotOut;
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

  //private final SpeakerAuto shootAuto = new SpeakerAuto(m_shooter, m_intake);
  //private final Cameras cameras = new Cameras(m_robotDrive);


  private final testAuto m_testAuto = new testAuto(m_robotDrive, m_shooter, m_intake);
  private final ShootAndMove m_shootAndMoveAuto = new ShootAndMove(m_robotDrive, m_intake, m_shooter);
  private final ShootAuto m_shootOnlyAuto = new ShootAuto(m_shooter, m_intake, m_robotDrive);
  private final FixedShoot m_fixedShooter = new FixedShoot(m_shooter, m_intake);
  private final DriveAndShootAuto m_twoNoteAuto = new DriveAndShootAuto(m_robotDrive, m_intake, m_shooter);
  //private final DriveAndShootAuto driveAndShootAuto = new DriveAndShootAuto(m_robotDrive, m_intake, shootAuto);
  //private final ShootAuto m_FixedShoot = new SpeakerAuto(m_shooter, m_intake).withTimeout(1.5);
  

  // romeo and juliet, this is where our humble tale begins 
  XboxController m_romeo = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_juliet = new XboxController(1);
  GenericHID stinkyPooPoo = new GenericHID(2);
  
  //XboxController m_romeo = m_juliet;

  private final IntakeAuto intakeAuto = new IntakeAuto(m_intake, m_juliet, m_romeo);
  private final IntakeStopAuto intakeStopAuto = new IntakeStopAuto(m_intake);

  private Pose2d zeroPose2d = new Pose2d(0, 0, Rotation2d.fromDegrees(0));


  /*public BooleanSupplier intakeAutoRunning = () -> {
    return intakeAuto.isScheduled();
  };*/

  //private final PrepareHangAuto prepareHangAuto = new PrepareHangAuto(m_intake, m_climber, m_juliet);
  //private final FinishHangAuto finishHangAuto = new FinishHangAuto(m_intake, m_climber)

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

    toggle.setDefaultOption("Fixed shoot only", m_fixedShooter);
    toggle.addOption("speaker Only", m_shootOnlyAuto);//queens gambit
    toggle.addOption("shoot And Move", m_shootAndMoveAuto);//london system
    toggle.addOption("null", null);//cloud bong
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
                -MathUtil.applyDeadband(m_romeo.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_romeo.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_romeo.getRightX(), OIConstants.kDriveDeadband) * -1,
                true, true),
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
  private void configureButtonBindings() { //ALL BUTTON BINDINGS ARE SUBJECT TO CHANGE

    new JoystickButton(m_romeo, Button.kX.value)//youll never guess what button you gotta press to do this
        .onTrue(new RunCommand(//thats right its a button combo not gonna tell you how I did it its a secret
        () -> m_robotDrive.toggleBox(), m_robotDrive));

        /***
         * This is like my favorite poem btw if you even care
         * 
         * 
O Captain! my Captain! our fearful trip is done,
The ship has weather’d every rack, the prize we sought is won,
The port is near, the bells I hear, the people all exulting,
While follow eyes the steady keel, the vessel grim and daring;
                         But O heart! heart! heart!
                            O the bleeding drops of red,
                               Where on the deck my Captain lies,
                                  Fallen cold and dead.

O Captain! my Captain! rise up and hear the bells;
Rise up—for you the flag is flung—for you the bugle trills,
For you bouquets and ribbon’d wreaths—for you the shores a-crowding,
For you they call, the swaying mass, their eager faces turning;
                         Here Captain! dear father!
                            This arm beneath your head!
                               It is some dream that on the deck,
                                 You’ve fallen cold and dead.

My Captain does not answer, his lips are pale and still,
My father does not feel my arm, he has no pulse nor will,
The ship is anchor’d safe and sound, its voyage closed and done,
From fearful trip the victor ship comes in with object won;
                         Exult O shores, and ring O bells!
                            But I with mournful tread,
                               Walk the deck my Captain lies,
                                  Fallen cold and dead.

         */

    new JoystickButton(m_romeo, Button.kStart.value)//romulus and remus
        .whileTrue(new RunCommand(
        () -> m_robotDrive.zeroHeading(), m_robotDrive));

    new JoystickButton(m_romeo, Button.kBack.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.resetOdometry(zeroPose2d) , m_robotDrive));



    new JoystickButton(m_juliet, Button.kB.value)
        .toggleOnTrue(new IntakeAuto(m_intake, m_juliet, m_romeo)).whileFalse(new IntakeStopAuto(m_intake));

    new JoystickButton(m_juliet, Button.kX.value)
        .toggleOnTrue(new PrepareHangAuto(m_intake, m_climber, m_juliet));



    new JoystickButton(m_juliet, Button.kRightBumper.value)
        .whileTrue(new SpeakerCommand(m_shooter, m_intake, m_juliet));

    new JoystickButton(m_juliet, Button.kStart.value)
        .whileTrue(new SourceIntake(m_intake, m_shooter));

    new JoystickButton(m_juliet, Button.kLeftBumper.value)
        .onTrue(new HandoffCommand(m_intake, m_shooter, m_juliet));

    /*new JoystickButton(m_juliet, Button.kA.value)
        .toggleOnTrue(new MovePivotIn(m_shootElevator));

    new JoystickButton(m_juliet, Button.kY.value)
        .toggleOnTrue(new MovePivotOut(m_shootElevator));*/



    new JoystickButton(stinkyPooPoo, 1)
        .whileTrue(new RunCommand( 
            () -> m_shootElevator.movePivor(false), m_shootElevator)).onFalse(
                new RunCommand(() -> m_shootElevator.stopPivot(), m_shootElevator));

    new JoystickButton(stinkyPooPoo, 3)
        .whileTrue(new RunCommand( 
            () -> m_shootElevator.movePivor(true), m_shootElevator)).onFalse(
                new RunCommand(() -> m_shootElevator.stopPivot(), m_shootElevator));

    new JoystickButton(stinkyPooPoo, 2).whileTrue(
        new RunCommand(() -> m_shootElevator.moveElevator(false), m_shootElevator)).onFalse(
            new RunCommand(() -> m_shootElevator.stopElevator(), m_shootElevator));

    new JoystickButton(stinkyPooPoo, 4).whileTrue(
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