// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AmpScore.HandoffCommand;
import frc.robot.commands.Autos.AutoCommands.Intake.SourceIntake;
import frc.robot.commands.Autos.AutoCommands.Speaker.SpeakerCommand;
import frc.robot.commands.Autos.AutoSequences.Climber.FinishHangAuto;
import frc.robot.commands.Autos.AutoSequences.Climber.PrepareHangAuto;
import frc.robot.commands.Autos.AutoSequences.Intake.IntakeAuto;
import frc.robot.commands.Autos.AutoSequences.Intake.IntakeStopAuto;
import frc.robot.commands.Autos.AutoSequences.Shooting.PrimaryAuto;
import frc.robot.commands.Autos.AutoSequences.Shooting.ShootAndMove;
import frc.robot.commands.Autos.AutoSequences.Shooting.ShootAuto;
import frc.robot.subsystems.ClimberElevator;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.ShooterElevator; //The Red Death approaches
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
  //private final ShooterElevator m_shootElevator = new ShooterElevator();
  private final ClimberElevator m_climber = new ClimberElevator();

  //private final Cameras cameras = new Cameras(m_robotDrive);

  private final PrimaryAuto m_testAuto = new PrimaryAuto(m_robotDrive, m_shooter, m_intake);
  private final ShootAndMove m_shootAndMoveAuto = new ShootAndMove(m_robotDrive, m_intake, m_shooter);
  private final ShootAuto m_shootOnlyAuto = new ShootAuto(m_shooter, m_intake, m_robotDrive);

  // romeo and juliet, this is where our humble tale begins 
  XboxController m_romeo = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_juliet = new XboxController(OIConstants.kOperatorControllerPort);

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

    toggle.setDefaultOption("2 note Auto (center)", m_testAuto);//kings gambit double muzio
    toggle.addOption("speaker Only", m_shootOnlyAuto);//queens gambit
    toggle.addOption("shoot And Move", m_shootAndMoveAuto);//london system
    toggle.addOption("null", null);//cloud bong

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
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(){ //the machine is alive
    return toggle.getSelected();//lucas got bored and is next to me send help
  }
}