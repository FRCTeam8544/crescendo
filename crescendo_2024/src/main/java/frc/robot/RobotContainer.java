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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimbElevatorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShootElevatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.StopConstant;
import frc.robot.commands.AmpScore.HandoffCommand;
import frc.robot.commands.Autos.AutoCommands.IntakeRetractAuto;
import frc.robot.commands.Autos.AutoCommands.SpeakerAuto;
import frc.robot.commands.Autos.AutoCommands.intakeRollersAuto;
import frc.robot.commands.Autos.AutoSequences.FinishHangAuto;
import frc.robot.commands.Autos.AutoSequences.IntakeAuto;
import frc.robot.commands.Autos.AutoSequences.IntakeStopAuto;
import frc.robot.commands.Autos.AutoSequences.PrepareHangAuto;
import frc.robot.commands.Autos.AutoSequences.ShootAndMove;
import frc.robot.commands.Autos.AutoSequences.ShootAuto;
import frc.robot.commands.Autos.AutoSequences.testAuto;
import frc.robot.commands.Intake.IntakeCommand;
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


  

  // romeo and juliet, this is where our humble tale begins 
  XboxController m_romeo = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_juliet = new XboxController(1);
  //XboxController m_romeo = m_juliet;

  //private final IntakeAuto intakeAuto = new IntakeAuto(m_intake);
  //private final IntakeStopAuto intakeStopAuto = new IntakeStopAuto(m_intake);

  /*public BooleanSupplier intakeAutoRunning = () -> {
    return intakeAuto.isScheduled();
  };*/

  //private final PrepareHangAuto prepareHangAuto = new PrepareHangAuto(m_intake, m_climber, m_juliet);
  //private final FinishHangAuto finishHangAuto = new FinishHangAuto(m_intake, m_climber);

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

    toggle.setDefaultOption("null", null);
    //toggle.setDefaultOption("2 note Auto (center)", m_testAuto);//kings gambit double muzio
    toggle.addOption("speaker Only", m_shootOnlyAuto);//queens gambit
    toggle.addOption("shoot And Move", m_shootAndMoveAuto);//london system
    toggle.addOption("null", null);//cloud bong
    toggle.addOption("2 not auto (center)", m_testAuto);

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
  private void configureButtonBindings() { //ALL BUTTON BINDINGS ARE SUBJECT TO CHANGE

    new JoystickButton(m_romeo, Button.kStart.value)//romulus and remus
        .whileTrue(new RunCommand(
        () -> m_robotDrive.zeroHeading(), m_robotDrive));

    //shooter commands
    /*
     * 
    new JoystickButton(m_romeo, Button.kA.value) //A Button
        .whileTrue(new RunCommand(
            () -> m_shooter.shoot(ShooterConstants.shootSetpoint),
            m_shooter))
            .onFalse(new RunCommand(
            () -> m_shooter.stop(StopConstant.stopSetpoint), m_shooter));
    */
    /* This is a kill switch that could be removed whenever, but it probably shouldnt be for now 
    new JoystickButton(m_romeo, Button.kB.value) // B Button
        .whileTrue(new RunCommand(
        () -> m_shooter.stop(StopConstant.stopSetpoint),
        m_shooter));
    */
    /*
        Likely uneeded since we won't intake via shooter anymore

    new JoystickButton(m_romeo, Button.k-.value) X button now conflicts with intaking fron ground,
        .whileTrue(new RunCommand(                          which is worth noting if we ever re-implement this
        () -> m_shooter.sourceIntake(ShooterConstants.intakeSetpoint), 
            m_shooter))
                .onFalse(new RunCommand(
                () -> m_shooter.stopMovement(StopConstant.stopSetpoint), m_shooter));
    */
    /*new JoystickButton(m_juliet, Button.kA.value)
        .whileTrue(new RunCommand(
            () -> m_climber.moveClimber(0.1), m_climber)).onFalse(
                new RunCommand(() -> m_climber.stop(), m_climber));
    new JoystickButton(m_juliet, Button.kBack.value)
        .whileTrue(new RunCommand(
            () -> m_climber.moveClimber(-0.1), m_climber)).onFalse(
                new RunCommand(() -> m_climber.stop(), m_climber));*/

    //intake commands
    /*new JoystickButton(m_juliet, Button.kB.value) // changed to X from left bumper
        .whileTrue(new RunCommand(
            () -> m_intake.suckySuck(), m_intake))
            .onFalse(new RunCommand(
            () -> m_intake.stop(), m_intake));*/
    /*new JoystickButton(m_juliet, Button.kB.value)
        .onTrue(intakeAuto);

    new JoystickButton(m_juliet, Button.kA.value)
        .onTrue(intakeStopAuto);*/

    new JoystickButton(m_juliet, Button.kB.value)
        .toggleOnTrue(new IntakeAuto(m_intake)).whileFalse(new IntakeStopAuto(m_intake));

    new JoystickButton(m_juliet, Button.kX.value)
        .onTrue(new PrepareHangAuto(m_intake, m_climber, m_juliet));//.andThen(new FinishHangAuto(m_intake, m_climber)));



    /*new JoystickButton(m_romeo, Button.kA.value) // changed to Y from right bumper
        .whileTrue(new ParallelCommandGroup(
            new RunCommand(() -> m_intake.feedTheMachine(), m_intake),
            new RunCommand(() -> m_shooter.shoot(5000), m_shooter))
        ).onFalse(new ParallelCommandGroup(
            new RunCommand(() -> m_intake.stop(), m_intake),
            new RunCommand(() -> m_shooter.shoot(0), m_shooter)));*/

    /*new JoystickButton(m_juliet, Button.kA.value)
        .whileTrue(new RunCommand(
            () -> m_intake.feedTheMachine(), m_intake))
            .onFalse(new RunCommand(() -> m_intake.stop(), m_intake));*/


            /*
             * 
             * old intake code
             */
    /*new JoystickButton(m_juliet, Button.kY.value)
        .whileTrue(new RunCommand(
            () -> m_intake.testRotate(false)).withTimeout(2))
            .onFalse(new RunCommand(
                () -> m_intake.rotateStop(), m_intake));

    new JoystickButton(m_juliet, Button.kX.value)
        .whileTrue(new RunCommand(
            () -> m_intake.testRotate(true)).withTimeout(2.5))
            .onFalse(new RunCommand(
                () -> m_intake.rotateStop(), m_intake));*/

    /*new JoystickButton(m_juliet, Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_shooter.shoot(5000), m_shooter))
            .onFalse(new RunCommand(
                () -> m_shooter.stop(), m_shooter));*/





    new JoystickButton(m_juliet, Button.kRightBumper.value)
        .onTrue(new SpeakerCommand(m_shooter, m_intake, m_juliet));

    new JoystickButton(m_juliet, Button.kLeftBumper.value)
        .whileTrue(new HandoffCommand(m_intake, m_shooter));

    new JoystickButton(m_juliet, Button.kStart.value)
        .whileTrue(new SourceIntake(m_intake, m_shooter));
        


          //test from earlier  
    /*new JoystickButton(m_romeo, Button.kA.value)
        .whileTrue(new IntakeCommand(m_intake, m_romeo));

    new JoystickButton(m_romeo, Button.kY.value)
        .whileTrue(new SpeakerCommand(m_shooter, m_intake, m_romeo))
        .onFalse(new RunCommand(
            () -> m_shooter.shoot(0), m_shooter));
    */
    //hand off
    /*new JoystickButton(m_romeo, Button.kB.value)
        .whileTrue(new HandoffCommand(m_intake, m_shooter));
    */
    //shooter elevator commands
    /*
     * 
    new JoystickButton(m_romeo, Button.kY.value) // Right Bumper
        .whileTrue(new RunCommand(
        () -> m_shootElevator.muevete(ShootElevatorConstants.elevatorSetpoint), //for upward motion
        m_shootElevator).onlyIf(init))
        .onFalse(new RunCommand(
            () -> m_shootElevator.stopElevator(StopConstant.stopSetpoint)));
    
    new JoystickButton(m_romeo, Button.kY.value) // Left Bumper
        .whileTrue(new RunCommand(
        () -> m_shootElevator.muevete(-ShootElevatorConstants.elevatorSetpoint), //for downward motion
        m_shootElevator).onlyIf(run))
        .onFalse(new RunCommand(
            () -> m_shootElevator.stopElevator(StopConstant.stopSetpoint)));

    new JoystickButton(m_romeo, Button.kStart.value) // Start Button (no clue where it is)
        .whileTrue(new RunCommand(
        () -> m_shootElevator.rotatePivot(ShootElevatorConstants.pivotSetpoint), //for upward motion
        m_shootElevator))
        .onFalse(new RunCommand(
            () -> m_shootElevator.stopPivot(StopConstant.stopSetpoint)));
    
    new JoystickButton(m_romeo, Button.kBack.value) // Back Button (no clue where this is either)
        .whileTrue(new RunCommand(
        () -> m_shootElevator.rotatePivot(ShootElevatorConstants.pivotSetpoint), //for downward motion
        m_shootElevator))
        .onFalse(new RunCommand(
            () -> m_shootElevator.stopPivot(StopConstant.stopSetpoint)));

    //climber commands
    new JoystickButton(m_romeo, Button.kX.value) // Right Stick pressed in
        .whileTrue(new RunCommand(
        () -> m_climber.moveClimber(ClimbElevatorConstants.elevatorSetpoint), //for upward motion
        m_climber).onlyIf(init))
        .onFalse(new RunCommand(
            () -> m_climber.stop(StopConstant.stopSetpoint)));
    
    new JoystickButton(m_romeo, Button.kX.value) // Left Stick pressed in
        .whileTrue(new RunCommand(
        () -> m_climber.moveClimber(-ClimbElevatorConstants.elevatorSetpoint), //for downward motion
        m_shootElevator).onlyIf(run))
        .onFalse(new RunCommand(
            () -> m_climber.stop(StopConstant.stopSetpoint)));
    */

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {//the machine is alive
    /*Pose2d initPose2d = new Pose2d(0, 0, new Rotation2d(0));
    Translation2d firstTrans = new Translation2d(0.5, 0.5);
    Translation2d secondTrans = new Translation2d(2, -1);
    Pose2d emoPose2d = new Pose2d(0, 0, new Rotation2d(90));
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        initPose2d, 
        List.of(firstTrans),
        emoPose2d,
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, true, false));*/
    return toggle.getSelected();//lucas got bored and is next to me send help
    //SpeakerAuto speaker = new SpeakerAuto(m_shooter, m_intake);
    //testAuto test = new testAuto(m_robotDrive, m_shooter, m_intake);
    /*SequentialCommandGroup test = new SequentialCommandGroup(
        new testAuto(m_robotDrive, m_shooter, m_intake).asProxy()
    );*/

   // return m_testAuto;

  }
}