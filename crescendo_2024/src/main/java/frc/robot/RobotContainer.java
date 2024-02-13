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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimbElevatorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShootElevatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.StopConstant;
import frc.robot.subsystems.ClimberElevator;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.ShooterElevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand; 
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

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

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);



  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
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

    new JoystickButton(m_driverController, Button.kStart.value)
        .whileTrue(new RunCommand(
        () -> m_robotDrive.zeroHeading(), m_robotDrive));

    //shooter commands
    new JoystickButton(m_driverController, Button.kA.value) //A Button
        .whileTrue(new RunCommand(
            () -> m_shooter.shoot(ShooterConstants.shootSetpoint),
            m_shooter))
            .onFalse(new RunCommand(
            () -> m_shooter.stop(StopConstant.stopSetpoint), m_shooter));

    /* This is a kill switch that could be removed whenever, but it probably shouldnt be for now */
    new JoystickButton(m_driverController, Button.kB.value) // B Button
        .whileTrue(new RunCommand(
        () -> m_shooter.stop(StopConstant.stopSetpoint),
        m_shooter));

    /*
        Likely uneeded since we won't intake via shooter anymore

    new JoystickButton(m_driverController, Button.k-.value) X button now conflicts with intaking fron ground,
        .whileTrue(new RunCommand(                          which is worth noting if we ever re-implement this
        () -> m_shooter.sourceIntake(ShooterConstants.intakeSetpoint), 
            m_shooter))
                .onFalse(new RunCommand(
                () -> m_shooter.stopMovement(StopConstant.stopSetpoint), m_shooter));
    */

    //intake commands
    new JoystickButton(m_driverController, Button.kX.value) // changed to X from left bumper
        .whileTrue(new RunCommand(
            () -> m_intake.suckySuck()))
            .onFalse(new RunCommand(
            () -> m_intake.stop()));
    
    new JoystickButton(m_driverController, Button.kY.value) // changed to Y from right bumper
        .whileTrue(new RunCommand(
            () -> m_intake.feedTheMachine()))
            .onFalse(new RunCommand(
            () -> m_intake.stop()));

    //shooter elevator commands
    new JoystickButton(m_driverController, Button.kRightBumper.value) // Right Bumper
        .whileTrue(new RunCommand(
        () -> m_shootElevator.muévete(ShootElevatorConstants.elevatorSetpoint), //for upward motion
        m_shootElevator))
        .onFalse(new RunCommand(
            () -> m_shootElevator.stopElevator(StopConstant.stopSetpoint)));
    
    new JoystickButton(m_driverController, Button.kLeftBumper.value) // Left Bumper
        .whileTrue(new RunCommand(
        () -> m_shootElevator.muévete(-ShootElevatorConstants.elevatorSetpoint), //for downward motion
        m_shootElevator))
        .onFalse(new RunCommand(
            () -> m_shootElevator.stopElevator(StopConstant.stopSetpoint)));

    new JoystickButton(m_driverController, Button.kStart.value) // Start Button (no clue where it is)
        .whileTrue(new RunCommand(
        () -> m_shootElevator.rotatePivot(ShootElevatorConstants.pivotSetpoint), //for upward motion
        m_shootElevator))
        .onFalse(new RunCommand(
            () -> m_shootElevator.stopPivot(StopConstant.stopSetpoint)));
    
    new JoystickButton(m_driverController, Button.kBack.value) // Back Button (no clue where this is either)
        .whileTrue(new RunCommand(
        () -> m_shootElevator.rotatePivot(-ShootElevatorConstants.pivotSetpoint), //for downward motion
        m_shootElevator))
        .onFalse(new RunCommand(
            () -> m_shootElevator.stopPivot(StopConstant.stopSetpoint)));

    //climber commands
    new JoystickButton(m_driverController, Button.kRightStick.value) // Right Stick pressed in
        .whileTrue(new RunCommand(
        () -> m_climber.moveClimber(ClimbElevatorConstants.elevatorSetpoint), //for upward motion
        m_climber))
        .onFalse(new RunCommand(
            () -> m_climber.stop(StopConstant.stopSetpoint)));
    
    new JoystickButton(m_driverController, Button.kLeftStick.value) // Left Stick pressed in
        .whileTrue(new RunCommand(
        () -> m_climber.moveClimber(-ClimbElevatorConstants.elevatorSetpoint), //for downward motion
        m_shootElevator))
        .onFalse(new RunCommand(
            () -> m_climber.stop(StopConstant.stopSetpoint)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
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
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,true, false));
  }
}