// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/** 
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 0.8; // radians per second originally 1.2
    public static final double kMagnitudeSlewRate = 1.5; // percent per second (1 = 100%) originally 1.8
    public static final double kRotationalSlewRate = 1.2; // percent per second (1 = 100%)originally 2

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = Math.PI/2;//was negative
    public static final double kFrontRightChassisAngularOffset = Math.PI;//was 0
    public static final double kBackLeftChassisAngularOffset = 0;//pi/2
    public static final double kBackRightChassisAngularOffset = -Math.PI/2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 5;
    public static final int kRearLeftDrivingCanId = 7;
    public static final int kFrontRightDrivingCanId = 6;
    public static final int kRearRightDrivingCanId = 8;

    public static final int kFrontLeftTurningCanId = 1;
    public static final int kRearLeftTurningCanId = 3;
    public static final int kFrontRightTurningCanId = 2;
    public static final int kRearRightTurningCanId = 4;

    public static final boolean kGyroReversed = true;
  }

  public static final class visionConstants {
    /*
     * copied and pasted from https://docs.photonvision.org/en/latest/docs/examples/simposeest.html
     * should be reviewed
     */
    // See
    // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
    // page 208
    public static final double targetWidth =
            Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70); // meters

    // See
    // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
    // page 197
    public static final double targetHeight =
            Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // meters

    // See https://firstfrc.blob.core.windows.net/frc2020/PlayingField/LayoutandMarkingDiagram.pdf
    // pages 4 and 5
    public static final double kFarTgtXPos = Units.feetToMeters(54);
    public static final double kFarTgtYPos =
            Units.feetToMeters(27 / 2) - Units.inchesToMeters(43.75) - Units.inchesToMeters(48.0 / 2.0);
    public static final double kFarTgtZPos =
            (Units.inchesToMeters(98.19) - targetHeight) / 2 + targetHeight;

    public static final Pose3d kFarTargetPose =
            new Pose3d(
                    new Translation3d(kFarTgtXPos, kFarTgtYPos, kFarTgtZPos),
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(180)));

    public static final Transform3d kCameraToRobot = new Transform3d();
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public final static double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 40; // amps
    public static final int kTurningMotorCurrentLimit = 40; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class StopConstant {
    public final static double stopSetpoint = 0;
  }

  public static final class ShooterConstants {
    public static final int leftMotorCANID = 11, rightMotorCANID = 12;
    public static final int limitSwitchPort = 3;

    public static final double handoffSpeed = 0.15;
    public static final double sourceSpeed = 0.25;

    public static final double kP = 0.000005;
    public static final double kI = 5e-7;
    public static final double kD = 0.0005;
    public final double kF = 0.0002;

    public final static double shootSetpoint = 5000;
    public final static double intakeSetpoint = -100; // should be negative, spinning in reverse for intake
  }

  public static final class IntakeConstants {
    public static final int ArmCANID = 13;
    public static final int RollerCANID = 14;
    public static final int IntakeLimitSwPort = 1;
    public static final int NoteLimitSwitchPort = 2;

    public static final double intakeCommandExtendTimeout = 1;
    public static final double intakeCommandRetractTimeout = 1;
    

    public static final double suckySuckSpeed = -.95; //percentage please
    public static final double rateMachineIsFed = .95; //percentage please

    //Values are 0 where tuning is required
    public static final double armkP = 0;
    public static final double armkI = 0;
    public static final double armkD = 0;
    public static final double armkF = 0;
  }

  public static final class ShootElevatorConstants {
    public static final int ElevatorCANID = 17;
    public static final int PivotCANID = 18;
    //pivot is what im calling the mechanism to aim the shooter up or down angularly

    //Values are 0 where tuning is required
    public static final double elevatorSetpoint = 0; //levantanse por favor
    public static final double pivotSetpoint = 0;

    public static final double elevkP = 0;
    public static final double elevkI = 0;
    public static final double elevkD = 0;
    public static final double elevkF = 0;

    public static final double pivotkP = 0;
    public static final double pivotkI = 0;
    public static final double pivotkD = 0;
    public static final double pivotkF = 0;
  }

  public static final class ClimbElevatorConstants {
    public static final int LeftElevatorCANID = 15;
    public static final int RightElevatorCANID = 16;
    public static final int DownLimitPort = 8;
    public static final int UpLimitPort = 9;
    public static final int CountsPerRev = 8192;

    //Values are 0 where tuning is required
    public static final double elevatorSetpoint = 0; //levantanse por favor
    public static final double elevatorRightSetpoint = elevatorSetpoint;
    public static final double elevatorPercentMove = 0.2;
    public static final double elevkP = 0;
    public static final double elevkI = 0;
    public static final double elevkD = 0;
    public static final double elevkF = 0;
  }
}