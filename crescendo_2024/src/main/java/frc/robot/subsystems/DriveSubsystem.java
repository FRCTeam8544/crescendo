// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;
  private double xPos, yPos;

  private double kXPos =  3.24, kYPos = 7.31; // found by (|ln(0.1)/a^2|)^(1/2) where a is the x value for tappering off
  private double speedCapX, speedCapY;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  private boolean imaginaryBox = false;


  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle() * -1),
      //Rotation2d.fromDegrees(0),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    //m_gyro.reset();
  }

  

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle() * -1),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle() * -1),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   * 
   * @param imaginaryBox  box for demos
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    SmartDashboard.putNumber("Y Position", yPos);
    SmartDashboard.putNumber("X Position", xPos);
    SmartDashboard.putNumber("X Max Speed", speedCapX);
    SmartDashboard.putNumber("Y Max Speed", speedCapY);

    SmartDashboard.putBoolean("Imaginary Box", imaginaryBox);


    if (imaginaryBox){
      xSpeed = xSpeed/4;
      ySpeed = ySpeed/4;

      xPos = m_odometry.getPoseMeters().getTranslation().getX(); 
      yPos = m_odometry.getPoseMeters().getTranslation().getY();

      speedCapX = Math.pow(2.72, (-1 * kXPos * Math.pow(xPos, 2))) / 3;
      speedCapY = Math.pow(2.72, (-1 * kYPos * Math.pow(yPos, 2))) / 3;
      //xSpeed = xSpeed > 0? Math.pow(Math.E, -1 * Math.pow(kXPos * m_odometry.getPoseMeters().getX(), 2)) :
      /*if (xSpeed > 0){
        xSpeed = xPos > 0? Math.pow(2.7, -1 * Math.pow(kXPos * xPos, 2)): xSpeed;
      }else{
        xSpeed = xPos < 0? Math.pow(2.7, -1 * Math.pow(kXNeg * xPos, 2)): xSpeed;
      }*/

      /*if (ySpeed > 0){
        ySpeed = yPos > 0? Math.pow(2.7, -1 * Math.pow(kYPos * yPos, 2)): ySpeed;
      }else{
        ySpeed = yPos < 0? Math.pow(2.7, -1 * Math.pow(kYNeg * yPos, 2)): ySpeed;
      }*/
      //ySpeed = Math.pow(2.7, -1 * Math.pow(kYPos * yPos, 2));
      //if ((xSpeed > 0 && xPos > 0) || (xSpeed < 0 && xPos < 0)){
        //xSpeed = Math.abs(xSpeed) > Math.abs(Math.pow(2.7, -1 * Math.pow(kXPos * xPos, 2)))? Math.pow(2.7, -1*Math.pow(kXPos * xPos, 2)): xSpeed;
      //}
      //if ((ySpeed > 0 && yPos > 0) || (ySpeed < 0 && yPos < 0)){
        //ySpeed = Math.abs(ySpeed) > Math.abs(Math.pow(2.7, -1 * Math.pow(kYPos * yPos, 2)))? Math.pow(2.7, -1*Math.pow(kYPos * yPos, 2)): ySpeed;
      //}

      //xSpeed = (xSpeed > 0) && xSpeed > Math.pow(2.7, -1 * Math.pow(kXPos * xPos, 2))? Math.pow(2.7, -1*Math.pow(kXPos * xPos, 2)): xSpeed;
      //xSpeed = (xSpeed < 0) && xSpeed < Math.pow(2.7, -1 * Math.pow(kXPos * xPos, 2))? Math.pow(2.7, -1*Math.pow(kXPos * xPos, 2)): xSpeed;
      //ySpeed = ySpeed * Math.pow(2.7, -1*Math.pow(kYPos * yPos, 2));
      //xSpeed = xSpeed * Math.pow(2.7, -1*Math.pow(kXPos * xPos, 2));

      //ySpeed = (ySpeed > 0) && ySpeed > Math.pow(2.7, -1 * Math.pow(kYPos * yPos, 2))? Math.pow(2.7, -1*Math.pow(kYPos * yPos, 2)): ySpeed;
      //ySpeed = (ySpeed < 0) && ySpeed < Math.pow(2.7, -1 * Math.pow(kYPos * yPos, 2))? Math.pow(2.7, -1*Math.pow(kYPos * yPos, 2)): ySpeed;

      if (xPos < 0 && xSpeed * -1 > speedCapX){
        xSpeed = speedCapX * -1;
      }else if (xPos > 0 && xSpeed > speedCapX){
        xSpeed = speedCapX;
      }
      if (yPos < 0 && ySpeed * -1 > speedCapY){
        ySpeed = speedCapY * -1;
      }else if (yPos > 0 && ySpeed > speedCapY){
        ySpeed = speedCapY;
      }
      
      if (Math.abs(xSpeed) < 0.007){
        xSpeed = 0;
      }
      if (Math.abs(ySpeed) < 0.007){
        ySpeed = 0;
      }

    }
    double xSpeedCommanded;
    double ySpeedCommanded;

    rot = setCurve(rot);

    /*if (opController.getAButton()){
      xSpeed = opController.getLeftX();
      ySpeed = opController.getLeftY();
      rot = opController.getRightX();
    }*/
    
    

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(m_gyro.getAngle() * -1))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  private double setCurve(double input){
    return input > 0? Math.atan((3/2) * (Math.pow(input, 2))): Math.atan((3/2) * (Math.pow(input, 2))) * -1;
  }

  
  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle() * -1).getDegrees();
  }

  public void toggleBox(){
    //my aura is immense 
    imaginaryBox ^= true;
    //(this was easily +10,000,000,000)
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}