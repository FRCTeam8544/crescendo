package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

    SwerveModuleSubsystem frontLeftModule;
    SwerveModuleSubsystem frontRightModule;
    SwerveModuleSubsystem backLeftModule;
    SwerveModuleSubsystem backRightModule;

    
    
    public DriveSubsystem(){
        frontLeftModule = new SwerveModuleSubsystem()

    }


    public void drive(double LeftX, double LeftY, double RightX, double RightY){
        

    }
    

    public void setX(){

    }
}
