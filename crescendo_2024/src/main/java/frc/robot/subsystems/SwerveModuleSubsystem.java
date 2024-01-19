package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModuleSubsystem extends SubsystemBase{

    SparkPIDController swervePID;

    CANSparkMax sparkywarky;
    CANSparkMax iDrive;

    AbsoluteEncoder uwuEncoder;

    Constants.ModuleConstants constants;

    public void swerveModuleSubsystem(int canIDS, int canIDD, double offset){

        sparkywarky = new CANSparkMax(canIDS, MotorType.kBrushless);
        iDrive = new CANSparkMax(canIDD, MotorType.kBrushless);

        uwuEncoder = sparkywarky.getAbsoluteEncoder(Type.kDutyCycle);

        swervePID = sparkywarky.getPIDController();        

        uwuEncoder.setZeroOffset(offset);


        swervePID.setD(constants.kDrivingD);
        swervePID.setP(constants.kDrivingP);
        swervePID.setI(constants.kDrivingI);
    }


    public void IDrive(double speed){
        iDrive.set(speed);
    }

    public void SlitherySnake (double setPoint){
        swervePID.set
        sparkywarky.set(swervePID.);
    }
    
}
