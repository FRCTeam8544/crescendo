package frc.robot.commands.Autos.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class NewDriveAuto extends Command{

    DriveSubsystem drive;
    double desiredRot, desiredX, desiredY;
    PIDController xPID, yPID, rotPID;
    double x, y, rot;
    public NewDriveAuto(DriveSubsystem drive, double desiredX, double desiredY, double desiredRot){
        this.drive = drive;
        this.desiredRot = desiredRot;
        this.desiredX = desiredX;
        this.desiredY = desiredY;
    }

    @Override
    public void initialize(){
        xPID = new PIDController(AutoConstants.kPx, AutoConstants.kIx, AutoConstants.kDx);
        yPID = new PIDController(AutoConstants.kPy, AutoConstants.kIy, AutoConstants.kDy);
        rotPID = new PIDController(AutoConstants.kProt, AutoConstants.kProt, AutoConstants.kProt);

        xPID.setTolerance(AutoConstants.kTolx);
        yPID.setTolerance(AutoConstants.kToly);
        rotPID.setTolerance(AutoConstants.kTolrot);
        drive.zeroHeading();
    }

    @Override
    public void execute(){
        x = xPID.calculate(drive.getPose().getX(), desiredX);
        y = xPID.calculate(drive.getPose().getY(), desiredY);
        rot = rotPID.calculate(drive.getPose().getRotation().getDegrees(), desiredRot);
        drive.drive(desiredY ,desiredX , desiredRot, true, true);
    }

    @Override
    public void end(boolean interupted){}

    @Override
    public boolean isFinished(){
        return (xPID.atSetpoint() && yPID.atSetpoint() && rotPID.atSetpoint());
    }
    
}
