package frc.robot.commands.Autos.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
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
        xPID = new PIDController(0.4, 0, 0);
        yPID = new PIDController(0.4, 0, 0);
        rotPID = new PIDController(0.01, 0, 0);

        xPID.setTolerance(0.1);
        yPID.setTolerance(0.1);
        rotPID.setTolerance(0.1);
        drive.zeroHeading();
    }

    @Override
    public void execute(){
        x = xPID.calculate(drive.getPose().getX(), desiredX);
        y = xPID.calculate(drive.getPose().getY(), desiredY);
        rot = rotPID.calculate(drive.getPose().getRotation().getDegrees(), desiredRot);
        drive.drive(desiredY ,desiredX , desiredRot, true, true, false);
    }

    @Override
    public void end(boolean interupted){

    }

    @Override
    public boolean isFinished(){
        return (xPID.atSetpoint() && yPID.atSetpoint() && rotPID.atSetpoint());
    }
    
}
