package frc.robot.commands.Autos.AutoCommands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class MathHomework extends Command{

    DriveSubsystem drive;
    double count = 0;//t value, ends when it equals or is greater than 1
    
    double[] xPoints = {0, 2.4, 1, 0, -0.8, -1};//the points for the curves
    double[] yPoints = {0, 1.2, 1, 1, 1, 1};


    PIDController xPID = new PIDController(0.04, 0, 0);
    PIDController yPID = new PIDController(0.04, 0, 0);


    private Pose2d zeroPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));


    public MathHomework(DriveSubsystem drive){//quite literally my math homework
        this.drive = drive;

        addRequirements(drive);
    }

    @Override
    public void initialize(){
        drive.zeroHeading();
        drive.resetOdometry(zeroPose);//sets the robots position as zero, not ideal for all situations but for this it is
    }

    @Override
    public void execute(){
        //arc length of 3.5, function called every 0.02 seconds
        //at every 0.02 seconds the count is increased slightly so by the end it equals 1 running at half speed
        //if this is too fast it would start cutting corners and if this was too slow it would be jerky 
        count = count + (0.02)/6;

        double xSpeed = xPID.calculate(drive.getPose().getX(), xCurve());//gets the next position and then relays the current position to calculate where it needs to be next
        double ySpeed = yPID.calculate(drive.getPose().getY(), yCurve());

        drive.drive(xSpeed, ySpeed, 0, true, true, false);

    }

    private double xCurve(){
        double desiredValue = 0;

        desiredValue = //this can be optimized so much
            Math.pow((1 - count), 5) * xPoints[0] 
            + 5 * count * Math.pow((1 - count), 4) * xPoints[1]
            + 10 * count * count * Math.pow(1 - count, 3) * xPoints[2]
            + 10 * Math.pow(count, 3) * Math.pow((1 - count), 2) * xPoints[3]
            + 5 * Math.pow(count, 4) * (1 - count) * xPoints[4]
            + Math.pow(count, 5) * xPoints[5];
        return desiredValue;
    }

    private double yCurve(){
        double desiredValue = 
            Math.pow((1 - count), 5) * yPoints[0] 
            + 5 * count * Math.pow((1 - count), 4) * yPoints[1]
            + 10 * count * count * Math.pow(1 - count, 3) * yPoints[2]
            + 10 * Math.pow(count, 3) * Math.pow((1 - count), 2) * yPoints[3]
            + 5 * Math.pow(count, 4) * (1 - count) * yPoints[4]
            + Math.pow(count, 5) * yPoints[5];
        return desiredValue;
    }

    

    @Override
    public void end(boolean interupted){
        drive.resetOdometry(zeroPose);
    }

    @Override
    public boolean isFinished(){
        return count >= 1;
    }

    
}
