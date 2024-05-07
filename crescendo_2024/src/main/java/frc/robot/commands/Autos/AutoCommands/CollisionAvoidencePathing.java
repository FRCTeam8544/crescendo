package frc.robot.commands.Autos.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class CollisionAvoidencePathing extends Command{

    double[] baseXPoints = {0, 0.3, 0, 1, 2, 0};
    double[] baseYPoints = {0 , 1, 1, 2.5, 2.5, 3};

    DriveSubsystem m_drive;

    double count = 0;
    boolean avoidRight;

    Pose2d zeroPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

    PIDController xPid = new PIDController(0.04, 0, 0);
    PIDController yPID = new PIDController(0.04, 0, 0);


    public CollisionAvoidencePathing(DriveSubsystem m_drive, boolean avoidRight){

        this.m_drive = m_drive;
        this.avoidRight = avoidRight;

        addRequirements(m_drive);
    }

    @Override
    public void initialize(){
        if (!avoidRight){//causes trajectory to curve towards the left
            for (int i = 0; i < baseXPoints.length; i++){
                baseXPoints[i] = -1 * baseXPoints[i];
            }
        }

        m_drive.resetOdometry(zeroPose);
    }

    @Override
    public void execute(){
        count = count + (0.02/5);

        m_drive.drive(
            xPid.calculate(xCurve(), m_drive.getPose().getX()), 
            yPID.calculate(yCurve(), m_drive.getPose().getY()), 
            0, false, true, false);

    }

    private double xCurve(){
        double desiredValue = 0;

        desiredValue = //this can be optimized so much
            Math.pow((1 - count), 5) * baseXPoints[0] 
            + 5 * count * Math.pow((1 - count), 4) * baseXPoints[1]
            + 10 * count * count * Math.pow(1 - count, 3) * baseXPoints[2]
            + 10 * Math.pow(count, 3) * Math.pow((1 - count), 2) * baseXPoints[3]
            + 5 * Math.pow(count, 4) * (1 - count) * baseXPoints[4]
            + Math.pow(count, 5) * baseXPoints[5];
        return desiredValue;
    }

    private double yCurve(){
        double desiredValue = 
            Math.pow((1 - count), 5) * baseYPoints[0] 
            + 5 * count * Math.pow((1 - count), 4) * baseYPoints[1]
            + 10 * count * count * Math.pow(1 - count, 3) * baseYPoints[2]
            + 10 * Math.pow(count, 3) * Math.pow((1 - count), 2) * baseYPoints[3]
            + 5 * Math.pow(count, 4) * (1 - count) * baseYPoints[4]
            + Math.pow(count, 5) * baseYPoints[5];
        return desiredValue;
    }

    @Override
    public void end(boolean interupted){

    }

    @Override
    public boolean isFinished(){
        return count >= 1;
    }
}
