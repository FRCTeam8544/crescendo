package frc.robot.commands.AmpScore;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterElevator;

public class MovePivotOut extends Command{

    ShooterElevator pivot;
    public MovePivotOut(ShooterElevator pivot){
        this.pivot = pivot;
    }

    @Override
    public void initialize(){
        System.out.println("pivot Out Initialized");
    }

    @Override
    public void execute(){
        if (pivot.getPivotEncoder() < 0.28){
            pivot.movePivotWithSpeed(true, 0.2);
        }else{
            pivot.stopPivot();
        }
    }

    @Override
    public void end(boolean interupted){
        System.out.println("pivot out finished");
        pivot.stopPivot();
    }

    @Override
    public boolean isFinished(){
        return pivot.getPivotEncoder() > 0.28 || pivot.getPivotEncoder() > 0.9;
    }
    
}
