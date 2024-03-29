package frc.robot.commands.AmpScore;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterElevator;

public class MovePivotIn extends Command{

    ShooterElevator pivot;
    public MovePivotIn(ShooterElevator pivot){
        this.pivot = pivot;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

        if (pivot.getPivotEncoder() > 0.01){
            pivot.movePivotWithSpeed(false, 0.2);
        }else{
            pivot.stopPivot();
        }

    }

    @Override
    public void end(boolean interupted){
        pivot.stopPivot();
    }

    @Override
    public boolean isFinished(){
        return pivot.getPivotEncoder() < 0.01;
    }
    
}
