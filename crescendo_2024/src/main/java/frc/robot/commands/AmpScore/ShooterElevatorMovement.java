package frc.robot.commands.AmpScore;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterElevator;

public class ShooterElevatorMovement extends Command{

    ShooterElevator shooterElevator;
    public ShooterElevatorMovement(ShooterElevator shooterElevator){
        this.shooterElevator = shooterElevator;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        if (!shooterElevator.noUp.getAsBoolean()){shooterElevator.moveElevator(true);}
        else{shooterElevator.stopElevator();}
    }
    
    @Override
    public void end(boolean interupted){
        shooterElevator.stopElevator();
    }

    @Override
    public boolean isFinished(){
        return shooterElevator.noUp.getAsBoolean();
    }

}
