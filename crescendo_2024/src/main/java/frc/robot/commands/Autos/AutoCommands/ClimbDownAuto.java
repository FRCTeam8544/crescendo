package frc.robot.commands.Autos.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberElevator;

public class ClimbDownAuto extends Command{

    ClimberElevator climber;
    public ClimbDownAuto(ClimberElevator climber){
        this.climber = climber;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        if (!climber.noDown.getAsBoolean()){climber.moveClimber(false);}
        else{climber.stop();}
    }

    @Override
    public void end(boolean Interupted){
        if (Interupted){climber.stop();}
    }

    @Override
    public boolean isFinished(){
        return climber.noDown.getAsBoolean();
    }
    
}
