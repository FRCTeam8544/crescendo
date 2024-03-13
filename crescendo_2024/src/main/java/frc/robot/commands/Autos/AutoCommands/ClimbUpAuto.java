package frc.robot.commands.Autos.AutoCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberElevator;

public class ClimbUpAuto extends Command{


    ClimberElevator climber;
    XboxController controller;
    public ClimbUpAuto(ClimberElevator climber, XboxController controller){
        this.climber = climber;
        this.controller = controller;
    }

    @Override
    public void initialize(){

    }
    
    @Override
    public void execute(){
        if (!climber.noUp.getAsBoolean() && controller.getRightTriggerAxis() > 0.1){climber.moveClimber(true);}
        else if (!climber.noDown.getAsBoolean() && controller.getLeftTriggerAxis() > 0.1){climber.moveClimber(false);}
        else{climber.stop();}
    }

    @Override
    public void end (boolean Interupted){
        if (Interupted){climber.stop();}
    }
    
    @Override
    public boolean isFinished(){
        return (controller.getRightBumper() && controller.getLeftBumper());
    }
    
}
