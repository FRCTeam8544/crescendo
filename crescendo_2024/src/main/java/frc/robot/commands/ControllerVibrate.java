package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class ControllerVibrate extends Command{

    XboxController controller;
    XboxController secondController;
    public ControllerVibrate(XboxController controller, XboxController secondController){
        this.controller = controller;
        this.secondController = secondController;
    }

    public void initialize(){

    }

    public void execute(){
        controller.setRumble(RumbleType.kBothRumble, 1);
        secondController.setRumble(RumbleType.kBothRumble, 1);
    }

    public void end(boolean interupted){
        controller.setRumble(RumbleType.kBothRumble, 0);
        secondController.setRumble(RumbleType.kBothRumble, 0);
    }

    public boolean finished(){
        return false;
    }
    
}
