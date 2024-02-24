package frc.robot.commands.SpeakerScore;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class SpeakerCommand extends Command{
    

    ShootSubsystem shooter;
    XboxController controller;
    IntakeSubsystem intake;
    public SpeakerCommand(ShootSubsystem shooter,IntakeSubsystem intake, XboxController controller){
        this.shooter = shooter;
        this.controller = controller;
        this.intake = intake;
    }


    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        if (controller.getRightTriggerAxis() > 0){
            shooter.shoot(5000);
            if (shooter.atSpeed.getAsBoolean()){intake.feedTheMachine();}
        }
        else if (controller.getLeftTriggerAxis() > 0){
            shooter.shoot(5000);
        }
    }

    @Override
    public void end(boolean interupted){
        if (interupted){
            shooter.stop();
            intake.stop();
        }
    }

    @Override
    public boolean isFinished(){
        if (controller.getRightBumperReleased()){
            shooter.stop();
            intake.stop();
            return true;
        }
        return false;
    }
}
