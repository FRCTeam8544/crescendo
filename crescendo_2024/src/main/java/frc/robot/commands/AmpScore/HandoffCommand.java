package frc.robot.commands.AmpScore;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class HandoffCommand extends Command{

    IntakeSubsystem intake;
    ShootSubsystem shooter;
    double count = 0;
    XboxController controller;
    public HandoffCommand(IntakeSubsystem intake, ShootSubsystem shooter, XboxController controller){
        this.shooter = shooter;
        this.intake = intake;
        this.controller = controller;
    }

    @Override
    public void initialize(){
        shooter.setBrake();
        System.out.println("handOff Started");
        count = 0;
    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("count", count);
        if (!shooter.noteInShooter.getAsBoolean()){
            intake.rageAgainsTheMachine();
            shooter.handoff();
        }
        else if (count < 7)
            count = count + 1;
        else
            intake.stop();
    }

    @Override
    public void end(boolean interupted){
        intake.stop();
        shooter.setZero();
    }

    @Override
    public boolean isFinished(){
        if ((shooter.noteInShooter.getAsBoolean() && count >= 7) || !controller.getLeftBumper()){
            System.out.println("handOffEnded");
            return true;
        }
        return false;
    }
}
