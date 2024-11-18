package frc.robot.commands.subsytem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.example.SubsystemConstants.SubsystemStates;

/**
 * An example subsystem command
 * 
 * @author Noah Simon
 */
public class SubsystemCommand extends Command {

    public SubsystemCommand() {
        super.addRequirements(Robot.subsystem);
    }

    @Override
    public void initialize() {
        Robot.subsystem.setState(SubsystemStates.GO);
    }

    // if further state change are needed over time, put them here with conditionals
    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interupted) {
        Robot.subsystem.setState(SubsystemStates.STOP);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
