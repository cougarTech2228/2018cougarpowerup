package org.usfirst.frc.team2228.commands;
import org.usfirst.frc.team2228.robot.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Switch extends Command {
	Elevator elevator;
    public Switch(Elevator _elevator, double timeout) {
    	super(timeout);
    	elevator = _elevator;
    	
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("Initialized rollers at " + Timer.getFPGATimestamp());
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	elevator.conveyors(true);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    	elevator.conveyors(false);
    	System.out.println("Finished at " + Timer.getFPGATimestamp());
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
