package org.usfirst.frc.team2228.commands;

import org.usfirst.frc.team2228.robot.Elevator;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class LiftRaiseTilLimit extends Command {
	Elevator elevator;
	

    public LiftRaiseTilLimit(Elevator high) {
    	
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	elevator = high;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
