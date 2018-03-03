package org.usfirst.frc.team2228.commands;

import org.usfirst.frc.team2228.robot.Elevator;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ElevatorToPortal extends Command {
	Elevator elevator;
	boolean done = false;
	public ElevatorToPortal(Elevator _elevator) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		elevator = _elevator;	
		
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		done = elevator.MoveElevator(2, .5, 1000);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return done;
	}

	// Called once after isFinished returns true
	protected void end() {
		System.out.println("dunzo");
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
