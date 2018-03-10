package org.usfirst.frc.team2228.commands;

import org.usfirst.frc.team2228.robot.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ElevatorAuto extends Command {
	Elevator elevator;
	Timer t = new Timer();
	boolean done = false;

	public ElevatorAuto(Elevator _elevator) {
		super(3.0);
		elevator = _elevator;

		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		
		System.out.println("Initialized elevator at " + Timer.getFPGATimestamp());
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		t.start();
		elevator.elevatorSet(-.3);
		if(t.get() >= 3.0){
			elevator.elevatorSet(0.05);
			done = true;
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return done;
	}

	// Called once after isFinished returns true
	protected void end() {
		System.out.println("Finished at " + Timer.getFPGATimestamp());
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
