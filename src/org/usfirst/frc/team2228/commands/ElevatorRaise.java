package org.usfirst.frc.team2228.commands;
import org.usfirst.frc.team2228.robot.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class ElevatorRaise extends Command {
	Elevator elevator;
	double height, speed;
	private boolean isDone;
	
    public ElevatorRaise(Elevator _elevator, double _height, double _speed) {
    	height = _height;
    	elevator = _elevator;
    	speed = _speed;
    	
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("Initialized raising/lowering at " + Timer.getFPGATimestamp());
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	isDone = elevator.elevatorSet(height, speed);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isDone;
    }

    // Called once after isFinished returns true
    protected void end() {
    	elevator.elevatorSet(height, 0);
    	System.out.println("Finished raising/lowering at " + Timer.getFPGATimestamp());
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
