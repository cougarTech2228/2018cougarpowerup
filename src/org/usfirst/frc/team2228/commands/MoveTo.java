package org.usfirst.frc.team2228.commands;

import org.usfirst.frc.team2228.robot.DebugLogger;
import org.usfirst.frc.team2228.robot.SRXDriveBase;

import edu.wpi.first.wpilibj.command.Command;

public class MoveTo extends Command {
	private boolean isDone;
	private double moveIn, movePwr;
	private boolean isCascade;
	private SRXDriveBase base;
	
	public MoveTo(SRXDriveBase base, double _MoveToPositionIn, double _MoveToPositionPwrLevel, boolean _isCascadeMove) {
		this.base = base;
		moveIn = _MoveToPositionIn;
		movePwr = _MoveToPositionPwrLevel;
		isCascade = _isCascadeMove;
	}
	
	protected void initialize() {
		System.out.println("initialized");
	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
		isDone = !base.velMoveToPosition(moveIn, movePwr, isCascade);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return isDone;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		System.out.println(this.getName() + " was Interupted!");
	}

}
