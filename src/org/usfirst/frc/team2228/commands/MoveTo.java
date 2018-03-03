package org.usfirst.frc.team2228.commands;

import org.usfirst.frc.team2228.robot.DebugLogger;
import org.usfirst.frc.team2228.robot.SRXDriveBase;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class MoveTo extends Command {
	private boolean isDone;
	private double moveIn, movePwr, startTime, endTime, startLeftCounts, endLeftCounts, startRightCounts, endRightCounts;
	private boolean isCascade;
	private SRXDriveBase base;

	public MoveTo(SRXDriveBase base, double _MoveToPositionIn, double _MoveToPositionPwrLevel, boolean _isCascadeMove, double timeWait) {
		super(timeWait);
		this.base = base;
		moveIn = _MoveToPositionIn;
		movePwr = _MoveToPositionPwrLevel;
		isCascade = _isCascadeMove;
	}

	protected void initialize() {
		startTime = Timer.getFPGATimestamp();
		startLeftCounts = base.getLeftEncoderPosition();
		startRightCounts = base.getRightEncoderPosition();
		System.out.println("initial move: " + moveIn);
		System.out.println("Started moving at " + startTime + " seconds");
		System.out.println("Started moving at " + startLeftCounts + " counts on the left side");
		System.out.println("Started moving at " + startRightCounts + " counts on the right side");
		base.setDriveTrainRamp(4);
		
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
		endLeftCounts = base.getLeftEncoderPosition();
		endRightCounts = base.getRightEncoderPosition();
		endTime = Timer.getFPGATimestamp();
		System.out.println("Stopped moving at " + endTime + " seconds");
		System.out.println("Started moving at " + endLeftCounts + " counts on the left side");
		System.out.println("Started moving at " + endRightCounts + " counts on the right side");

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		System.out.println(this.getName() + " was Interupted!");
	}

}
