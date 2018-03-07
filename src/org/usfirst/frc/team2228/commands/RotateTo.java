package org.usfirst.frc.team2228.commands;

import org.usfirst.frc.team2228.robot.SRXDriveBase;

import edu.wpi.first.wpilibj.command.Command;

public class RotateTo extends Command {
	private boolean isDone;
	private SRXDriveBase base;
	private double ToAngle, PowerLvl;
	private boolean isDirectionReversed;
	private boolean isCascadeTurn;
	private double turnAngleDeg;
	private double turnRadiusIn;
	
	public RotateTo(SRXDriveBase base, double _turnAngleDeg, double _turnRadiusIn, double _PowerLvl, boolean _isDirectionReversed, boolean _isCascadeTurn) {
		PowerLvl = _PowerLvl;
		this.base = base;
		isDirectionReversed = _isDirectionReversed;
		isCascadeTurn = _isCascadeTurn;
		turnAngleDeg = _turnAngleDeg;
		turnRadiusIn = _turnRadiusIn;
	}
	
	protected void initialize() {
		
		System.out.println("initialized");
	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
		isDone = !base.turnByEncoderToAngle(turnAngleDeg, turnRadiusIn, PowerLvl, isDirectionReversed, isCascadeTurn);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return isDone;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		System.out.println("Turn Finsihed: " + turnAngleDeg + " degrees");
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		System.out.println(this.getName() + " was Interupted!");
	}
}
