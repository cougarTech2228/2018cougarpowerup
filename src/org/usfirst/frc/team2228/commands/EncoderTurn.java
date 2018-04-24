package org.usfirst.frc.team2228.commands;

import org.usfirst.frc.team2228.robot.SRXDriveBase;

import edu.wpi.first.wpilibj.command.Command;

public class EncoderTurn extends Command {
	private boolean isDone, d, e;
	private SRXDriveBase base;
	private double a,b,c;
	
	public EncoderTurn(SRXDriveBase base, double _turnAngleDeg, double _turnRadiusIn, double _turnPowerLevel, boolean _isDirectionReverse, boolean _isCascadeTurn ) {
		a = _turnAngleDeg;
		b = _turnRadiusIn;
		c = _turnPowerLevel;
		d = _isDirectionReverse;
		e = _isCascadeTurn;
		this.base = base;
	}
	
	protected void initialize() {
		
		System.out.println("initialized");
	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
		isDone = !base.turnByEncoderToAngle(a, b, c, d, e);
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
