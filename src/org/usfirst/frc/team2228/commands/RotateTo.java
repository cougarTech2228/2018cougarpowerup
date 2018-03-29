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
	
	public RotateTo(SRXDriveBase base, double _turnAngleDeg, double _PowerLvl) {
		PowerLvl = _PowerLvl;
		this.base = base;
		turnAngleDeg = _turnAngleDeg;
	}
	
	protected void initialize() {
		//base.setLeftEncPositionToZero();
		//base.setRightEncPositionToZero();
		System.out.println("Turn initialized");
	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
		isDone = !base.rotateToAngle(turnAngleDeg, PowerLvl);
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
