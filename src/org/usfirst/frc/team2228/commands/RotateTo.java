package org.usfirst.frc.team2228.commands;

import org.usfirst.frc.team2228.robot.SRXDriveBase;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class RotateTo extends Command {
	private boolean isDone;
	private SRXDriveBase base;
	private double ToAngle, PowerLvl, startTime, endTime;
	
	public RotateTo(SRXDriveBase base, double _ToAngle, double _PowerLvl) {
		ToAngle = _ToAngle;
		PowerLvl = _PowerLvl;
		this.base = base;
	}
	
	protected void initialize() {
		startTime = Timer.getFPGATimestamp();
		System.out.println("Started rotating at " + startTime + " seconds");
	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
		isDone = !base.rotateToAngle(ToAngle, PowerLvl);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return isDone;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		endTime = Timer.getFPGATimestamp();
		System.out.println("Stopped rotating at " + endTime + " seconds");
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		System.out.println(this.getName() + " was Interupted!");
	}
}
