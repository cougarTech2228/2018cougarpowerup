package org.usfirst.frc.team2228.commands;

import org.usfirst.frc.team2228.robot.CubeManipulator;
import org.usfirst.frc.team2228.robot.DebugLogger;
import org.usfirst.frc.team2228.robot.RobotMap;
import org.usfirst.frc.team2228.robot.SRXDriveBase;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class PneumaticGrabber extends Command {
		private CubeManipulator cube;
		private boolean grabbing;
	
	
	public PneumaticGrabber(CubeManipulator _cube, boolean _grab, double timeWait) {
		//It's actually ahhhhhhhhhhhhhhhhhhh
		super(timeWait);
		cube = _cube;
		grabbing = _grab;
	}
	
	protected void initialize() {
		if(grabbing == true){
			System.out.println("Grabbing Started At " + Timer.getFPGATimestamp());
			
		}
		else{
			System.out.println("Not Grabbing At " + Timer.getFPGATimestamp());
		}
	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
		cube.squeeze(grabbing);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return isTimedOut();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		if(grabbing == true){
			System.out.println("Grabbing Stopped At " + Timer.getFPGATimestamp());
		}
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}

}
