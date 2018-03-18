package org.usfirst.frc.team2228.commands;

import org.usfirst.frc.team2228.robot.CubeManipulator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class AutoRollers extends Command {
	CubeManipulator cube;
	double speed;
	public AutoRollers(CubeManipulator cube, double speed, double timeout) {
		super(timeout);
		this.speed = speed;
		this.cube = cube;
	}
	protected void initialize() {
    	System.out.println("Initialized rollers at " + Timer.getFPGATimestamp());
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	cube.rollerSet(speed);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    	cube.rollerSet(0);
    	System.out.println("Finished at " + Timer.getFPGATimestamp());
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
