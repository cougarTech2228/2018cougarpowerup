package org.usfirst.frc.team2228.commands;

import org.usfirst.frc.team2228.robot.CubeManipulator;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class CubeRotate extends Command {
	private CubeManipulator cube;
	private boolean state;
    public CubeRotate(CubeManipulator cube, boolean state) {
    	this.cube = cube;
    	this.state = state;
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	cube.liftSet(state);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(cube.lift.get() == state) {
    		return true;
    	}
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
