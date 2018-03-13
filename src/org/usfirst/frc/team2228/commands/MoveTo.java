package org.usfirst.frc.team2228.commands;
import org.usfirst.frc.team2228.robot.DebugLogger;
import org.usfirst.frc.team2228.robot.SRXDriveBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class MoveTo extends Command {
	private boolean isDone;
	private double moveIn, movePwr;
	private boolean isCascade;
	private SRXDriveBase base;
	private Object startTime;
	private double startLeftCounts;
	private double startRightCounts;
	private double endLeftCounts;
	private double endRightCounts;
	private double endTime;
	private double timeOut;
	
	public MoveTo(SRXDriveBase base, double _MoveToPositionIn, double _MoveToPositionPwrLevel, boolean _isCascadeMove, double _timeOut) {
		super(_timeOut);
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
//		base.setAngleZero();
	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
		isDone = !base.velMoveToPosition(moveIn, movePwr, false, isCascade);
//		isDone = !base.testDriveStraightCalibration(moveIn, movePwr);
//		base.SetDriveTrainCmdLevel(.2, .2);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return isDone || isTimedOut();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		if(isTimedOut()) {
			base.setStopMotors();
			base.setSRXDriveBaseInit();
			System.out.println("stop TIMED OUT!");
		}
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
