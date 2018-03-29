package org.usfirst.frc.team2228.commands;

import org.usfirst.frc.team2228.robot.CubeManipulator;
import org.usfirst.frc.team2228.robot.Dimensions;
import org.usfirst.frc.team2228.robot.Elevator;
import org.usfirst.frc.team2228.robot.SRXDriveBase;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CommandSet {
	private CommandGroup command;
	private Elevator elevator;
	private SRXDriveBase base;
	private CubeManipulator cube;

	public CommandSet(Elevator _elevator, SRXDriveBase _base, CubeManipulator _cube) {
		elevator = _elevator;
		base = _base;
		cube = _cube;
	}

	public void baseline(double speed, CommandGroup cmdGrp) {
		cmdGrp.addSequential(new PneumaticGrabber(cube, true, 0.5));
		cmdGrp.addSequential(
				new MoveTo(base, (Dimensions.AUTOLINE_TO_ALLIANCE - Dimensions.LENGTH_OF_ROBOT), speed, false, 3.0));
	}

	public void driveElevator(double speed, CommandGroup cmdGrp) {
		cmdGrp.addParallel(new ElevatorAuto(elevator, .5, 2.0));
		cmdGrp.addParallel(new MoveTo(base, 19, speed / 2, false, 1.0));
	}

	public void leftSwitchInit(CommandGroup cmdGrp, double speed) {
		cmdGrp.addSequential(new WaitCommand(SmartDashboard.getNumber("Wait Time", 0)));
		System.out.println("Left Switch selected");
		cmdGrp.addSequential(new PneumaticGrabber(cube, true, 0.5));
		cmdGrp.addSequential(new MoveTo(base, (Dimensions.ALLIANCE_WALL_TO_MID_SWITCH - Dimensions.LENGTH_OF_ROBOT),
				speed, false, 4.0));
		cmdGrp.addSequential(new WaitCommand(SmartDashboard.getNumber("Wait Time", 0)));
	}

	public void leftSwitchTurn(CommandGroup cmdGrp, double speed) {
		cmdGrp.addSequential(new RotateTo(base, 90, speed));
		cmdGrp.addSequential(new MoveTo(base, 19, speed / 1.5, false, 2.0));
		score(cmdGrp);
	}

	/**
	 * @param cmdGrp
	 * @param speed
	 *            Initialize the straight right switch auto command
	 */
	public void rightSwitchInit(CommandGroup cmdGrp, double speed) {
		cmdGrp.addSequential(new WaitCommand(SmartDashboard.getNumber("Wait Time", 0)));
		System.out.println("Right Switch selected");
		cmdGrp.addSequential(new PneumaticGrabber(cube, true, 0.5));
		cmdGrp.addSequential(new MoveTo(base, (Dimensions.ALLIANCE_WALL_TO_SWITCH - Dimensions.LENGTH_OF_ROBOT), speed,
				false, 4.0));
	}

	public void rightSwitch(CommandGroup cmdGrp, double speed) {
		score(cmdGrp);
	}

	public void rightSwitchTurnInit(CommandGroup cmdGrp, double speed) {
		cmdGrp.addSequential(new PneumaticGrabber(cube, true, 0.5));
		cmdGrp.addSequential(new MoveTo(base, (Dimensions.ALLIANCE_WALL_TO_MID_SWITCH - Dimensions.LENGTH_OF_ROBOT),
				speed, false, 4.0));
		cmdGrp.addSequential(new WaitCommand(SmartDashboard.getNumber("Wait Time", 0)));
	}

	public void rightSwitchTurnTurn(CommandGroup cmdGrp, double speed) {
		cmdGrp.addSequential(new RotateTo(base, -90, speed));
		cmdGrp.addParallel(new MoveTo(base, 19, speed / 2, false, 2.0));
		score(cmdGrp);
	}

	public void leftScaleInit(CommandGroup cmdGrp) {
		cmdGrp.addSequential(new WaitCommand(SmartDashboard.getNumber("Wait Time", 0)));
		System.out.println("Left Scale selected");
		// Adds movement to the auto sequence
		cmdGrp.addSequential(new PneumaticGrabber(cube, true, 0.5));
	}

	public void leftScaleTurn(CommandGroup cmdGrp, double speed) {
		cmdGrp.addSequential(
				new MoveTo(base, (Dimensions.ALLIANCE_WALL_TO_SCALE - Dimensions.LENGTH_OF_ROBOT) + 8, speed, false, 8.0));
		cmdGrp.addSequential(new ElevatorAuto(elevator, .7, 5.0));
		cmdGrp.addSequential(new RotateTo(base, 60, speed));
//		cmdGrp.addSequential(new CubeRotate(cube, true));
//		cmdGrp.addSequential(new AutoRollers(cube, 1.0, 1.0));
		cmdGrp.addSequential(new MoveTo(base, 12, speed / 2, false, 3.0));
		cmdGrp.addSequential(new PneumaticGrabber(cube, false, 1.0));
		cmdGrp.addSequential(new WaitCommand(1.5));
		backUp(cmdGrp, 12.0, speed / 2);
		cmdGrp.addSequential(new CubeRotate(cube, false));
		cmdGrp.addSequential(new ElevatorAuto(elevator, -.7, 3.0));
		cmdGrp.addSequential(new ElevatorAuto(elevator, -.4, 2.0));
	}

	public void rightScaleInit(CommandGroup cmdGrp) {
		cmdGrp.addSequential(new WaitCommand(SmartDashboard.getNumber("Wait Time", 0)));
		System.out.println("Right Scale selected");
		// Adds movement to the auto sequence
		cmdGrp.addSequential(new PneumaticGrabber(cube, true, 0.5));
	}

	public void rightScaleTurn(CommandGroup cmdGrp, double speed) {
		cmdGrp.addSequential(
				new MoveTo(base, (Dimensions.ALLIANCE_WALL_TO_SCALE - Dimensions.LENGTH_OF_ROBOT), speed, false, 8.0));
		cmdGrp.addSequential(new RotateTo(base, -45, speed));
		cmdGrp.addSequential(new ElevatorAuto(elevator, .7, 5.0));
		cmdGrp.addSequential(new CubeRotate(cube, true));
		cmdGrp.addSequential(new PneumaticGrabber(cube, false, 1.0));
		cmdGrp.addSequential(new WaitCommand(1.5));
		backUp(cmdGrp, 18, speed);
		cmdGrp.addSequential(new CubeRotate(cube, false));
		cmdGrp.addSequential(new ElevatorAuto(elevator, -.7, 4.0));
	}

	public void score(CommandGroup cmdGrp) {
		cmdGrp.addSequential(new ElevatorAuto(elevator, .5, 2.0));
		cmdGrp.addParallel(new CubeRotate(cube, true));
		cmdGrp.addSequential(new PneumaticGrabber(cube, false, 1.0));
		cmdGrp.addSequential(new WaitCommand(1.5));
		cmdGrp.addSequential(new CubeRotate(cube, false));
		cmdGrp.addSequential(new ElevatorAuto(elevator, -.35, 2.0));
	}
	public void backUp(CommandGroup cmdGrp, double distance, double speed) {
		System.out.println("Backing up");
		cmdGrp.addSequential(new MoveTo(base, distance, speed, true, false, 2.0));
		rotateCube(cmdGrp);
	}
	public void rotateCube(CommandGroup cmdGrp) {
		cmdGrp.addSequential(new CubeRotate(cube, true));
	}
}
