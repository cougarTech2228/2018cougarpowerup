package org.usfirst.frc.team2228.robot;

import org.usfirst.frc.team2228.commands.ElevatorAuto;
import org.usfirst.frc.team2228.commands.EncoderTurn;
import org.usfirst.frc.team2228.commands.MoveTo;
import org.usfirst.frc.team2228.commands.PneumaticGrabber;
import org.usfirst.frc.team2228.commands.RotateTo;
import org.usfirst.frc.team2228.commands.Scale;
import org.usfirst.frc.team2228.commands.StringCommand;
import org.usfirst.frc.team2228.commands.Switch;

import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoMaster {
	final String baseLineAuto = "Baseline";
	final String leftSwitchAuto = "Left Switch";
	final String rightSwitchAuto = "Right Switch";
	private char[] positions;
	private SRXDriveBase base;
	private String autoSelected;
	private String input = "";
	private CommandGroup Cg;
	private SendableChooser<String> chooser = new SendableChooser<>();
	private String robotSide = "Right";
	private double leftAutoMoveToSwitch = 14.81;
	private double THISISWRONGSHOULDCALIBRATE = 5.0;
	private Elevator elevator;
	private PneumaticController pneu;
	private double speed = .2;

	public AutoMaster(SRXDriveBase srxdb, Elevator _elevator, PneumaticController _pneu) {
		pneu = _pneu;
		base = srxdb;
		chooser.addObject("Left Switch", leftSwitchAuto);
		elevator = _elevator;
		chooser.addObject("Baseline", baseLineAuto);
		chooser.addDefault("Right Switch", rightSwitchAuto);
		SmartDashboard.putData("Auto choices", chooser);
		// BOOP BEEP BOP BEEPEDIE BOOP BOP

		// autoSelected = SmartDashboard.getString("Auto Selector",
		// baseLineAuto);
		StringCommand command = new StringCommand(input);
		// command.start();

	}

	public void init() {
		base.setRightEncPositionToZero();
		base.setLeftEncPositionToZero();

		Cg = new CommandGroup();

		autoSelected = chooser.getSelected();
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		positions = gameData.toCharArray();
		System.out.println("Auto selected: " + autoSelected);
		if (gameData.charAt(0) == 'L') {
			System.out.println("L");
		} else {
			System.out.println("R");
		}
		switch (autoSelected) {

		case "Baseline":
			System.out.println("Baseline selected");
			// Adds movement to the auto sequence
			Cg.addSequential(
					new MoveTo(base, (Dimensions.AUTOLINE_TO_ALLIANCE - Dimensions.LENGTH_OF_ROBOT), speed, false));
			break;

		case "Left Switch":

			if (gameData.length() == 0) {
				Cg.addSequential(
						new MoveTo(base, (Dimensions.AUTOLINE_TO_ALLIANCE - Dimensions.LENGTH_OF_ROBOT), speed, false));
				System.out.println("Did not recieve game data");
			}
			System.out.println("Left Switch selected");
			// Adds movement to the auto sequence

			Cg.addSequential(new PneumaticGrabber(pneu, true, 0.5));
			// After half a second the bot starts moving
			Cg.addSequential(new MoveTo(base,
					(Dimensions.SWITCHWALL_TO_ALLIANCESTATION - Dimensions.LENGTH_OF_ROBOT + 4.0), speed, false));
			// While the bot is moving, it continues closing the aquirer arms
			// for another second and a half
			Cg.addParallel(new PneumaticGrabber(pneu, true, 1.5));
			Cg.addSequential(new RotateTo(base, 50, SRXDriveBaseCfg.kTrackWidthIn + 4, .1, false, true));
			// Cg.addSequential(new MoveTo(base, leftAutoMoveToSwitch, speed,
			// false));

			// If the left side of the switch is ours, it places the cube, if
			// not, it does nothing
			if (gameData.charAt(0) == 'L' || gameData.charAt(0) == 'l') {
				Cg.addSequential(new PneumaticGrabber(pneu, false, 2.0));
				Cg.addParallel(new Switch(elevator));
			}

			else if (gameData.charAt(0) == 'R' || gameData.charAt(0) == 'r') {
				Cg.addSequential(new MoveTo(base, -10, .2, false));
			}

			// Scale cube command
			break;

		case "Right Switch":
			if (gameData.length() == 0) {
				Cg.addSequential(
						new MoveTo(base, (Dimensions.AUTOLINE_TO_ALLIANCE - Dimensions.LENGTH_OF_ROBOT), speed, false));
				System.out.println("Did not recieve game data");
			}

			System.out.println("Right Switch selected");
			// The bot starts closing the aquirer arms for half a second
			Cg.addSequential(new PneumaticGrabber(pneu, true, 0.5));
			// After half a second the bot starts moving
			Cg.addSequential(new MoveTo(base, (Dimensions.SWITCHWALL_TO_ALLIANCESTATION - Dimensions.LENGTH_OF_ROBOT),
					speed, false));
			// While the bot is moving, it continues closing the aquirer arms
			// for another
			// second and a half
			Cg.addParallel(new PneumaticGrabber(pneu, true, 1.5));

			if (gameData.charAt(0) == 'R' || gameData.charAt(0) == 'r') {
				// If the right side of the switch is ours, it places the cube
				// while opening the aquirer arms
				Cg.addSequential(new PneumaticGrabber(pneu, false, 2.0));
				Cg.addParallel(new ElevatorAuto(elevator));
				Cg.addParallel(new Switch(elevator));

			} else if (gameData.charAt(0) == 'L' || gameData.charAt(0) == 'l') {
				Cg.addSequential(new MoveTo(base, -10, .2, false));
			}

			// Scale cube command
			break;

		case "Right Scale":
			if (gameData.length() == 0) {
				Cg.addSequential(
						new MoveTo(base, (Dimensions.AUTOLINE_TO_ALLIANCE - Dimensions.LENGTH_OF_ROBOT), speed, false));
				System.out.println("Did not recieve game data");
			}
			System.out.println("Left Switch selected");
			// Adds movement to the auto sequence

			Cg.addSequential(new PneumaticGrabber(pneu, true, 0.5));
			// After half a second the bot starts moving
			Cg.addSequential(new MoveTo(base,
					(Dimensions.ALLIANCE_WALL_TO_SCALE_PLATE_EDGE - Dimensions.LENGTH_OF_ROBOT + 4.0), speed, false));
			// While the bot is moving, it continues closing the aquirer arms
			// for another
			// second and a half
			Cg.addParallel(new PneumaticGrabber(pneu, true, 1.5));
			Cg.addSequential(new RotateTo(base, -50, SRXDriveBaseCfg.kTrackWidthIn + 4, .1, false, true));
			Cg.addSequential(new MoveTo(base, leftAutoMoveToSwitch, speed, false));

			// If the left side of the switch is ours, it places the cube, if
			// not, it does nothing
			if (gameData.charAt(1) == 'R' || gameData.charAt(1) == 'r') {
				Cg.addSequential(new PneumaticGrabber(pneu, false, 2.0));
				Cg.addSequential(new Scale(elevator));
				Cg.addSequential(new Switch(elevator));
			}
			break;

		case "Left Scale":
			if (gameData.length() == 0) {
				Cg.addSequential(
						new MoveTo(base, (Dimensions.AUTOLINE_TO_ALLIANCE - Dimensions.LENGTH_OF_ROBOT), speed, false));
				System.out.println("Did not recieve game data");
			}
			System.out.println("Left Switch selected");
			// Adds movement to the auto sequence

			Cg.addSequential(new PneumaticGrabber(pneu, true, 0.5));
			// After half a second the bot starts moving
			Cg.addSequential(new MoveTo(base,
					(Dimensions.ALLIANCE_WALL_TO_SCALE_PLATE_EDGE - Dimensions.LENGTH_OF_ROBOT + 4.0), speed, false));
			// While the bot is moving, it continues closing the aquirer arms
			// for another
			// second and a half
			Cg.addParallel(new PneumaticGrabber(pneu, true, 1.5));
			Cg.addSequential(new RotateTo(base, 50, SRXDriveBaseCfg.kTrackWidthIn + 4, .1, false, true));
			Cg.addSequential(new MoveTo(base, leftAutoMoveToSwitch, speed, false));

			// If the left side of the switch is ours, it places the cube, if
			// not, it does nothing
			if (gameData.charAt(1) == 'L' || gameData.charAt(1) == 'l') {
				Cg.addSequential(new PneumaticGrabber(pneu, false, 2.0));
				Cg.addSequential(new Scale(elevator));
				Cg.addSequential(new Switch(elevator));
			}
			break;
		}
		Cg.start();
	}

	public void run() {
		// Runs the sequence made in auto init
		Scheduler.getInstance().run();
	}
}
