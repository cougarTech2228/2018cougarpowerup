package org.usfirst.frc.team2228.robot;

import org.usfirst.frc.team2228.commands.EncoderTurn;
import org.usfirst.frc.team2228.commands.MoveTo;
import org.usfirst.frc.team2228.commands.PneumaticGrabber;
import org.usfirst.frc.team2228.commands.RotateTo;
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
	private double THISISWRONGSHOULDCALIBRATE = 5.0;
	private Elevator elevator;
	private PneumaticController pneu;
	private double speed = .2;
	private String firstIndex;
	private boolean firstIndexL, firstIndexR;
	private boolean noGameData;
	

	public AutoMaster(SRXDriveBase srxdb, Elevator _elevator, PneumaticController _pneu) {
		pneu = _pneu;
		base = srxdb;
		chooser.addDefault("Left Switch", leftSwitchAuto);
		elevator = _elevator;
		chooser.addObject("Baseline", baseLineAuto);
		chooser.addObject("Right Switch", rightSwitchAuto);
		SmartDashboard.putData("Auto choices", chooser);
		// BOOP BEEP BOP BEEPEDIE BOOP BOP

		// autoSelected = SmartDashboard.getString("Auto Selector",
		// baseLineAuto);
		StringCommand command = new StringCommand(input);
		// command.start();
		firstIndexL = false;
		firstIndexR = false;
		noGameData = false;

	}

	public void init() {
		base.setRightEncPositionToZero();
		base.setLeftEncPositionToZero();

		Cg = new CommandGroup();

		autoSelected = chooser.getSelected();
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		gameData = gameData.replace(" ", "");
		firstIndex = gameData.substring(0, 1);
		if (gameData.length() >= 3) {
			gameData = gameData.substring(0, 3);
		}
		else if(gameData.isEmpty()) {
			noGameData = true;
		}
		SmartDashboard.putString("Game Data", gameData);
		positions = gameData.toCharArray();
		System.out.println("Auto selected: " + autoSelected);
		if (firstIndex.equalsIgnoreCase("l")) {
			System.out.println("L");
			firstIndexL = true;
			firstIndexR = false;
			noGameData = false;
		} else if(firstIndex.equalsIgnoreCase("r")) {
			System.out.println("R");
			firstIndexR = true;
			firstIndexL = false;
			noGameData = false;
		}
		switch (autoSelected) {

		case "Baseline":
			System.out.println("Baseline selected");
			// Adds movement to the auto sequence
			Cg.addSequential(
					new MoveTo(base, (Dimensions.AUTOLINE_TO_ALLIANCE - Dimensions.LENGTH_OF_ROBOT), speed, false));
			break;

		case "Left Switch":
			System.out.println("Left Switch selected");
			// Adds movement to the auto sequence

			Cg.addSequential(new PneumaticGrabber(pneu, true, 0.5));
			// After half a second the bot starts moving
			Cg.addSequential(new MoveTo(base, (Dimensions.AUTOLINE_TO_ALLIANCE - Dimensions.LENGTH_OF_ROBOT + 4.0),
					speed, false));
			// While the bot is moving, it continues closing the aquirer arms for another
			// second and a half
			Cg.addParallel(new PneumaticGrabber(pneu, true, 1.5));
			Cg.addSequential(new RotateTo(base, 50, SRXDriveBaseCfg.kTrackWidthIn + 4, .1, false, true));
			

			// If the left side of the switch is ours, it places the cube, if not, it does
			// nothing
			if (!gameData.isEmpty() && firstIndexL && !noGameData) {
				Cg.addSequential(new PneumaticGrabber(pneu, false, 2.0));
				Cg.addParallel(new Switch(elevator));
			}
			else {
				System.out.println("Incorrect game data");
			}

			// Scale cube command
			break;

		case "Right Switch":
			System.out.println("Right Switch selected");
			// The bot starts closing the aquirer arms for half a second
			Cg.addSequential(new PneumaticGrabber(pneu, true, 0.5));
			// After half a second the bot starts moving
			Cg.addSequential(new MoveTo(base, (Dimensions.SWITCHWALL_TO_ALLIANCESTATION - Dimensions.LENGTH_OF_ROBOT),
					speed, false));
			// While the bot is moving, it continues closing the aquirer arms for another
			// second and a half
			Cg.addParallel(new PneumaticGrabber(pneu, true, 1.5));

			if (!gameData.isEmpty() && firstIndexR && !noGameData) {
				// If the right side of the switch is ours, it places the cube while opening the
				// aquirer arms
				Cg.addSequential(new PneumaticGrabber(pneu, false, 2.0));
				Cg.addParallel(new Switch(elevator));

			}
			else {
				System.out.println("Incorrect game data");
			}

			// Scale cube command
			break;
		}
		Cg.start();
	}

	public void run() {
		// Runs the sequence made in auto init
		Scheduler.getInstance().run();
	}
}
