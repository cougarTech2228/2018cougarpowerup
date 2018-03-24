package org.usfirst.frc.team2228.robot;

import org.usfirst.frc.team2228.commands.AutoRollers;
import org.usfirst.frc.team2228.commands.CommandSet;
import org.usfirst.frc.team2228.commands.CubeRotate;
import org.usfirst.frc.team2228.commands.ElevatorAuto;
import org.usfirst.frc.team2228.commands.EncoderTurn;
import org.usfirst.frc.team2228.commands.MoveTo;
import org.usfirst.frc.team2228.commands.PneumaticGrabber;
import org.usfirst.frc.team2228.commands.RotateTo;
import org.usfirst.frc.team2228.commands.StringCommand;
import org.usfirst.frc.team2228.commands.TurnTo;

import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoMaster {
	final String baseLineAuto = "Baseline";
	final String leftSwitchAuto = "Left Switch";
	final String rightSwitchAuto = "Right Switch";
	final String rightSwitchTurnAuto = "Right Switch Turn";
	final String leftScale = "Left Scale";
	final String rightScale = "Right Scale";
	private SRXDriveBase base;
	private String autoSelected;
	private String input = "";
	private CommandGroup Cg = null;
	private SendableChooser<String> chooser = new SendableChooser<>();
	private Elevator elevator;
	private CubeManipulator cube;
	private double speed = .4;
	private String firstIndex;
	private String secondIndex;
	private CommandSet cmdSet;

	private enum GameData {
		firstIndexL, firstIndexR, noGameData;
	}

	private enum GameData2 {
		noGameData, secondIndexR, secondIndexL;
	}

	private GameData data = GameData.noGameData;
	private GameData2 scaleData = GameData2.noGameData;

	public AutoMaster(SRXDriveBase srxdb, Elevator _elevator, CubeManipulator _cube) {
		cube = _cube;
		base = srxdb;
		chooser.addDefault("Left Switch", leftSwitchAuto);
		elevator = _elevator;
		chooser.addObject("Baseline", baseLineAuto);
		chooser.addObject("Right Switch", rightSwitchAuto);
		chooser.addObject("Right Switch Turn", rightSwitchTurnAuto);
		chooser.addObject(leftScale, leftScale);
		chooser.addObject(rightScale, rightScale);
		SmartDashboard.putData("Auto choices", chooser);
		SmartDashboard.putNumber("Wait Time", 0);
		cmdSet = new CommandSet(elevator, base, cube);
		// BOOP BEEP BOP BEEPEDIE BOOP BOP

		// autoSelected = SmartDashboard.getString("Auto Selector",
		// baseLineAuto);
		StringCommand command = new StringCommand(input);

		// command.start();

	}

	public void teleopInit() {
		if (Cg != null) {
			Cg.cancel();
			System.out.println("auto master cancel...");
		}
	}

	public void init() {
		Scheduler.getInstance().removeAll();
		Cg = new CommandGroup();

		base.setSRXDriveBaseInit();
		autoSelected = chooser.getSelected();
		String gameData = "";
		gameData += DriverStation.getInstance().getGameSpecificMessage();
		gameData = gameData.replace(" ", "");

		if (gameData.isEmpty()) {
			firstIndex = "";
			System.out.println("No game data");
		} else {
			firstIndex = gameData.substring(0, 1);
			secondIndex = gameData.substring(1, 2);
		}
		if (gameData.length() >= 3) {
			gameData = gameData.substring(0, 3);
		}

		SmartDashboard.putString("Game Data", gameData);
		System.out.println("Auto selected: " + autoSelected);
		if (firstIndex.equalsIgnoreCase("l")) {
			System.out.println("L");
			data = GameData.firstIndexL;
		} else if (firstIndex.equalsIgnoreCase("r")) {
			System.out.println("R");
			data = GameData.firstIndexR;
		}
		if (secondIndex.equalsIgnoreCase("L")) {
			scaleData = GameData2.secondIndexL;
		} else if (secondIndex.equalsIgnoreCase("R")) {
			scaleData = GameData2.secondIndexR;
		}
		switch (autoSelected) {
		case "Baseline":
		cmdSet.baseline(speed, Cg);
			break;

		case "Left Switch":
			cmdSet.leftSwitchInit(Cg, speed);
			if (data == GameData.firstIndexL) {
				cmdSet.leftSwitchTurn(Cg, speed);
			} else {
				System.out.println("Incorrect game data");
			}

			// Scale cube command
			break;

		case "Right Switch":
			cmdSet.rightSwitchInit(Cg, speed);
			if (data == GameData.firstIndexR) {
				cmdSet.rightSwitch(Cg, speed);
			} else {
				System.out.println("Incorrect game data");
			}

			// Scale cube command
			break;
		case "Right Switch Turn":
			cmdSet.rightSwitchTurnInit(Cg, speed);
			if (data == GameData.firstIndexR) {
				cmdSet.rightSwitchTurnTurn(Cg, speed);
			} else {
				System.out.println("Incorrect game data");
			}
			// Scale cube command
			break;
		case leftScale:
			cmdSet.leftScaleInit(Cg);
			if (scaleData == GameData2.secondIndexL) {
				cmdSet.leftScaleTurn(Cg, speed);
			} else if (data == GameData.firstIndexL) {
				cmdSet.leftSwitchTurn(Cg, speed);
			} else {
				System.out.println("Incorrect game data");
			}
			// Cg.addSequential(new MoveTo(base, -6.0, speed, false));
			break;
		case rightScale:
			cmdSet.rightScaleInit(Cg);
			if (scaleData == GameData2.secondIndexR) {
				cmdSet.rightScaleTurn(Cg, speed);
			} else if (data == GameData.firstIndexR) {
				cmdSet.rightSwitchTurnTurn(Cg, speed);
			} else {
				System.out.println("Incorrect game data");
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
