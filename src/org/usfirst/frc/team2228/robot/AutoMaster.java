package org.usfirst.frc.team2228.robot;

import org.usfirst.frc.team2228.commands.AutoRollers;
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
	final String scale = "Scale";
	private SRXDriveBase base;
	private String autoSelected;
	private String input = "";
	private CommandGroup Cg = null;
	private CommandGroup rightSwitch = null;
	private SendableChooser<String> chooser = new SendableChooser<>();
	private String robotSide = "Right";
	private double THISISWRONGSHOULDCALIBRATE = 5.0;
	private Elevator elevator;
	private CubeManipulator cube;
	private double speed = .4;
	private String firstIndex;
	private String secondIndex;

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
		chooser.addObject("Scale", scale);
		SmartDashboard.putData("Auto choices", chooser);
		SmartDashboard.putNumber("Wait Time", 0);
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
			// Cg.addSequential(new WaitCommand(SmartDashboard.getNumber("Wait Time", 0)));
			// System.out.println("Baseline selected");
			// // Adds movement to the auto sequence
			Cg.addSequential(new MoveTo(base, (Dimensions.AUTOLINE_TO_ALLIANCE - Dimensions.LENGTH_OF_ROBOT), speed,
					false, 3.0));
			Cg.addSequential(new PneumaticGrabber(cube, true, 0.5));
			System.out.println("Left Encoders: " + base.getLeftEncoderPosition() + " Right Encoders: "
					+ base.getRightEncoderPosition());
			break;

		case "Left Switch":
			Cg.addSequential(new WaitCommand(SmartDashboard.getNumber("Wait Time", 0)));
			System.out.println("Left Switch selected");
			// Adds movement to the auto sequence
			Cg.addSequential(new PneumaticGrabber(cube, true, 0.5));
			// After half a second the bot starts moving
			Cg.addSequential(new MoveTo(base, (Dimensions.ALLIANCE_WALL_TO_SWITCH - Dimensions.LENGTH_OF_ROBOT + 12.0),
					speed, false, 4.0));
			Cg.addSequential(new WaitCommand(SmartDashboard.getNumber("Wait Time", 0)));
			// While the bot is moving, it continues closing the aquirer arms for another
			// second and a half
			Cg.addParallel(new PneumaticGrabber(cube, true, 1.5));
			// If the left side of the switch is ours, it places the cube, if not, it does
			// nothing
			if (data == GameData.firstIndexL) {
				Cg.addSequential(new RotateTo(base, 45, speed));
				Cg.addParallel(new ElevatorAuto(elevator, 2.0));
				Cg.addParallel(new MoveTo(base, 19, speed / 2, false, 1.0), 3);
				Cg.addParallel(new PneumaticGrabber(cube, false, 2.0));
				Cg.addSequential(new CubeRotate(cube, false));
				Cg.addSequential(new AutoRollers(cube, speed, 3.0));
			} else {
				// Cg.addSequential(new MoveTo(base, -6.0, speed, false));
				System.out.println("Incorrect game data");
			}

			// Scale cube command
			break;

		case "Right Switch":
			Cg.addSequential(new WaitCommand(SmartDashboard.getNumber("Wait Time", 0)));
			System.out.println("Right Switch selected");
			// The bot starts closing the acquirer arms for half a second
			Cg.addSequential(new PneumaticGrabber(cube, true, 0.5));
			// After half a second the bot starts moving
			Cg.addSequential(new MoveTo(base, (Dimensions.AUTOLINE_TO_ALLIANCE - Dimensions.LENGTH_OF_ROBOT) + 3, speed,
					false, 4.0));
			// While the bot is moving, it continues closing the aquirer arms for another
			// second and a half
			Cg.addParallel(new PneumaticGrabber(cube, true, 1.5));

			if (data == GameData.firstIndexR) {
				// If the right side of the switch is ours, it places the cube while opening the
				// aquirer arms
				// Cg.addParallel(new MoveTo(base, 3, speed / 2, false, 2.0));
				Cg.addParallel(new PneumaticGrabber(cube, false, 2.0));
				Cg.addParallel(new ElevatorAuto(elevator, 4.0));
				Cg.addParallel(new CubeRotate(cube, false));
				Cg.addParallel(new AutoRollers(cube, speed, 3.0));

			} else {
				System.out.println("Incorrect game data");
			}

			// Scale cube command
			break;
		case "Right Switch Turn":
			rightSwitch.addSequential(new WaitCommand(SmartDashboard.getNumber("Wait Time", 0)));
			System.out.println("Right Switch turn selected");
			// Adds movement to the auto sequence
			rightSwitch.addSequential(new PneumaticGrabber(cube, true, 0.5));
			// After half a second the bot starts moving
			// While the bot is moving, it continues closing the aquirer arms for another
			// second and a half
			// If the left side of the switch is ours, it places the cube, if not, it does
			// nothing
			rightSwitchTurnSet();
			Cg.addSequential(rightSwitch);
			// Scale cube command
			break;
		case "Scale":
			Cg.addSequential(new WaitCommand(SmartDashboard.getNumber("Wait Time", 0)));
			System.out.println("Scale selected");
			// Adds movement to the auto sequence
			Cg.addSequential(new PneumaticGrabber(cube, true, 0.5));
			if (scaleData == GameData2.secondIndexR) {
				Cg.addSequential(new MoveTo(base, (Dimensions.ALLIANCE_WALL_TO_SCALE - Dimensions.LENGTH_OF_ROBOT), speed,
						false, 8.0));
				Cg.addSequential(new WaitCommand(SmartDashboard.getNumber("Wait Time", 0)));
				Cg.addSequential(new RotateTo(base, -45, speed));
				Cg.addParallel(new ElevatorAuto(elevator, 2.0));
				Cg.addParallel(new MoveTo(base, 19, speed / 2, false, 1.0), 3);
				Cg.addParallel(new PneumaticGrabber(cube, false, 2.0));
				Cg.addSequential(new AutoRollers(cube, speed, 3.0));
			} else if (data == GameData.firstIndexR) {
				rightSwitchTurnSet();
				Cg.addSequential(rightSwitch);
			} else {
				System.out.println("Incorrect game data");
			}
			// Cg.addSequential(new MoveTo(base, -6.0, speed, false));
			
		}
		Cg.start();
	}
	
	public void run() {
		// Runs the sequence made in auto init
		Scheduler.getInstance().run();
	}
	public void rightSwitchTurnSet() {
		rightSwitch.addSequential(new MoveTo(base, (Dimensions.ALLIANCE_WALL_TO_SWITCH - Dimensions.LENGTH_OF_ROBOT + 12.0),
				speed, false, 4.0));
		rightSwitch.addSequential(new WaitCommand(SmartDashboard.getNumber("Wait Time", 0)));
		if (data == GameData.firstIndexR) {
			rightSwitch.addSequential(new RotateTo(base, -45, speed));
			rightSwitch.addParallel(new ElevatorAuto(elevator, 2.0));
			rightSwitch.addParallel(new MoveTo(base, 19, speed / 2, false, 1.0), 3);
			rightSwitch.addParallel(new PneumaticGrabber(cube, false, 2.0));
			rightSwitch.addSequential(new AutoRollers(cube, speed, 3.0));
		} else {
			// Cg.addSequential(new MoveTo(base, -6.0, speed, false));
			System.out.println("Incorrect game data");
		}
	}
}
