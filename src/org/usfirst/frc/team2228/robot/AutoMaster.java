package org.usfirst.frc.team2228.robot;

//import org.usfirst.frc.team2228.commands.ElevatorRaise;
import org.usfirst.frc.team2228.commands.EncoderTurn;
import org.usfirst.frc.team2228.commands.MoveTo;
import org.usfirst.frc.team2228.commands.PneumaticGrabber;
import org.usfirst.frc.team2228.commands.RotateTo;
import org.usfirst.frc.team2228.commands.StringCommand;
import org.usfirst.frc.team2228.commands.Switch;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoMaster {
	final String baseLineAuto = "Baseline";
	final String leftSwitchAuto = "Left Switch";
	final String rightSwitchAuto = "Right Switch";
	private char[] positions;
	private double leftAutoMoveToSwitch = 14.81;
	private double autoMoveToScale = Dimensions.ALLIANCE_WALL_TO_SCALE_PLATE_EDGE - (Dimensions.WIDTH_OF_ROBOT + 29.69);
	private SRXDriveBase base;
	private String autoSelected;
	private String input = "";
	private CommandGroup Cg;
	private SendableChooser<String> chooser = new SendableChooser<>();
	private String robotSide = "Right";
	private double THISISWRONGSHOULDCALIBRATE = 5.0;
	private Elevator elevator;
	private PneumaticController pneu;

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

	}

	public void init() {
		base.setRightEncPositionToZero();
		base.setLeftEncPositionToZero();

		Cg = new CommandGroup();

		autoSelected = chooser.getSelected();
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		gameData = gameData.replace(" ", "");
		if (gameData.length() >= 3) {
			gameData = gameData.substring(0, 3);
		}
		positions = gameData.toCharArray();
		System.out.println("Auto selected: " + autoSelected);
		if (gameData.charAt(0) == 'L' || gameData.charAt(0) == 'l') {
			System.out.println("L");
		} else {
			System.out.println("R");
		}
		switch (autoSelected) {

		case "Baseline":
			System.out.println("Baseline selected");
			// Adds movement to the auto sequence
			Cg.addSequential(
					new MoveTo(base, (Dimensions.AUTOLINE_TO_ALLIANCE - Dimensions.LENGTH_OF_ROBOT), 0.3, false, 7));
			break;

		// case "Left Switch":
		//
		// if(gameData.length() == 0){
		// System.out.println("Could not find game data");
		// Cg.addSequential(
		// new MoveTo(base, (Dimensions.AUTOLINE_TO_ALLIANCE -
		// Dimensions.LENGTH_OF_ROBOT), 0.2, false));
		// }
		// else{
		// System.out.println("Left Switch selected");
		// // Adds movement to the auto sequence
		// Cg.addSequential(new MoveTo(base,
		// (Dimensions.ALLIANCE_WALL_TO_SWITCH_CENTER -
		// Dimensions.LENGTH_OF_ROBOT),
		// 0.4, false));
		// // If the left side of the switch is ours, it places the cube, if
		// // not, it does nothing
		// if (gameData.charAt(0) == 'L' || gameData.charAt(0) == 'l') {
		// Cg.addSequential(new Switch(elevator));
		// }
		// }
		// // Scale cube command
		// break;

		case "Left Switch":
			if (gameData.length() == 0) {
				System.out.println("Could not find game data");
				Cg.addSequential(
						new MoveTo(base, (Dimensions.AUTOLINE_TO_ALLIANCE - Dimensions.LENGTH_OF_ROBOT), 0.3, false, 10));
			} else {
				System.out.println("Left Switch Auto Selected");
				Cg.addSequential(new PneumaticGrabber(pneu, true, 0.5));
				Cg.addSequential(new MoveTo(base,
						(Dimensions.ALLIANCE_WALL_TO_SWITCH_CENTER - Dimensions.LENGTH_OF_ROBOT), 0.3, false, 10));
				Cg.addSequential(new RotateTo(base, 90, 0.3));
				Cg.addSequential(new MoveTo(base, leftAutoMoveToSwitch, 0.2, false, 10));

				if (gameData.charAt(0) == 'l' || gameData.charAt(0) == 'L') {
					Cg.addSequential(new Switch(elevator));
				}

			}
			break;

		case "Right Switch":

			if (gameData.length() == 0) {
				System.out.println("Could not find game data");
				Cg.addSequential(
						new MoveTo(base, (Dimensions.AUTOLINE_TO_ALLIANCE - Dimensions.LENGTH_OF_ROBOT), 0.2, false, 10));
			} else {
				System.out.println("Right Switch selected");
				// The bot starts closing the acquirer arms for half a second
				Cg.addSequential(new PneumaticGrabber(pneu, true, 0.5));
				// After half a second the bot starts moving
				Cg.addSequential(new MoveTo(base,
						(Dimensions.SWITCHWALL_TO_ALLIANCESTATION - Dimensions.LENGTH_OF_ROBOT), 0.3, false, 10));
				// While the bot is moving, it continues closing the acquirer
				// arms for another second and a half
				Cg.addParallel(new PneumaticGrabber(pneu, true, 0.5));
				//Changes 1.5 to .5 

				if (gameData.charAt(0) == 'R' || gameData.charAt(0) == 'r') {
					// If the right side of the switch is ours, it places the
					// cube while opening the acquirer arms
					Cg.addSequential(new PneumaticGrabber(pneu, false, 1.0));
					Cg.addSequential(new Switch(elevator));
				}
			}
			// Scale cube command
			break;

		case "Right Scale":
			if (gameData.length() == 0) {
				System.out.println("Could not find game data");
				Cg.addSequential(
						new MoveTo(base, (Dimensions.AUTOLINE_TO_ALLIANCE - Dimensions.LENGTH_OF_ROBOT), 0.2, false, 10));
			} else {
				Cg.addSequential(
						new MoveTo(base, (Dimensions.AUTOLINE_TO_ALLIANCE - Dimensions.LENGTH_OF_ROBOT), 0.2, false, 10));
				Cg.addSequential(new RotateTo(base, -90, .3));
				Cg.addSequential(new MoveTo(base, autoMoveToScale, 0.2, false, 10));

				if (gameData.charAt(0) == 'R' || gameData.charAt(0) == 'r') {
//					Cg.addSequential(new ElevatorRaise(elevator, Dimensions.SCALE_STARTING_POSITION, 0.3));
					Cg.addSequential(new Switch(elevator));
				}
			}

			break;

		case "Left Scale":
			if (gameData.length() == 0) {
				System.out.println("Could not find game data");
				Cg.addSequential(
						new MoveTo(base, (Dimensions.AUTOLINE_TO_ALLIANCE - Dimensions.LENGTH_OF_ROBOT), 0.2, false, 10));
			} else {
				Cg.addSequential(
						new MoveTo(base, (Dimensions.AUTOLINE_TO_ALLIANCE - Dimensions.LENGTH_OF_ROBOT), 0.2, false, 10));
				Cg.addSequential(new RotateTo(base, 90, .3));
				Cg.addSequential(new MoveTo(base, autoMoveToScale, 0.2, false, 10));

				if (gameData.charAt(0) == 'l' || gameData.charAt(0) == 'L') {
//					Cg.addSequential(new ElevatorRaise(elevator, Dimensions.SCALE_STARTING_POSITION, 0.3));
					Cg.addSequential(new Switch(elevator));
				}
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
