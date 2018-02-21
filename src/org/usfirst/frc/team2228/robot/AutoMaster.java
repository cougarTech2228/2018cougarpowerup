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
			//Adds movement to the auto sequence
			Cg.addSequential(
					new MoveTo(base, (Dimensions.AUTOLINE_TO_ALLIANCE - Dimensions.LENGTH_OF_ROBOT), 0.2, false));
			break;

		case "Left Switch":
			System.out.println("Left Switch selected");
			//Adds movement to the auto sequence
			Cg.addSequential(new MoveTo(base, (Dimensions.SWITCHWALL_TO_ALLIANCESTATION - Dimensions.LENGTH_OF_ROBOT),
					0.4, false));
			//If the left side of the switch is ours, it places the cube, if not, it does nothing
			if (gameData.charAt(0) == 'L') {
				Cg.addSequential(new Switch(elevator));
			}

			// Scale cube command
			break;

		case "Right Switch":
			System.out.println("Right Switch selected");
			//The bot starts closing the aquirer arms for half a second
			Cg.addSequential(new PneumaticGrabber(pneu, true, 0.5));
			//After half a second the bot starts moving
			Cg.addSequential(new MoveTo(base, (Dimensions.SWITCHWALL_TO_ALLIANCESTATION - Dimensions.LENGTH_OF_ROBOT),
					0.4, false));
			//While the bot is moving, it continues closing the aquirer arms for another second and a half
			Cg.addParallel(new PneumaticGrabber(pneu, true, 1.5));

			if (gameData.charAt(0) == 'R') {
				//If the right side of the switch is ours, it places the cube while opening the aquirer arms
				Cg.addSequential(new Switch(elevator));
				Cg.addParallel(new PneumaticGrabber(pneu, false, 2.0));
			}

			// Scale cube command
			break;
		}
		Cg.start();
	}

	public void run() {
		//Runs the sequence made in auto init
		Scheduler.getInstance().run();
	}
}
