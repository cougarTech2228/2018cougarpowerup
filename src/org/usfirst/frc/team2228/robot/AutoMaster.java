package org.usfirst.frc.team2228.robot;

import org.usfirst.frc.team2228.commands.EncoderTurn;
import org.usfirst.frc.team2228.commands.MoveTo;
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
	
	public AutoMaster(SRXDriveBase srxdb, Elevator _elevator) {
		base = srxdb;
		chooser.addDefault("Left Switch", leftSwitchAuto);
		elevator = _elevator;
		chooser.addObject("Baseline", baseLineAuto);
		chooser.addObject("Right Switch", rightSwitchAuto);
		SmartDashboard.putData("Auto choices", chooser);
		//BOOP BEEP BOP BEEPEDIE BOOP BOP 
		
		
		
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// baseLineAuto);
		StringCommand command = new StringCommand(input);
		//command.start();

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
		if(gameData.charAt(0) == 'L'){
			System.out.println("L");
		}else{
			System.out.println("R");
	}
		switch (autoSelected) {
			case "Baseline":
				System.out.println("Baseline selected");
				Cg.addSequential(new MoveTo(base, (THISISWRONGSHOULDCALIBRATE +
						                            Dimensions.AUTOLINE_TO_ALLIANCE - Dimensions.LENGTH_OF_ROBOT), 0.2, false));
				break;
				
			case "Left Switch":
				System.out.println("Left Switch selected");
				Cg.addSequential(new MoveTo(base, (THISISWRONGSHOULDCALIBRATE +
						                           Dimensions.SWITCHWALL_TO_ALLIANCESTATION - Dimensions.LENGTH_OF_ROBOT), 0.4, false));
				if(gameData.charAt(0) == 'L'){
				Cg.addSequential(new Switch(elevator));
				}
				
				// Scale cube command
				break;
				
			case "Right Switch":
				System.out.println("Right Switch selected");
				Cg.addSequential(new MoveTo(base, (THISISWRONGSHOULDCALIBRATE +
						                           Dimensions.SWITCHWALL_TO_ALLIANCESTATION - Dimensions.LENGTH_OF_ROBOT), 0.4, false));
				if(gameData.charAt(0) == 'R'){
				Cg.addSequential(new Switch(elevator));
				}
				
				// Scale cube command
				break;
		}
		Cg.start();
	}
	public void run() {
		Scheduler.getInstance().run();
	}
}
