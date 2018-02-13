package org.usfirst.frc.team2228.robot;

import org.usfirst.frc.team2228.commands.EncoderTurn;
import org.usfirst.frc.team2228.commands.MoveTo;
import org.usfirst.frc.team2228.commands.RotateTo;
import org.usfirst.frc.team2228.commands.StringCommand;

import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoMaster {
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	private char[] positions;
	private SRXDriveBase base;
	private String autoSelected;
	private String input = "";
	private CommandGroup Cg;
	private SendableChooser<String> chooser = new SendableChooser<>();
	
	public AutoMaster(SRXDriveBase srxdb) {
		base = srxdb;
		
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto choices", chooser);
		//BOOP BEEP BOP BEEPEDIE BOOP BOP 
		
		
		autoSelected = chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
		StringCommand command = new StringCommand(input);
		//command.start();
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		
		positions = gameData.toCharArray();
	}
	public void init() {
		base.setRightPositionToZero();
		base.setLeftPositionToZero();
		
		Cg = new CommandGroup();
		
		Cg.addSequential(new MoveTo(base, 12, 0.2, true));
		Cg.addSequential(new RotateTo(base, 180, 0.2));
		
		Cg.start();
		
		
	}
	public void run() {
		Scheduler.getInstance().run();
	}
}
