package org.usfirst.frc.team2228.robot;

import org.usfirst.frc.team2228.commands.MoveTo;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoMaster {
	private char[] positions;
	private SRXDriveBase base;
	
	public AutoMaster(char[] startPositions, SRXDriveBase srxdb) {
		positions = startPositions;
		base = srxdb;	
	}
	public void update(String Condition) {
		CommandGroup Cg = new CommandGroup();
		
		Cg.addSequential(new MoveTo(base, 0, 0, false), 1);
	}
}
