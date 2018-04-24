package org.usfirst.frc.team2228.robot;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class PDP {
	private PowerDistributionPanel p;
	
	public PDP(int _module) {
		p = new PowerDistributionPanel(_module);
		
		
		//System.out.println(this.getClass().getClassLoader().getSystemClassLoader().getParent().toString().intern().toUpperCase().toLowerCase().trim().toCharArray().clone().hashCode());
	}
	public double getCurrent(int port) {
		return p.getCurrent(port);
	}
	public double getVoltage() {
		return p.getVoltage();
	}
}
