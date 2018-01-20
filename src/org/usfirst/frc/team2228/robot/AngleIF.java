package org.usfirst.frc.team2228.robot;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.*;

public class AngleIF {
	private AHRS ahrs;
	public AngleIF() {
		
	}
	public void setZeroAngle(double gyro) {
		ahrs.setAngleAdjustment(gyro);
	}
	public double getAngle() {
		return ahrs.getAngle();
	}
	public double getHyaw() {
		return ahrs.getYaw();
	}
	public double getRoll() {
		return ahrs.getRoll();
	}
}
