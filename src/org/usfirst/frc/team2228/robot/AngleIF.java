package org.usfirst.frc.team2228.robot;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.*;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AngleIF {
	private AHRS ahrs;

	public AngleIF() {
		try {
			ahrs = new AHRS(Port.kOnboard);
		} catch (RuntimeException ex) {
			System.out.println("Error starting the navx");
		}
		ahrs.zeroYaw();
	}

	public void setZeroAngle(double gyro) {
		ahrs.setAngleAdjustment(gyro);
	}

	public double getAngle() {
		SmartDashboard.putNumber("Navx angle", ahrs.getAngle());
		return ahrs.getAngle();

	}

	public double getHyaw() {
		return ahrs.getYaw();
	}

	public double getRoll() {
		return ahrs.getRoll();
	}

	public double getBaroPressure() {
		return ahrs.getBarometricPressure();
	}
}
