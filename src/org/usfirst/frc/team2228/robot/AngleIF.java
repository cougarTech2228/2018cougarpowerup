package org.usfirst.frc.team2228.robot;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.*;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AngleIF implements PIDOutput {
	private AHRS ahrs;
//	private PIDController _PIDController;
	static final double AngleSP = 0.0;
	static double rate = 0;
	static double kP = 0.03;
	static final double kI = 0.00;
	static final double kD = 0.00;
	
	static final double kF = 0.00;
	private PIDSource pidSource;
	private PIDOutput pidOutput;
	// how close the navX will get to the value
	static final double kToleranceDegrees = 2.0f;
	private boolean rotateToAngle = false;

	public AngleIF() {
		try {
			ahrs = new AHRS(Port.kOnboard);
		} catch (RuntimeException ex) {
			System.out.println("Error starting the navx");
		}
		ahrs.reset();
//		PIDController _PIDController = new PIDController(kP, kI, kD, kF, pidSource, pidOutput);
//        _PIDController.setInputRange(-180.0f,  180.0f);
//        _PIDController.setOutputRange(-1.0, 1.0);
//        _PIDController.setAbsoluteTolerance(kToleranceDegrees);
//        _PIDController.setContinuous(true);
        SmartDashboard.putNumber("Barometric Pressure", getBaroPressure());
        SmartDashboard.putNumber("kP", getBaroPressure());
        SmartDashboard.putNumber("Barometric Pressure", getBaroPressure());
	}

	public void setZeroAngle(double gyro) {
		ahrs.setAngleAdjustment(gyro);
	}

	public double getAngle() {
		SmartDashboard.putNumber("Navx angle", ahrs.getAngle());
		return ahrs.getAngle();

	}

	public double getYaw() {
		return ahrs.getYaw();
	}

	public double getRoll() {
		return ahrs.getRoll();
	}
	public void zeroYaw() {
		ahrs.zeroYaw();
	}

	public double _PIDCorrection(double angle) {
		double error;
		error = getYaw() - angle;
		return kP * error;

	}
	public double getAngleCorrection() {
		double error;
		error = AngleSP - getYaw();
		return (kP * error) - (kD * rate);
	}

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub

	}
}
