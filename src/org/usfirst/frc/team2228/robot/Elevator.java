package org.usfirst.frc.team2228.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DMC60;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Relay;

public class Elevator {
	boolean on = false, off = true;
	DriverIF driverIF;
	WPI_TalonSRX elevator;
	DMC60 conveyor1, conveyor2;
	WPI_TalonSRX winch;
	Relay hook;
	Spark hookDown;
	PneumaticController pneu;
	DigitalInput limitSwitch, leftLimitSwitch, rightLimitSwitch, hookArmDownwards, hookArmUpwards;
	FeedbackDevice encoder;
	private boolean lastButtonDown;
	private boolean triggered;
	double currentHeight;
	int heightCount = 0;
	private boolean lastButtonUp;
	double previousHeight;
	boolean raising;
	boolean lowering;
	boolean lastButton1;
	boolean lastButton2;
	boolean triggered2;
	Timer timer;

	public enum ElevatorHeights {
		BOTTOM(0), PORTAL(100), SCALE_LOW(-1349826), SCALE_NEUTRAL(-1489334), SCALE_HIGH(-2637075);
		public final double height;

		ElevatorHeights(double encoderVal) {
			height = encoderVal;
		}
	}

	public Elevator(DriverIF _driverIF, PneumaticController _pneu) {
		pneu = _pneu;
		driverIF = _driverIF;
		elevator = new WPI_TalonSRX(RobotMap.CAN_ID_5);
		winch = new WPI_TalonSRX(RobotMap.CAN_ID_6);
		conveyor1 = new DMC60(RobotMap.PWM_PORT_2);
		conveyor2 = new DMC60(RobotMap.PWM_PORT_3);
		// limitSwitch = new DigitalInput(RobotMap.DIO_PORT_0);
		leftLimitSwitch = new DigitalInput(RobotMap.DIO_PORT_0);
		// rightLimitSwitch = new DigitalInput(RobotMap.DIO_PORT_1);
		hookArmDownwards = new DigitalInput(RobotMap.DIO_PORT_2);
		hookArmUpwards = new DigitalInput(RobotMap.DIO_PORT_3);
		elevator.set(0);
		hook = new Relay(0, Relay.Direction.kForward);
		hook.set(Relay.Value.kForward);
		hookDown = new Spark(RobotMap.PWM_PORT_4);
		hookDown.set(0);
		SmartDashboard.putNumber("back conveyor:", 0);
		SmartDashboard.putNumber("front conveyor:", 0);
		SmartDashboard.putNumber("Elevator Speed:", 0);
		SmartDashboard.putNumber("Launch:", 0);
		elevator.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		elevator.setNeutralMode(NeutralMode.Brake);
		System.out.println(ElevatorHeights.BOTTOM.height);
		elevator.getSensorCollection().setQuadraturePosition(0, 15);
		SmartDashboard.putBoolean("Limit Switch:", leftLimitSwitch.get());
		SmartDashboard.putNumber("Elevator Encoder Cts:", elevator.getSensorCollection().getQuadraturePosition());
		raising = false;
		lowering = true;
		triggered = false;
		triggered2 = false;
		timer = new Timer();
		elevator.configOpenloopRamp(2, 0);

	}

	public void teleopPeriodic() {
		double b = .5;
		// SmartDashboard.getNumber("Elevator Speed:", 0);
		// b is the speed of the

		if (driverIF.hookForward() && hookArmUpwards.get()) {
			hookDown.set(.4);
		} else if (driverIF.hookBackward() && hookArmDownwards.get()) {
			hookDown.set(-.4);
		} else {
			hookDown.set(0);
		}

		if (driverIF.RaiseElevator()) {
			pneu.brakeSet(off);
			pneu.squeezeSet(false);
			elevator.set(-b);
			// if(elevator.getSelectedSensorPosition(0) == -1){
			// elevator.set(0);
			//
			// }

		} else if (driverIF.LowerElevator()) {
			pneu.brakeSet(off);
			slowElevator(b);

			pneu.squeezeSet(false);
			if (!leftLimitSwitch.get()) {
				System.out.println("Limit Switch Triggered");
				elevator.set(0);
			}
			// if(!limitSwitch.get()){
			// elevator.set(0);
			// }
		} else {
			elevator.set(-0.05);
			pneu.brakeSet(on);
		}

		double e = .85;
		// SmartDashboard.getNumber("front conveyor:", 0);
		// hya

		if (!driverIF.conveyorsForward() && lastButton1 && triggered == false) {
			conveyor1.set(e);
			conveyor2.set(e);
			triggered = true;
			// System.out.println("Suck in");
		} else if (!driverIF.conveyorsForward() && lastButton1 && triggered == true) {
			conveyor1.set(0);
			conveyor2.set(0);
			triggered = false;
			// System.out.println("Co");
		}

		else if (!driverIF.conveyorsBackward() && lastButton2 && triggered2 == false) {
			conveyor1.set(-e);
			conveyor2.set(-e);
			triggered2 = true;
			// System.out.println("Suck in");
		} else if (!driverIF.conveyorsBackward() && lastButton2 && triggered2 == true) {
			conveyor1.set(0);
			conveyor2.set(0);
			triggered2 = false;
			// System.out.println("Co");
		}
		lastButton1 = driverIF.conveyorsForward();
		lastButton2 = driverIF.conveyorsBackward();
		double LaunchValue = SmartDashboard.getNumber("Launch:", 0);
		if (LaunchValue == 1) {
			hook.set(Relay.Value.kOff);
		}
		if (driverIF.winchWindUp()) {
			winch.set(.7);
		} else {
			winch.set(0);
		}
		SmartDashboard.putBoolean("Limit Switch:", leftLimitSwitch.get());
		SmartDashboard.putNumber("Elevator Encoder Cts:", elevator.getSensorCollection().getQuadraturePosition());
	}

	public void slowElevator(double speed) {
		if (elevator.getSensorCollection().getQuadraturePosition() > 10000) {
			elevator.set(speed);
		} else {
			elevator.set(0.2);
		}
	}

	public boolean elevatorSet(double height, double speed) {
		pneu.brakeSet(off);
		elevator.set(speed);
		if (raising = true && elevator.getSensorCollection().getQuadraturePosition() >= height) {
			elevator.set(0);
			pneu.brakeSet(on);
			return true;
		} else if (elevator.getSensorCollection().getQuadraturePosition() <= height) {
			elevator.set(0);
			pneu.brakeSet(on);
			return true;
		}
		return false;
	}
	public void elevatorSet(double speed){
		elevator.set(speed);
	}

	public boolean elevatorPortalSet() {
		timer.start();
		elevator.set(.3);
		if (timer.get() > 2.5) {
			elevator.set(0);
			timer.stop();
			return true;
		}
		return false;
	}

	public void conveyors(boolean on) {
		if (on) {
			conveyor1.set(1);
			conveyor2.set(1);
		} else {
			conveyor1.set(0);
			conveyor2.set(0);
		}
	}
}
