package org.usfirst.frc.team2228.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DMC60;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.Spark;
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
	DigitalInput limitSwitch;
	Toggler t;
	FeedbackDevice encoder;
	private boolean lastButtonDown;
	private boolean triggered;
	double currentHeight;
	int heightState;
	private boolean lastButtonUp;
	double previousHeight;
	boolean raising;
	boolean lowering;
	boolean lastButton1Conveyor;
	boolean lastButton2Conveyor;
	boolean lastButton1Elevator;
	boolean lastButton2Elevator;
	boolean triggered2;
	boolean triggered1Elevator;
	private boolean lastButton;

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
		limitSwitch = new DigitalInput(RobotMap.DIO_PORT_0);
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
		raising = false;
		lowering = true;
		triggered = false;
		triggered2 = false;
		lastButton = false;
		triggered1Elevator = false;
		t = new Toggler(5);
		currentHeight = ElevatorHeights.BOTTOM.height;
	}

	public void teleopPeriodic() {
		elevator.getSensorCollection().getQuadraturePosition();
		double b = 1;
		if (driverIF.hookForward()) {
			hookDown.set(.4);
		} else if (driverIF.hookBackward()) {
			hookDown.set(-.4);
		} else {
			hookDown.set(0);
		}

		// if (driverIF.RaiseElevator()) {
		// pneu.brakeSet(off);
		// elevator.set(b);
		// // if(elevator.getSelectedSensorPosition(0) == -1){
		// // elevator.set(0);
		// //
		// // }
		//
		// } else if (driverIF.LowerElevator()) {
		// pneu.brakeSet(off);
		// elevator.set(-b);
		// pneu.liftSet(false);
		// if (!limitSwitch.get()) {
		// System.out.println("Limit Switch Triggered");
		// elevator.set(0);
		// }
		// } else {
		// elevator.set(0);
		// pneu.brakeSet(on);
		// }

		double d = 1;
		elevatorToggle(driverIF.elevatorToggleUp(), driverIF.elevatorToggleDown(), 1.0, elevator);
		motorToggle(driverIF.conveyorsForward(), lastButton1Conveyor, triggered, true, 1, conveyor1);
		motorToggle(driverIF.conveyorsForward(), lastButton1Conveyor, triggered, true, 1, conveyor2);
		motorToggle(driverIF.conveyorsBackward(), lastButton2Conveyor, triggered2, true, -1, conveyor1);
		double LaunchValue = SmartDashboard.getNumber("Launch:", 0);
		if (LaunchValue == 1) {
			hook.set(Relay.Value.kOff);
		}
		if (driverIF.winchWindUp()) {
			winch.set(.7);
		} else if (driverIF.winchWindDown()) {
			winch.set(-.7);
		}

	}

	public boolean elevatorSet(double height, double speed) {
		pneu.brakeSet(off);
		if (raising) {
			elevator.set(1);
			if (elevator.getSensorCollection().getQuadraturePosition() >= height) {
				elevator.set(0);
				pneu.brakeSet(on);
				return true;
			} else {

			}
		} else {
			elevator.set(-1);
			if (elevator.getSensorCollection().getQuadraturePosition() <= height) {
				elevator.set(0);
				pneu.brakeSet(on);
			}
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

	/**
	 * 
	 * @param button
	 *            - button you would like to toggle
	 * @param lastButton
	 *            - set to state of button after loop
	 * @param triggered
	 *            - if motor is on or off
	 * @param on_off
	 *            - true if user wants the button to toggle the motor on/off,
	 *            false if they want it to toggle to speed/-speed
	 * @param speed
	 *            - speed at which user wants motors to run
	 * @param motor
	 *            - motor the user wants to toggle
	 */
	public void motorToggle(boolean button, boolean lastButton, boolean isTriggered, boolean on_off, double speed,
			PWMSpeedController motor) {
		if (on_off) {
			// driverIF.toggle1(button, lastButton, isTriggered);
			if (!button && lastButton && !isTriggered) {
				isTriggered = true;
				motor.set(speed);
			}
			if (!button && lastButton && isTriggered == true) {
				motor.set(0);
			}
			lastButton = button;
			// driverIF.toggle2(button, lastButton, isTriggered);

		} else {
			// driverIF.toggle1(button, lastButton, isTriggered);
			if (!button && lastButton && !isTriggered) {
				isTriggered = true;
				motor.set(speed);
			}
			if (!button && lastButton && isTriggered == true) {
				motor.set(-speed);
			}
			lastButton = button;

		}

	}

	/**
	 * 
	 * @param buttonUp
	 *            - button user wants to toggle the elevator up
	 * @param buttonDown
	 *            - button user wants to toggle elevator down
	 * @param lastButton
	 *            - set to state of button at the end of loop cycle
	 * @param triggered
	 *            - if motor is on or off
	 * @param on_off
	 *            - true if user wants the button to toggle the motor on/off,
	 *            false if they want it to toggle to speed/-speed
	 * @param speed
	 *            - speed at which user wants motors to run
	 * @param elevator
	 *            - motor the user wants to toggle
	 */

	public void elevatorToggle(boolean buttonUp, boolean buttonDown, double speed, WPI_TalonSRX elevator) {
		/*
		 * int heightCount = 0; if (heightCount < 4) {
		 * driverIF.toggle1(buttonUp, lastButton, isTriggered); heightCount++;
		 * System.out.println("Going up"); driverIF.toggle2(buttonUp,
		 * lastButton, isTriggered); heightCount++;
		 * System.out.println("Going up"); } else if (heightCount > 0) {
		 * driverIF.toggle1(buttonDown, lastButton, isTriggered); heightCount--;
		 * System.out.println("Going down"); driverIF.toggle2(buttonDown,
		 * lastButton, isTriggered); heightCount--;
		 * System.out.println("Going down"); }
		 */

		int heightCount = t.toggle(buttonUp, true);
		heightCount = t.toggle(buttonDown, false);
		if (heightState < heightCount) {
			raising = true;
		} else {
			raising = false;
		}

		if (heightCount == 0) {
			elevatorSet(ElevatorHeights.BOTTOM.height, .7);
			currentHeight = ElevatorHeights.BOTTOM.height;
			System.out.println("Bottom");
		} else if (heightCount == 1) {
			elevatorSet(ElevatorHeights.PORTAL.height, .7);
			currentHeight = ElevatorHeights.PORTAL.height;
			System.out.println("Portal");
		} else if (heightCount == 2) {
			elevatorSet(ElevatorHeights.SCALE_LOW.height, .7);
			currentHeight = ElevatorHeights.SCALE_LOW.height;
			System.out.println("Scale low");
		} else if (heightCount == 3) {
			elevatorSet(ElevatorHeights.SCALE_NEUTRAL.height, .7);
			currentHeight = ElevatorHeights.SCALE_NEUTRAL.height;
			System.out.println("Scale neutral");
		} else if (heightCount == 4) {
			elevatorSet(ElevatorHeights.SCALE_HIGH.height, .7);
			currentHeight = ElevatorHeights.SCALE_HIGH.height;
			System.out.println("Scale high");
		}
		heightState = heightCount;

	}
}
