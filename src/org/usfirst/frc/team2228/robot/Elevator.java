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
	WPI_TalonSRX winch;
	Relay hook;
	Spark hookDown;
	DigitalInput limitSwitch, leftLimitSwitch, rightLimitSwitch, hookArmDownwards, hookArmUpwards;
	Toggler t;
	Toggler Encoder;
	FeedbackDevice encoder;
	private boolean lastButtonDown;
	private boolean triggered;
	double currentHeight;
	int heightCount = 0;
	private boolean lastButtonUp;
	// DMC60 rearConveyor;
	double previousHeight;
	boolean raising;
	boolean lowering;
	boolean lastButton1;
	boolean lastButton2;
	boolean triggered2;
	boolean elevatorBackwards = false;
	TeleopController tc;
	Timer timer;
	CubeManipulator cube;

	public enum ElevatorHeights {
		BOTTOM(0), PORTAL(-703219), SCALE_LOW(-1349826), SCALE_NEUTRAL(-1489334), SCALE_HIGH(-2637075);
		public final double height;

		ElevatorHeights(double encoderVal) {
			height = encoderVal;
		}
	}

	public Elevator(DriverIF _driverIF, CubeManipulator _cube, TeleopController _tc) {
		driverIF = _driverIF;
		cube = _cube;
		tc = _tc;
		elevator = new WPI_TalonSRX(RobotMap.CAN_ID_5);
		winch = new WPI_TalonSRX(RobotMap.CAN_ID_6);
		// limitSwitch = new DigitalInput(RobotMap.DIO_PORT_0);
		leftLimitSwitch = new DigitalInput(RobotMap.DIO_PORT_0);
		rightLimitSwitch = new DigitalInput(RobotMap.DIO_PORT_1);
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
		SmartDashboard.putBoolean("Elevator backwards", elevatorBackwards);
		raising = false;
		lowering = true;
		triggered = false;
		triggered2 = false;
		timer = new Timer();
		t = new Toggler(5, false);

	}

	public void SlowRobot() {
		double power = 1.0;
		int encoders = elevator.getSensorCollection().getQuadraturePosition();
		if (elevator.getSensorCollection().getQuadraturePosition() > -1500000) {
			if (!cube.lift.get())
				tc.SetMaxThrottlePower(0.5);
			else
				tc.SetMaxThrottlePower(0.75);
		} else {
			tc.SetMaxThrottlePower(0.2);
			tc.SetMaxTurnPower(.3);
			System.out.println("Limiting Speed");
		}
	}

	public void teleopInit() {
		if (!leftLimitSwitch.get() || !rightLimitSwitch.get()) {
			elevator.getSensorCollection().setQuadraturePosition(0, 10);
		}
	}

	public void teleopPeriodic() {
		double b = .7666333420;
		if (!leftLimitSwitch.get() || !rightLimitSwitch.get()) {
			elevator.getSensorCollection().setQuadraturePosition(0, 10);
		}
		SlowRobot();
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
			// cube.squeezeSet(false);
			elevator.set(b);

		} else if (driverIF.LowerElevator()) {
			slowElevator(-0.8);
			if (!leftLimitSwitch.get() || !rightLimitSwitch.get()) {
				System.out.println("Limit Switch Triggered");
				elevator.set(0);
			}
		} else {
			elevator.set(0.11);
		}
		// int d = Encoder.state;
		// int e = Encoder.toggle(driverIF.elevatorToggleUp(),
		// driverIF.elevatorToggleDown());
		// if (d != e) {
		// MoveElevator(e, 0.5, 100000);
		// }
		// lastButton1 = driverIF.conveyorsForward();
		// lastButton2 = driverIF.conveyorsBackward();
		// double LaunchValue = SmartDashboard.getNumber("Launch:", 0);
		// if (LaunchValue == 1) {
		// hook.set(Relay.Value.kOff);
		// }
		// if (driverIF.winchWindUp()) {
		// winch.set(1);
		// driverIF.rumbleSet(true,.5);
		//
		// } else {
		// winch.set(0);
		// driverIF.rumbleSet(false, .5);
		// }
		SmartDashboard.putBoolean("Limit Switch:", rightLimitSwitch.get());
		SmartDashboard.putNumber("Elevator Encoder Cts:", elevator.getSensorCollection().getQuadraturePosition());
		System.out.println("Elevator Encoder cts: " + elevator.getSensorCollection().getQuadraturePosition());
		// System.out.println("Elevator Encoder Velocity: " +
		// elevator.getSensorCollection().getQuadratureVelocity());
	}

	public void slowElevator(double speed) {
		if (elevator.getSensorCollection().getQuadraturePosition() < -1000000) {
			elevator.set(speed);
			System.out.println("up");
		} else {
			elevator.set(-0.2);
			System.out.println("Slowing elevator");
		}
		if (!leftLimitSwitch.get() || !rightLimitSwitch.get()) {
			elevator.set(0);
			elevator.getSensorCollection().setQuadraturePosition(0, 10);
		}
		if (elevator.getSensorCollection().getQuadraturePosition() > 0) {
			elevatorBackwards = true;
		} else {
			elevatorBackwards = false;
		}
		SmartDashboard.putBoolean("Elevator backwards", elevatorBackwards);
	}

	public void elevatorSet(double speed) {
		elevator.set(speed);
	}

	public void lowerElevator(double speed) {
		elevator.set(speed);
		if (!leftLimitSwitch.get() || !rightLimitSwitch.get()) {
			System.out.println("Limit Switch Triggered");
			elevator.set(0);
		}
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

	/**
	 * @param encoderCount
	 *            - the encoderCount that the motor is supposed to go to
	 * @param speed
	 *            -speed of the motor
	 * @param Error
	 *            -the margin of error (encoder counts) acceptable (should be quite
	 *            large)
	 * @return -true if the action is completed, false if otherwise
	 */
	public boolean EncoderSet(double encoderCount, double speed, int Error) {
		if (Encoder.state == 0) {
			// pneu.brakeSet(false);
			if (elevator.getSensorCollection().getQuadraturePosition() > encoderCount + Error) {
				elevator.set(speed);
				return false;
			} else if (elevator.getSensorCollection().getQuadraturePosition() < encoderCount - Error) {
				elevator.set(-speed);
				return false;
			} else {
				elevator.set(0.15);
				Encoder.toggle(true);
				return true;
			}

		} else {
			// pneu.brakeSet(true);
			return true;
		}

	}

	public void EncoderReset() {
		Encoder.state = 0;
	}

	/**
	 * @param state
	 *            -the index of the state in the enum elevatorheights
	 * @param speed
	 *            -speed of the motor
	 * @param error
	 *            -the margin of error (encoder counts) acceptable (should be quite
	 *            large)
	 * @return -true if the action is completed, false if otherwise
	 */
	public boolean MoveElevator(int state, double speed, int error) {
		boolean done = false;
		switch (state) {
		case 0:
			done = EncoderSet(ElevatorHeights.BOTTOM.height, speed, error);
			break;
		case 1:
			done = EncoderSet(ElevatorHeights.PORTAL.height, speed, error);
			break;
		case 2:
			done = EncoderSet(ElevatorHeights.SCALE_LOW.height, speed, error);
			break;
		case 3:
			done = EncoderSet(ElevatorHeights.SCALE_NEUTRAL.height, speed, error);
			break;
		case 4:
			done = EncoderSet(ElevatorHeights.SCALE_HIGH.height, speed, error);
			break;
		}
		// System.out.println("Encoder counts: " +
		// elevator.getSensorCollection().getQuadraturePosition());
		return done;
	}

	public void manual(double b) {
		if (driverIF.RaiseElevator()) {
			cube.brakeSet(off);
			// cube.squeezeSet(false);
			elevator.set(b);

		} else if (driverIF.LowerElevator()) {
			cube.brakeSet(off);
			slowElevator(-0.8);
			if (!leftLimitSwitch.get() || !rightLimitSwitch.get()) {
				System.out.println("Limit Switch Triggered");
				elevator.set(0);
			}
		} else {
			elevator.set(0.11);
			cube.brakeSet(on);
		}

	}

}
