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
	boolean on = false, off = true, hitlimit = false;
	DriverIF driverIF;
	WPI_TalonSRX elevator;
	DMC60 conveyor1, conveyor2;
	WPI_TalonSRX winch;
	Relay hook;
	Spark hookDown;
	PneumaticController pneu;
	DigitalInput limitSwitch;
	Toggler t;
	Toggler Encoder;
	FeedbackDevice encoder;
	double currentHeight;
	int heightState;
	double previousHeight;
	boolean raising;
	boolean lowering;
	boolean lastButton1Conveyor;
	boolean lastButton2Conveyor;
	boolean lastButton1Elevator;
	boolean lastButton2Elevator;
	boolean triggered2;
	boolean triggered1Elevator;
	int heightCount;
	final int UNKNOWN = 99;

	public enum ElevatorHeights {
		BOTTOM(0), PORTAL(-141401), SCALE_LOW(-1349826), SCALE_NEUTRAL(-1489334), SCALE_HIGH(-2637075);
		public final double height;

		ElevatorHeights(double encoderVal) {
			height = encoderVal;
		}
	}
	public enum ElevatorTimes {
		BOTTOM(0), PORTAL(-141401), SCALE_LOW(-1349826), SCALE_NEUTRAL(-1489334), SCALE_HIGH(-2637075);
		public final double time;

		ElevatorTimes(double _time) {
			time = _time;
		}
	}

	public Elevator(DriverIF _driverIF, PneumaticController _pneu) {
		pneu = _pneu;
		driverIF = _driverIF;

		// elevator related stuff
		elevator = new WPI_TalonSRX(RobotMap.CAN_ID_5);
		elevator.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		elevator.setNeutralMode(NeutralMode.Brake);
		elevator.getSensorCollection().setQuadraturePosition(0, 15);
		elevator.set(0);
		limitSwitch = new DigitalInput(RobotMap.DIO_PORT_0);
		t = new Toggler(6);

		// winch + hook
		winch = new WPI_TalonSRX(RobotMap.CAN_ID_6);

		hook = new Relay(0, Relay.Direction.kForward);
		hook.set(Relay.Value.kForward);
		hookDown = new Spark(RobotMap.PWM_PORT_4);
		hookDown.set(0);

		// conveyors
		conveyor1 = new DMC60(RobotMap.PWM_PORT_2);
		conveyor2 = new DMC60(RobotMap.PWM_PORT_3);

		// dashboard stuff
		SmartDashboard.putNumber("back conveyor:", 0);
		SmartDashboard.putNumber("front conveyor:", 0);
		SmartDashboard.putNumber("Elevator Speed:", 0);
		SmartDashboard.putNumber("Launch:", 0);
		SmartDashboard.putNumber("Elevator", elevator.getSensorCollection().getQuadraturePosition());
		SmartDashboard.putBoolean("LimitSwitch", limitSwitch.get());
		elevator.setNeutralMode(NeutralMode.Brake);

		// for encoderset method
		Encoder = new Toggler(2);
	}

	public void teleopPeriodic() {

		ConveyorToggle(driverIF.conveyorToggle(), driverIF.conveyorsBackward(), 0.25, conveyor1, conveyor2);

		if (limitSwitch.get()) {
			elevator.getSensorCollection().setQuadraturePosition(0, 0);
			elevator.set(0);
			hitlimit = true;
		} else if (!hitlimit) {
			elevator.set(0.7);
			
		}
		int i = Encoder.state;
		int i2 = Encoder.toggle(driverIF.elevatorToggleUp(), driverIF.elevatorToggleDown());
		int state = t.toggle(driverIF.RaiseElevator(), driverIF.LowerElevator());
		double speed = 0.7;
		int error = 50000;

		if (i != i2) {
			EncoderReset();
		}
		System.out.println(Encoder + " " + t);

		MoveElevator(state, speed, error);
		SmartDashboard.putBoolean("LimitSwitch", limitSwitch.get());
		SmartDashboard.putNumber("Elevator", elevator.getSensorCollection().getQuadraturePosition());
		System.out.println("Encoder Cts." + elevator.getSensorCollection().getQuadraturePosition());
	}

	/**
	 * @param state
	 *            -the index of the state in the enum elevatorheights
	 * @param speed
	 *            -speed of the motor
	 * @param error
	 *            -the margin of error (encoder counts) acceptable (should be
	 *            quite large)
	 * @return -true if the action is completed, false if otherwise
	 */
	public boolean MoveElevator(int state, double speed, int error) {
		boolean done = false;
		switch (t.toggle(driverIF.RaiseElevator(), driverIF.LowerElevator())) {
		case 0:
			break;
		case 1:
			done = EncoderSet(ElevatorHeights.BOTTOM.height, speed, error);
			break;
		case 2:
			done = EncoderSet(ElevatorHeights.PORTAL.height, speed, error);
			break;
		case 3:
			done = EncoderSet(ElevatorHeights.SCALE_LOW.height, speed, error);
			break;
		case 4:
			done = EncoderSet(ElevatorHeights.SCALE_NEUTRAL.height, speed, error);
			break;
		case 5:
			done = EncoderSet(ElevatorHeights.SCALE_HIGH.height, speed, error);
			break;
		}
//		System.out.println("Encoder counts: " + elevator.getSensorCollection().getQuadraturePosition());
		return done;
	}
	public boolean moveElevatorOnTime(){
		return false;
	}
	
	public void conveyors(boolean on){
		if(on){
			conveyor1.set(1);
			conveyor2.set(-1);
		}
		else{
			conveyor1.set(0);
			conveyor2.set(0);
		}
	}

	/**
	 * @param encoderCount
	 *            - the encoderCount that the motor is supposed to go to
	 * @param speed
	 *            -speed of the motor
	 * @param Error
	 *            -the margin of error (encoder counts) acceptable (should be
	 *            quite large)
	 * @return -true if the action is completed, false if otherwise
	 */
	public boolean EncoderSet(double encoderCount, double speed, int Error) {
		if (Encoder.state == 0) {
			//pneu.brakeSet(false);
			if (elevator.getSensorCollection().getQuadraturePosition() > encoderCount + Error) {
				elevator.set(speed);
				return false;
			} else if (elevator.getSensorCollection().getQuadraturePosition() < encoderCount - Error) {
				elevator.set(-speed);
				return false;
			} else {
				elevator.set(0.05);
				Encoder.toggle(true);
				return true;
			}
			
		} else {
			//pneu.brakeSet(true);
			return true;
		}
	
	}

	public void EncoderReset() {
		Encoder.state = 0;
	}

	/**
	 * 
	 * @param button
	 *            - button that toggles moving the conveyors forward
	 * @param button2
	 *            - button that moves conveyors backwards
	 * @param speed
	 *            - speed
	 * @param motor
	 *            - first conveyor
	 * @param motor2
	 *            - second conveyor
	 */
	public void ConveyorToggle(boolean button, boolean button2, double speed, PWMSpeedController motor,
			PWMSpeedController motor2) {
		if (button) {
			motor.set(speed);
			motor2.set(-speed);
		}
		else if (button2) {
			motor.set(-speed);
			motor2.set(speed);
		}
		else {
			motor.set(0);
			motor2.set(0);
		}
	}
}
