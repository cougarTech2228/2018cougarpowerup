package org.usfirst.frc.team2228.robot;


import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	private SRXDriveBase base;
	private CubeManipulator cube;
	private DriverIF driverIF;
	private AngleIF angleIF;
	private TeleopController chessyDrive;
	private AutoMaster auto;
	private Elevator elevator;
	private PneumaticController pc;
	private AnalogUltrasonic au;


//	UsbCamera camera;
	//private CANLED LED;
	//private AngleIF angle;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */

	@Override
	public void robotInit() {

		angleIF = new AngleIF();
		driverIF = new DriverIF(angleIF);
		base = new SRXDriveBase();

		chessyDrive = new TeleopController(driverIF, base);
		pc = new PneumaticController(driverIF);
		elevator = new Elevator(driverIF, pc);
		auto = new AutoMaster(base, elevator);
		au = new AnalogUltrasonic();

		auto = new AutoMaster(base, elevator);

//		camera = CameraServer.getInstance().startAutomaticCapture();

//		LED = new CANLED();
//		LED.colorInit();
		//angle = new AngleIF();
	}

	
	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		auto.init();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		auto.run();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopInit() {
		chessyDrive.teleopInit();
	}
	
	@Override
	public void teleopPeriodic() {
		
		// SmartDashboard.putNumber("Sonar", us.getDistance());
		chessyDrive.teleopPeriodic();

		cube.teleopPeriodic();
		pc.teleopPeriodic();
		elevator.teleopPeriodic();

		//LED.allianceColorLED();
		//LED.autonomousColorInit();
		//LED.rainbowShift();

		au.roundTo(3);
		au.updateSensors();
		
		System.out.println(au.getDistance1() + " " + au.getDistance2());
		SmartDashboard.putNumber("Angle", angleIF.getAngle());
		SmartDashboard.putNumber("Yaw", angleIF.round(angleIF.getYaw(), 3));
		SmartDashboard.putNumber("Roll", angleIF.getRoll());
		SmartDashboard.putNumber("Correction", angleIF.getAngleCorrection());
		//SmartDashboard.putNumber("Sensor1", au.getDistance1());
		//SmartDashboard.putNumber("Sensor2", au.getDistance2());
		//angle.getAngle();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}

}
