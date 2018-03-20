package org.usfirst.frc.team2228.robot;

import org.usfirst.frc.team2228.commands.StringCommand;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
	private TeleopController chessyDrive;
	private AutoMaster auto;
	private Elevator elevator;
	private PneumaticController pc;
	private AnalogUltrasonic au;
	private CameraController cam;
	private CANLED LED;
	private AngleIF angleIF;
//	private UsbCamera backCamera;
//	private UsbCamera cam;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */

	@Override
	public void robotInit() {

		driverIF = new DriverIF();
		base = new SRXDriveBase();
		cube = new CubeManipulator(driverIF);
		chessyDrive = new TeleopController(driverIF, base);
		cam = new CameraController();
		pc = new PneumaticController(driverIF);
		elevator = new Elevator(driverIF, pc, chessyDrive);
		auto = new AutoMaster(base, elevator, pc);
		au = new AnalogUltrasonic();
		angleIF = new AngleIF();
		base.setAngleIF(angleIF);
		
//		base.setCorrectionSensor(3); // navx
		
//		frontCamera = new UsbCamera("Front Camera", 0);
//		backCamera = new UsbCamera("Back Camera", 1);
//		frontCamera = CameraServer.getInstance().startAutomaticCapture();
//		backCamera = CameraServer.getInstance().startAutomaticCapture();
//		 LED = new CANLED();
//		 LED.colorInit();
		// angle = new AngleIF();
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
		// Gets everything from Autonomous Init from the AutoMaster class
		angleIF.zeroYaw();
		auto.init();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		// Gets everything from the Autonomous Periodic from the AutoMaster
		// class
		auto.run();
	}
	
	public void teleopInit() {
		angleIF.zeroYaw();
	}
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		// Gets everything from the generic drivebase, the CubeManipulator
		// class, the Pneumatic class and the Elevatorclass for the teleop
		// period
		chessyDrive.teleopPeriodic();
		cube.teleopPeriodic();
		pc.teleopPeriodic();
		elevator.teleopPeriodic();
		cam.cameraCommand();

		// LED.allianceColorLED();
		// LED.autonomousColorInit();
		// LED.rainbowShift();
		//au.roundTo(0.0001);
		// System.out.println(au.getDistance1());
		SmartDashboard.putNumber("Sensor1", au.getDistance1());
		SmartDashboard.putNumber("Sensor2", au.getDistance2());

	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
	
}
