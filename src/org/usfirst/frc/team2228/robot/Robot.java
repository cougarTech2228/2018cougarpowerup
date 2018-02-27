package org.usfirst.frc.team2228.robot;

import org.usfirst.frc.team2228.commands.StringCommand;

import edu.wpi.cscore.MjpegServer;
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
	UsbCamera camera;
	private UsbCamera camera0;
	// private CANLED LED;
	// private AngleIF angle;
	private MjpegServer server;
	private UsbCamera camera1;
	private UsbCamera curCam;
	private Toggler toggler;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */

	@Override
	public void robotInit() {

		driverIF = new DriverIF();
		base = new SRXDriveBase();
		cube = new CubeManipulator(driverIF);
		chessyDrive = new TeleopController(driverIF, base);

		pc = new PneumaticController(driverIF);
		elevator = new Elevator(driverIF, pc);
		auto = new AutoMaster(base, elevator);
		au = new AnalogUltrasonic();
		camera = CameraServer.getInstance().startAutomaticCapture();
		// LED = new CANLED();
		// LED.colorInit();
		// angle = new AngleIF();
		toggler = new Toggler(2);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString line to get the
	 * auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the SendableChooser
	 * make sure to add them to the chooser code above as well.
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

	// public void teleopInit() {
	// System.out.println("teleopInit() fi!");
	// chessyDrive.teleopInit();
	// System.out.println("Teleop Init done");
	// }
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {

		// SmartDashboard.putNumber("Sonar", us.getDistance());
		chessyDrive.teleopPeriodic();
		cube.teleopPeriodic();
		pc.teleopPeriodic();
		elevator.teleopPeriodic();
		cameraCommand();

		// LED.allianceColorLED();
		// LED.autonomousColorInit();
		// LED.rainbowShift();
		au.roundTo(0.0001);
		// System.out.println(au.getDistance1());
		SmartDashboard.putNumber("Sensor1", au.getDistance1());
		SmartDashboard.putNumber("Sensor2", au.getDistance2());

		// angle.getAngle();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}

	public void cameraCommand() {
		int intcam0 = 0;
		int intcam1 = 1;
		camera0 = new UsbCamera("USB Camera " + intcam0, intcam0);
		CameraServer.getInstance().addCamera(camera0);
		server = CameraServer.getInstance().addServer("serve_" + camera0.getName());
		camera1 = new UsbCamera("USB Camera " + intcam1, intcam1);
		CameraServer.getInstance().addCamera(camera1);
		curCam = camera0;
		SmartDashboard.putString("Current Cam", curCam.getName());
		server.setSource(camera0);
		if (toggler.toggle(driverIF.camSwitch()) == 1) {
			curCam = camera1;
		} else {
			curCam = camera0;
		}
	}

}
