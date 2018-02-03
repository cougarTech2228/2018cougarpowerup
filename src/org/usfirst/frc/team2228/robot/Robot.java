package org.usfirst.frc.team2228.robot;

import org.usfirst.frc.team2228.commands.StringCommand;

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
	private String input = "";
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();
	private TestDriveBase base;
	private StringCommand command;
	private CubeManipulator cube;
	private DriverIF driverIf;
	private ThingsUpHigh highThings;
	// private AnalogUltrasonic us;
	private PneumaticController pc;
	private AnalogUltrasonic au;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */

	@Override
	public void robotInit() {

		driverIf = new DriverIF();
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto choices", chooser);
		base = new TestDriveBase(driverIf);
		cube = new CubeManipulator(driverIf);
		//pc = new PneumaticController(driverIf);
		highThings = new ThingsUpHigh(driverIf);
		au = new AnalogUltrasonic();
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
		autoSelected = chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
		StringCommand command = new StringCommand(input);
		// command.start();
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();

		if (gameData.charAt(0) == 'L') {
			System.out.println("L");
		} else {
			System.out.println("R");
		}
		if (gameData.charAt(1) == 'L') {
			System.out.println("L");
		} else {
			System.out.println("R");
		}
		if (gameData.charAt(2) == 'L') {
			System.out.println("L");
		} else {
			System.out.println("R");
		}
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		// Scheduler.getInstance().run();
		switch (autoSelected) {
		case customAuto:
			// Put custom auto code here
			break;
		case defaultAuto:
		default:
			// Put default auto code here
			break;
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		// SmartDashboard.putNumber("Sonar", us.getDistance());
		base.teleopPeriodic();
		cube.teleopPeriodic();
		//pc.teleopPeriodic();
		highThings.teleopPeriodic();
		au.roundTo(0.0001);
		System.out.println(au.getDistance());
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}

}
