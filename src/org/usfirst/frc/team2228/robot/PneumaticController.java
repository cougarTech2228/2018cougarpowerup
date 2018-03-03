package org.usfirst.frc.team2228.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PneumaticController {
	boolean on = true;
	boolean off = false;
	public DriverIF driverIF;
	public Compressor c = new Compressor(RobotMap.CAN_ID_10);
	private Toggler toggle = new Toggler(2);
	private Toggler toggle2 = new Toggler(2);

	public Solenoid squeezies = new Solenoid(RobotMap.CAN_ID_10, RobotMap.PCM_PORT_0);
	public Solenoid lift = new Solenoid(RobotMap.CAN_ID_10, RobotMap.PCM_PORT_1);
	public Solenoid brake = new Solenoid(RobotMap.CAN_ID_10, RobotMap.PCM_PORT_3);

	public boolean pressureSwitch = c.getPressureSwitchValue();
	private boolean liftState = true;
	private boolean squeezeState = true;
	public PneumaticController(DriverIF _driverIF) {
		driverIF = _driverIF;
		brakeSet(false);
		SmartDashboard.putBoolean("liftState", liftState);
		SmartDashboard.putBoolean("squeezeState", squeezeState);
	}

	public void teleopPeriodic() {
		c.setClosedLoopControl(true);
		squeezeToggle(driverIF.squeezeToggle(), true, squeezies);
		liftToggle(driverIF.cubeRotateToggle(), true, lift);
		SmartDashboard.putBoolean("liftState", liftState);
		SmartDashboard.putBoolean("squeezeState", squeezeState);
	}

	public void liftSet(boolean state) {
		lift.set(state);
		liftState = state;
	}
	public void smartDashboardLiftSet() {
		lift.set(SmartDashboard.getBoolean("liftState", liftState));
		liftState = SmartDashboard.getBoolean("liftState", liftState);
	}

	public void brakeSet(boolean state) {
		brake.set(state);
	}
	public void squeezeSet(boolean state) {
		squeezies.set(state);
		squeezeState = state;
	}

	/**
	 * 
	 * @param button
	 *            - button you would like to toggle
	 * @param onDirection
	 *            - primary direction of solenoid
	 * @param solenoid
	 *            - solenoid user would like to activate
	 */
	public void	liftToggle(boolean button, boolean onDirection,
			Solenoid solenoid) {
			
			if(toggle.toggle(button) == 0) {
				solenoid.set(onDirection);
			}
			else {
				solenoid.set(!onDirection);
			}
	}
	public void squeezeToggle(boolean button, boolean onDirection, Solenoid solenoid) {
		if(toggle2.toggle(button) == 0) {
			solenoid.set(onDirection);
		}
		else {
			solenoid.set(!onDirection);
		}
	}
	public boolean getLiftState() {
		if(toggle.state == 0) {
			return false;
		}
		return true;
	}
	public boolean getSqueezeState() {
		if(toggle2.state == 0) {
			return false;
		}
		return true;
	}
	// beep bop boopedy beep beep

}
