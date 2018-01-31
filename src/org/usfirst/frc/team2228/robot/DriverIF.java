package org.usfirst.frc.team2228.robot;

public class DriverIF {
	XboxIF xboxIF;

	public DriverIF() {
		xboxIF = new XboxIF();
	}

	public boolean raiseElevator() {
		return xboxIF.RB_BUTTON();
	}

	public boolean lowerElevator() {
		return xboxIF.LB_BUTTON();
	}
	
	public boolean liftCube() {
		return xboxIF.START_BUTTON();
	}
	
	public boolean lowerCube() {
		return xboxIF.BACK_BUTTON();
	}

	public boolean collection() {
		return xboxIF.X_BUTTON();
	}

	public boolean expulsion() {
		return xboxIF.B_BUTTON();
	}

	public boolean squeeze() {
		return xboxIF.A_BUTTON();
	}

	public boolean release() {
		return xboxIF.Y_BUTTON();
	}

	public double rightStickY() {
		return xboxIF.RIGHT_JOYSTICK_Y();
	}

	public double Turn() {
		return xboxIF.RIGHT_JOYSTICK_X();
	}

	public double Throttle() {
		return xboxIF.LEFT_JOYSTICK_Y();
	}

	public double leftStickX() {
		return xboxIF.LEFT_JOYSTICK_X();
	}
}
