package org.usfirst.frc.team2228.robot;

public class DriverIF {
	XboxIF xboxIF;
	public DriverIF() {
		xboxIF = new XboxIF();
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
	public double rightStickX() {
		return xboxIF.RIGHT_JOYSTICK_X();
	}
	public double leftStickY() {
		return xboxIF.LEFT_JOYSTICK_Y();
	}
	public double leftStickX() {
		return xboxIF.LEFT_JOYSTICK_X();
	}
}
