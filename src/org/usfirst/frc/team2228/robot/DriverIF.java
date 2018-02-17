package org.usfirst.frc.team2228.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;

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
	public double Turn() {
		return xboxIF.RIGHT_JOYSTICK_X();
	}
	public double Throttle() {
		return xboxIF.LEFT_JOYSTICK_Y();
	}
	public double leftStickX() {
		return xboxIF.LEFT_JOYSTICK_X();
	}
	public boolean RB_Button() {
		return xboxIF.RB_BUTTON();
	}
	public boolean cascadeBotton(){
		return xboxIF.LB_BUTTON();
	}
	public boolean Xbutton() {
		return xboxIF.START_BUTTON();
	}
}
