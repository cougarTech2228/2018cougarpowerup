package org.usfirst.frc.team2228.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;

public class DriverIF {
	XboxIF xboxIF;

	public DriverIF() {
		xboxIF = new XboxIF();
	}
	public boolean BackConveyorForwards() {
		return xboxIF.POV_UP();
	}

	public boolean BackConveyorBackwards() {
		return xboxIF.POV_DOWN();
	}

	public boolean FrontConveyorForwards() {
		return xboxIF.POV_RIGHT();
	}
	public boolean FrontConveyorBackwards() {
		return xboxIF.POV_LEFT();
	}
	
	public boolean liftCube() {
		return xboxIF.RB_BUTTON();
	}
	
	public boolean lowerCube() {
		return xboxIF.LB_BUTTON();
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
	public boolean LowerElevator() {
		if(xboxIF.LEFT_TRIGGER() > 0.6)
			return true;
		else
			return false;
	}
	public boolean RaiseElevator() {
		if(xboxIF.RIGHT_TRIGGER() > 0.6)
			return true;
		else
			return false;
	}
	public boolean Xbutton() {
		return xboxIF.START_BUTTON();
	}
}
