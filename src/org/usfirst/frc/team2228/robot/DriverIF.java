package org.usfirst.frc.team2228.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;

public class DriverIF {
	XboxIF xboxIF;
	XboxIF xboxIF2;

	public DriverIF() {
		xboxIF = new XboxIF(1);
		xboxIF2 = new XboxIF(2);
	}

	public boolean conveyorsForward() {
		if (xboxIF.Y_BUTTON() || xboxIF2.Y_BUTTON())
			return true;
		else
			return false;
	}

	public boolean conveyorsBackward() {
		if (xboxIF.X_BUTTON() || xboxIF2.X_BUTTON()) {
			return true;
		} else {
			return false;
		}
	}

	public boolean hookForward() {
		if (xboxIF.POV_UP() || xboxIF2.POV_UP()) {
			return true;
		} else {
			return false;
		}
	}

	public boolean hookBackward() {
		if (xboxIF.POV_DOWN() || xboxIF2.POV_DOWN()) {
			return true;
		} else {
			return false;
		}
	}

	public boolean cubeRotateToggle() {
		if (xboxIF.A_BUTTON() || xboxIF2.A_BUTTON()) {
			return true;
		} else {
			return false;
		}
	}

	public boolean expulsion() {
		if (xboxIF.LB_BUTTON() || xboxIF2.LB_BUTTON()) {
			return true;
		} else {
			return false;
		}
	}

	public boolean collectionToggle() {
		if (xboxIF.B_BUTTON() || xboxIF2.B_BUTTON()) {
			return true;
		} else {
			return false;
		}
	}

	public double Turn() {
		return xboxIF.RIGHT_JOYSTICK_X();

	}

	public double Throttle() {
		return xboxIF.LEFT_JOYSTICK_Y();
	}

	public boolean LowerElevator() {
		if (xboxIF.LEFT_TRIGGER() > 0.6 || xboxIF2.LEFT_TRIGGER() > 0.6)
			return true;
		else
			return false;
	}

	public boolean RaiseElevator() {
		if (xboxIF.RIGHT_TRIGGER() > 0.6 || xboxIF2.RIGHT_TRIGGER() > 0.6)
			return true;
		else
			return false;
	}

	public boolean squeezeToggle() {
		if (xboxIF.RB_BUTTON() || xboxIF2.RB_BUTTON()) {
			return true;
		} else {
			return false;
		}
	}

	public boolean winchWindUp() {
		if (xboxIF.START_BUTTON() || xboxIF2.START_BUTTON()) {
			return true;
		} else {
			return false;
		}
	}

	public boolean elevatorToggleUp() {
		if (xboxIF.POV_UP() || xboxIF2.POV_UP()) {
			return true;
		} else {
			return false;
		}
	}

	public boolean elevatorToggleDown() {
		if (xboxIF.POV_DOWN() || xboxIF2.POV_DOWN()) {
			return true;
		} else {
			return false;
		}
	}

}
