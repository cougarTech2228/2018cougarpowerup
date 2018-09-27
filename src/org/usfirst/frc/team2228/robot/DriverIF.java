package org.usfirst.frc.team2228.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;

public class DriverIF {
	XboxIF xboxIF;
	XboxIF xboxIF2;
	Toggler toggler;
	Toggler liftToggle, squeezeToggle, wheelForward, wheelBackward;
	int wheelState;
	public DriverIF() {
		xboxIF = new XboxIF(1);
		xboxIF2 = new XboxIF(2);
		toggler = new Toggler(2);
		liftToggle = new Toggler(2);
		squeezeToggle = new Toggler(2);
		wheelForward = new Toggler(2);
		wheelBackward = new Toggler(2);
	}

	public boolean fastSpeed(boolean toggle) {
		if(!toggle) {
		return xboxIF.Y_BUTTON();
		}
		else {
			Toggler toggler = new Toggler(2);
			if(toggler.toggle(xboxIF.Y_BUTTON()) == 1) {
			return true;
			}
		}
		return false;
	}
	public boolean lowerSpeed() {
		return xboxIF.X_BUTTON();
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

//	public boolean cubeRotateToggle() {
//		if (xboxIF.A_BUTTON() || xboxIF2.A_BUTTON()) {
//			return true;
//		} else {
//			return false;
//		}
//	}
	public boolean cubeLiftToggle() {
		liftToggle.toggle(xboxIF.LB_BUTTON());
		return liftToggle.getBool();
	}
	public boolean cubeGrapToggle() {
		squeezeToggle.toggle(xboxIF.RB_BUTTON());
		return squeezeToggle.getBool();
	}
	public int wheelState() {
		int prev_a = wheelForward.state;
		int prev_b = wheelBackward.state;
		int a = wheelForward.toggle(xboxIF.A_BUTTON());
		int b = wheelBackward.toggle(xboxIF.B_BUTTON());
		
		if(prev_a == a && prev_b != b) {
			wheelState = -b;
			wheelForward.state = 0;
		}
		else if(prev_b == b && prev_a != a) {
			wheelState = a;
			wheelBackward.state = 0;
		}
		
		return wheelState;//wheelState;
	}
	

//	public boolean expulsion() {
//		if (xboxIF.B_BUTTON() || xboxIF2.B_BUTTON()) {
//			return true;
//		} else {
//			return false;
//		}
//	}
//
//	public boolean collectionToggle() {
//		if (xboxIF.LB_BUTTON() || xboxIF2.LB_BUTTON()) {
//			return true;
//		} else {
//			return false;
//		}//
//	}

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

	public boolean camSwitch() {
		return xboxIF.POV_LEFT();
	}
	public void rumbleSet(boolean on, double rumbleSpeed) {
		if(on) {
		xboxIF.RUMBLE(rumbleSpeed);
		}
		else {
			xboxIF.RUMBLE(0);
		}
	}

}
