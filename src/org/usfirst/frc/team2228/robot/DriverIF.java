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

	public boolean conveyorsForward() {
		if (xboxIF.RIGHT_TRIGGER() > 0.6)
			return true;
		else
			return false;
	}

	public boolean conveyorsBackward() {
		return xboxIF.Y_BUTTON();
	}

	public boolean hookForward() {
		return xboxIF.POV_RIGHT();
	}

	public boolean hookBackward() {
		return xboxIF.POV_LEFT();
	}

	// public boolean FrontConveyorForwards() {
	// return xboxIF.POV_RIGHT();
	// }
	// public boolean FrontConveyorBackwards() {
	// return xboxIF.POV_LEFT();
	// }

//	public boolean liftCube() {
//		return xboxIF.RS_BUTTON();
//	}

//	public boolean lowerCube() {
//		return xboxIF.LS_BUTTON();
//	}

	public boolean cubeRotateToggle() {
		return xboxIF.A_BUTTON();
	}

	public boolean collection() {
		return xboxIF.X_BUTTON();
	}

	public boolean expulsion() {
		return xboxIF.B_BUTTON();
	}

	public boolean squeeze() {
		return xboxIF.RS_BUTTON();
	}

	public boolean collectionToggle() {
		return xboxIF.LB_BUTTON();
	}

	public boolean release() {
		return xboxIF.LS_BUTTON();
	}

	public double Turn() {
		return xboxIF.RIGHT_JOYSTICK_X();
	}

	public double Throttle() {
		return xboxIF.LEFT_JOYSTICK_Y();
	}

	public boolean LowerElevator() {
		if (xboxIF.POV_DOWN())
			return true;
		else
			return false;
	}

	public boolean RaiseElevator() {
		if (xboxIF.POV_UP())

			return true;
		else
			return false;
	}
	public boolean squeezeToggle(){
		return xboxIF.RB_BUTTON();
	}

	public boolean winchWindUp() {
		return xboxIF.START_BUTTON();
	}
	public boolean winchWindDown(){
		return xboxIF.BACK_BUTTON();
	}

	public boolean elevatorToggleUp() {
		return xboxIF.POV_UP();
	}

	public boolean elevatorToggleDown() {
		return xboxIF.POV_DOWN();
	}
}
