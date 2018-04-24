package org.usfirst.frc.team2228.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;

public class DriverIF {
	XboxIF xboxIF;
	XboxIF xboxIF2;
	Toggler toggler;
	Toggler[] toggles;
	public DriverIF() {
		xboxIF = new XboxIF(1);
		xboxIF2 = new XboxIF(2);
		toggler = new Toggler(2);
		toggles = new Toggler[12];
		for(int i = 0; i < toggles.length; i++)
			toggles[i] = new Toggler(2);
	}

	public boolean fastSpeed(boolean Toggle) {
		boolean button = xboxIF.Y_BUTTON();
		if(Toggle) {
			return (toggles[11].toggle(button) == 1) ? true : false;	
		}
		else
			return button;
	}
	public boolean lowerSpeed(boolean Toggle) {
		boolean button = xboxIF.X_BUTTON();
		if(Toggle) {
			return (toggles[0].toggle(button) == 1) ? true : false;	
		}
		else
			return button;
	}

	public boolean hookForward(boolean Toggle) {
		boolean button = (xboxIF.POV_UP() || xboxIF2.POV_UP());
		if(Toggle) {
			return (toggles[1].toggle(button) == 1) ? true : false;	
		}
		else
			return button;
	}

	public boolean hookBackward(boolean Toggle) {
		boolean button = (xboxIF.POV_DOWN() || xboxIF2.POV_DOWN());
		if(Toggle)
			return (toggles[2].toggle(button) == 1) ? true : false;	
		else
			return button;
	}

	public boolean cubeRotateToggle(boolean Toggle) {
		boolean button = (xboxIF.A_BUTTON() || xboxIF2.A_BUTTON());
		if(Toggle)
			return (toggles[3].toggle(button) == 1) ? true : false;	
		else
			return button;
	}

	public boolean expulsion(boolean Toggle) {
		boolean button = (xboxIF.B_BUTTON() || xboxIF2.B_BUTTON());
		if(Toggle)
			return (toggles[4].toggle(button) == 1) ? true : false;	
		else
			return button;
	}

	public boolean collectionToggle(boolean Toggle) {
		boolean button = (xboxIF.LB_BUTTON() || xboxIF2.LB_BUTTON());
		if(Toggle)
			return (toggles[5].toggle(button) == 1) ? true : false;	
		else
			return button;
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

	public boolean squeezeToggle(boolean Toggle) {
		boolean button = (xboxIF.RB_BUTTON() || xboxIF2.RB_BUTTON());
		if(Toggle)
			return (toggles[6].toggle(button) == 1) ? true : false;	
		else
			return button;
	}

	public boolean winchWindUp(boolean Toggle) {
		boolean button = (xboxIF.START_BUTTON() || xboxIF2.START_BUTTON());
		if(Toggle)
			return (toggles[7].toggle(button) == 1) ? true : false;	
		else
			return button;
	}

	public boolean elevatorToggleUp(boolean Toggle) {
		boolean button = (xboxIF.POV_UP() || xboxIF2.POV_UP());
		if(Toggle)
			return (toggles[8].toggle(button) == 1) ? true : false;	
		else
			return button;
	}

	public boolean elevatorToggleDown(boolean Toggle) {
		boolean button = (xboxIF.POV_DOWN() || xboxIF2.POV_DOWN());
		if(Toggle)
			return (toggles[9].toggle(button) == 1) ? true : false;	
		else
			return button;
	}

	public boolean camSwitch(boolean Toggle) {
		boolean button = xboxIF.POV_LEFT();
		if(Toggle)
			return (toggles[10].toggle(button) == 1) ? true : false;	
		else
			return button;
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
