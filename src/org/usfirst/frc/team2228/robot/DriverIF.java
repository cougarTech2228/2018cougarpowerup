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
	public boolean hookForward(){
		//return xboxIF.LB_BUTTON();
		return false;
		//did this for toggling the collection. NEEDS TO BE REMAPPED.
	}
	
	public boolean hookBackward(){
		//return xboxIF.RB_BUTTON();
		return false;
}

	public boolean FrontConveyorForwards() {
		return xboxIF.POV_RIGHT();
	}
	public boolean FrontConveyorBackwards() {
		return xboxIF.POV_LEFT();
	}
	
	public boolean liftCube() {
		return xboxIF.RS_BUTTON();
	}
	
	public boolean lowerCube() {
		return xboxIF.LS_BUTTON();
	}
	public boolean cubeRotateToggle(){
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
	public boolean collectionToggle(){
		return xboxIF.LB_BUTTON();
	}

	public boolean release() {
		return xboxIF.Y_BUTTON();
	}

	public double Turn() {
		return xboxIF.RIGHT_JOYSTICK_X();
	}

	public double Throttle() {
		
		return xboxIF.LEFT_JOYSTICK_Y();
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
	public boolean winchWind(){
		if(xboxIF.START_BUTTON()){
			return true;
		}
		else{
			return false;
		}
	}
}
