package org.usfirst.frc.team2228.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class XboxIF {
	private XboxController xbox;
	public XboxIF() {
		xbox = new XboxController(1);
	
	}
	
	public boolean A_BUTTON() {return xbox.getAButton();};
	public boolean B_BUTTON() {return xbox.getBButton();};
	public boolean X_BUTTON() {return xbox.getXButton();};
	public boolean Y_BUTTON() {return xbox.getYButton();};
	
	public boolean START_BUTTON() {return xbox.getStartButton();};
	public boolean BACK_BUTTON() {return xbox.getBackButton();};
	public boolean RB_BUTTON() {return xbox.getBumper(Hand.kRight);};
	public boolean LB_BUTTON() {return xbox.getBumper(Hand.kLeft);};
	public boolean LS_BUTTON() {return xbox.getStickButton(Hand.kLeft);};
	public boolean RS_BUTTON() {return xbox.getStickButton(Hand.kRight);};
	
	public double RIGHT_TRIGGER() {return xbox.getTriggerAxis(Hand.kRight);};
	public double LEFT_TRIGGER() {return xbox.getTriggerAxis(Hand.kLeft);};
	
	public double RIGHT_JOYSTICK_X() {return xbox.getX(Hand.kRight);};
	public double RIGHT_JOYSTICK_Y() {return xbox.getY(Hand.kRight);};
	public double LEFT_JOYSTICK_X() {return xbox.getX(Hand.kLeft);};
	public double LEFT_JOYSTICK_Y() {return xbox.getY(Hand.kLeft);};
	
	//public int D_PAD_VERTICAL() {return }
	//public int D_PAD_HORIZONTAL() {(GenericHID)xbox}
}
