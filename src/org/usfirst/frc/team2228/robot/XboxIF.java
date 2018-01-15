package org.usfirst.frc.team2228.robot;

import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
/*
public interface XboxIF {
	public default void Use() {XboxVars x = new XboxVars();}

	public default boolean A_BUTTON() {return XboxVars.xbox.getAButton();};
	public default boolean B_BUTTON() {return XboxVars.xbox.getBButton();};
	public default boolean X_BUTTON() {return XboxVars.xbox.getXButton();};
	public default boolean Y_BUTTON() {return XboxVars.xbox.getYButton();};
	
	public default boolean START_BUTTON() {return XboxVars.xbox.getStartButton();};
	public default boolean BACK_BUTTON() {return XboxVars.xbox.getBackButton();};
	public default boolean RB_BUTTON() {return XboxVars.xbox.getBumper(Hand.kRight);};
	public default boolean LB_BUTTON() {return XboxVars.xbox.getBumper(Hand.kLeft);};
	public default boolean LS_BUTTON() {return XboxVars.xbox.getStickButton(Hand.kLeft);};
	public default boolean RS_BUTTON() {return XboxVars.xbox.getStickButton(Hand.kRight);};
	
	public default double RIGHT_TRIGGER() {return XboxVars.xbox.getTriggerAxis(Hand.kRight);};
	public default double LEFT_TRIGGER() {return XboxVars.xbox.getTriggerAxis(Hand.kLeft);};
	
	public default double RIGHT_JOYSTICK_X() {return XboxVars.xbox.getX(Hand.kRight);};
	public default double RIGHT_JOYSTICK_Y() {return XboxVars.xbox.getY(Hand.kRight);};
	public default double LEFT_JOYSTICK_X() {return XboxVars.xbox.getX(Hand.kLeft);};
	public default double LEFT_JOYSTICK_Y() {return XboxVars.xbox.getY(Hand.kRight);};
	
}*/
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
	public double LEFT_JOYSTICK_Y() {return xbox.getY(Hand.kRight);};
}
