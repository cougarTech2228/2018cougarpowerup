package org.usfirst.frc.team2228.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class LogitechIF {
	// Gamepad axis
		public static final int kGamepadAxisLeftStickX = 1;
		public static final int kGamepadAxisLeftStickY = 2;
		public static final int kGamepadAxisShoulder = 3;
		public static final int kGamepadAxisRightStickX = 4;
		public static final int kGamepadAxisRightStickY = 5;
		public static final int kGamepadAxisDpad = 6;

		// Gamepad buttons
		public static final int kGamepadButtonA = 1; // Bottom Button    //Done
		public static final int kGamepadButtonB = 2; // Right Button    //Done
		public static final int kGamepadButtonX = 3; // Left Button		//Done
		public static final int kGamepadButtonY = 4; // Top Button		//Done
		public static final int kGamepadButtonShoulderL = 5;			//Done
		public static final int kGamepadButtonShoulderR = 6;			//Done
		public static final int kGamepadButtonBack = 7;				//Done
		public static final int kGamepadButtonStart = 8;			//Done
		public static final int kGamepadButtonLeftStick = 9;		//Done
		public static final int kGamepadButtonRightStick = 10;		//Done
		public static final int kGamepadButtonMode = -1;
		public static final int kGamepadButtonLogitech = -1;
		Joystick logitechController;
	public LogitechIF() {
		logitechController = new Joystick(1);
		
	}
	public boolean A_BUTTON() {
		return logitechController.getRawButton(kGamepadButtonA);	
	};

	public boolean B_BUTTON() {
		return logitechController.getRawButton(kGamepadButtonB);
	};

	public boolean X_BUTTON() {
		return logitechController.getRawButton(kGamepadButtonX);
	};

	public boolean Y_BUTTON() {
		return logitechController.getRawButton(kGamepadButtonY);
	};

	public boolean START_BUTTON() {
		return logitechController.getRawButton(kGamepadButtonStart);
	};

	public boolean BACK_BUTTON() {
		return logitechController.getRawButton(kGamepadButtonBack);
	};

	public boolean RB_BUTTON() {
		return logitechController.getRawButton(kGamepadButtonShoulderR);
	};

	public boolean LB_BUTTON() {
		return logitechController.getRawButton(kGamepadButtonShoulderL);
	};

	public boolean LS_BUTTON() {
		return logitechController.getRawButton(kGamepadButtonLeftStick);
	};

	public boolean RS_BUTTON() {
		return logitechController.getRawButton(kGamepadButtonRightStick);
	};

	public double RIGHT_TRIGGER() {
		
	};

	public double LEFT_TRIGGER() {
		return xbox.getTriggerAxis(Hand.kLeft);
	};

	public double RIGHT_JOYSTICK_X() {
		return xbox.getX(Hand.kRight);
	};

	public double RIGHT_JOYSTICK_Y() {
		return xbox.getY(Hand.kRight);
	};

	public double LEFT_JOYSTICK_X() {
		return xbox.getX(Hand.kLeft);
	};

	public double LEFT_JOYSTICK_Y() {
		return xbox.getY(Hand.kLeft);
	};

	public boolean POV_UP() {
		if (xbox.getPOV(0) == 0) {
			return true;
		} else {
			return false;
		}
	}
	public boolean POV_RIGHT() {
		if (xbox.getPOV(0) == 90) {
			return true;
		} else {
			return false;
		}
	}
	public boolean POV_DOWN() {
		if (xbox.getPOV(0) == 180) {
			return true;
		} else {
			return false;
		}
		
	}
	public boolean POV_LEFT() {
		if (xbox.getPOV(0) == 270) {
			return true;
		} else {
			return false;
		}
	}
	
}
