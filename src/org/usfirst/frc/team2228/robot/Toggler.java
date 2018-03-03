package org.usfirst.frc.team2228.robot;

public class Toggler {
	public int state, states;
	private boolean on;

	public Toggler(int states) {
		state = 0;
		this.states = states;
		on = true;
	}
	public int get() {
		return state;
	}

	public int toggle(boolean buttonUp, boolean buttonDown) {
		if (buttonUp || buttonDown) {
//			System.out.println("Button " + iterateUp);
			if (on) {
				if (buttonUp)
					state += 1;
				else if (buttonDown)
					state -= 1;
				on = false;
			}
		} else {
//			System.out.println("No Button " + iterateUp);
			if (!on)
				on = true;
		}
//		if (state > states - 1)
//			state = 0;
//		if (state < 0)
//			state = states - 1;

		return state;
	}
	public int toggle(boolean button) {
		return toggle(button, false);
	}
	@Override
	public String toString() {
		return "state: " + state + " max: " + states;
	}
}
