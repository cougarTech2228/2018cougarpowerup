package org.usfirst.frc.team2228.robot;

public class Toggler {
	private int state, states;
	boolean on;

	public Toggler(int states) {
		state = 0;
		this.states = states;
		on = true;
	}

	public int toggle(boolean button, boolean iterateUp) {
		if (button) {
//			System.out.println("Button " + iterateUp);
			if (on) {
				if (iterateUp)
					state += 1;
				else
					state -= 1;
				on = false;
			}
		} else {
//			System.out.println("No Button " + iterateUp);
			if (!on)
				on = true;
		}
		if (state > states - 1)
			state = 0;

		return state;
	}
}
