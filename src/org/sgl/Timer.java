package org.sgl;

public class Timer extends Thread {
	private float time;
	private boolean finished = false;
	public Timer(int milliseconds) {
		time = milliseconds;
	}
	public void run() {
		try {
			Thread.sleep((long)time);
		} catch (Exception e) {}
		finished = true;
	}
	public boolean isDone() {
		if(finished) {
			finished = false;
			return true;
		}
		else return false;
	}
	public void Start() {
		try {
			this.start();
		} catch (Exception e) {}
	}
}
