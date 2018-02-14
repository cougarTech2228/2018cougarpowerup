import java.awt.Color;

import org.sgl.G2D;
import org.sgl.Game;
import org.sgl.Timer;
import org.sgl.Vector;

public class Runner extends Game {
	private G2D g;
	private Timer t;
	
	public Runner() {
		super(700, 600, "no");
	}
	public static void main(String args[]) {
		new Runner();
	}
	public void init() {
		t = new Timer(1000);
		g = new G2D();
		setBack(Color.white);
		g.setColor(Color.black);
		setFont("Arial", "Bold", 50);
	}
	public void update() {

		t.Start();
		if(t.isDone()) {
			System.out.println("hi");
		}
		System.out.println("test");
		g.setColor(Color.yellow);
		g.drawRect(new Vector(0, 100), new Vector(100, 200));
		g.setColor(Color.black);
		g.drawText(getKeys(), new Vector(0, 100));
		
	}
}
