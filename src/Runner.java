import java.awt.Color;

import org.sgl.G2D;
import org.sgl.Game;
import org.sgl.Timer;
import org.sgl.Vector;

public class Runner extends Game {
	private G2D g;
	private Timer t;
	public int redScore = 0;
	public int blueScore = 0;
	
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
		setFont("Papyrus", "Italics", 20);
		g.printFontList();
		t.Start();
	}
	public void update() {
		g.setColor(Color.RED);
		g.drawText("Red score " + redScore, new Vector(100, 50));
		g.setColor(Color.BLUE);
		g.drawText("Blue Score: " + blueScore, new Vector(400, 50));
		
		if(t.isDone()) {
			System.out.println("hi");
			t = new Timer(1000);
			t.Start();
		}
		g.setColor(Color.GREEN);
		g.drawText(getKeys(), new Vector(50, 100));
		
	}

}
