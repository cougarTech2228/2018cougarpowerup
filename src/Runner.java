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
	public int switch1;
	public int scale;
	public int switch2;
	public int force;
	public int boost;
	public int blocksInForceRed;
	public int blocksInBoostRed;
	public int blocksInForceBlue;
	public int blocksInBoostBlue;
	public int isForceActive;
	public int isBoostActive;
	public Runner() {
		super(700, 600, "no");
	}
	public static void main(String args[]) {
		new Runner();
	}
	//The player will be the red team the blue team will be controlled by the computer the red team owns switch 1 by default
	public void init() {
		t = new Timer(1000);
		g = new G2D();
		setBack(Color.white);
		g.setColor(Color.black);
		setFont("Papyrus", "Italics", 20);
		g.printFontList();
		t.Start();
		switch1 = -1;
		scale = -1;
		switch2 = -1;
		force = 0;
		boost = 0;
		blocksInForceRed = 3;
		blocksInBoostRed = 0;
		blocksInForceBlue = 0;
		blocksInBoostBlue = 0;
		isForceActive = 1;
		isBoostActive = 0;
	}
	public void update() {
		g.setColor(Color.RED);
		g.drawText("Red score " + redScore, new Vector(100, 50));
		g.drawText("Blocks In Force" + blocksInForceRed, new Vector(100, 70));
		if(force>0){
			g.drawText("Force Active", new Vector(100, 90));
			}
		g.drawText("Blocks In Boost" + blocksInBoostRed, new Vector(100, 110));
		if(boost>0){
			g.drawText("Boost Active", new Vector(100, 130));
			}
		
		g.setColor(Color.BLUE);
		g.drawText("Blue Score: " + blueScore, new Vector(400, 50));
		g.drawText("Blocks In Force" + blocksInForceBlue, new Vector(400, 70));
		if(force<0){
			g.drawText("Force Active", new Vector(400, 90));
			}
		g.drawText("Blocks In Boost" + blocksInBoostBlue, new Vector(400, 110));
		if(boost<0){
			g.drawText("Boost Active", new Vector(400, 130));
			}
		

			if(t.isDone()) {
		if(isForceActive == 1 && blocksInForceRed == 1){
			force = 1;
		}
		if(isForceActive == 1 && blocksInForceRed == 2){
			force = 2;
		}
		if(isForceActive == 1 && blocksInForceRed == 3){
			force = 3;
		}
		if(isForceActive == -1 && blocksInForceBlue == 1){
			force = -1;
		}
		if(isForceActive == -1 && blocksInForceBlue == 2){
			force = -2;
		}
		if(isForceActive == -1 && blocksInForceBlue == 3){
			force = -3;
		}
				
		if(force == 1){
		switch1 = 1;
		}
		if(force == 2){
		scale = 1;
		}
		if(force == 3){
		switch1 = 1;
		scale = 1;
		}
		if(force == -1){
		switch2 = 1;
		}
		if(force == -2){
		scale = 1;
		}
		if(force == -3){
		switch2 = 1;
		scale = 1;
		}
		
		if(isBoostActive == 1 && blocksInBoostRed == 1){
			boost = 1;
		}
		if(isBoostActive == 1 && blocksInBoostRed == 2){
			boost = 2;
		}
		if(isBoostActive == 1 && blocksInBoostRed == 3){
			boost = 3;
		}
		if(isBoostActive == -1 && blocksInBoostBlue == 1){
			boost = -1;
		}
		if(isBoostActive == -1 && blocksInBoostBlue == 2){
			boost = -2;
		}
		if(isBoostActive == -1 && blocksInBoostBlue == 3){
			boost = -3;
		}
		if(boost == 1 && switch1>0){
			redScore +=1;	
			}
		if(boost == 2 && scale>0){
			redScore +=1;
			}
		if(boost == 3 && switch1>0){
			redScore +=1;	
			}
		if(boost == 3 && scale>0){
			redScore +=1;	
			}
		if(boost == -1 && switch2<0){
			blueScore +=1;	
			}
		if(boost == -2 && scale<0){
			blueScore +=1;
			}
		if(boost == -3 && switch2<0){
			blueScore +=1;	
			}
		if(boost == -3 && scale<0){
			blueScore +=1;	
			}
			
			if(switch1>0){
			redScore += 1;
			}
			if(switch1<0){
			blueScore += 1;
			}
			
			if(scale>0){
				redScore += 1;
				}
			if(scale<0){
			blueScore += 1;
			}
			
			if(switch2>0){
				redScore += 1;
				System.out.println("yur mum");
				}
			if(switch2<0){
			blueScore +=1;
			}
			
			t = new Timer(1000);
			t.Start();
		}
		g.setColor(Color.GREEN);
		g.drawText(getKeys(), new Vector(50, 100));
	}

}
