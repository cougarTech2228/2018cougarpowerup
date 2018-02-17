package org.sgl;
import javax.swing.*;

import java.awt.*;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.image.BufferStrategy;
import java.util.ArrayList;

public abstract class Game extends JFrame implements KeyListener {
	
	private ArrayList<Integer> KeyList = new ArrayList<Integer>();
	private String keydata = "";
	//private static Game game;
	public int key;
    private final int WIDTH;
    private final int HEIGHT;
    private BufferStrategy strategy;
    private Graphics2D g;
    
    //loop vars
    private boolean isRunning = true; //is the window running
    private long rest = 0; //how long to sleep the main thread

    //timing variables
    private double dt; //delta time
    private long lastFrame; //time since last frame
    private long startFrame; //time since start of frame
    private int fps; //current fps
    private int maxFps = 10;

    public Game(int width, int height, String title){
        super(title);
        this.WIDTH = width;
        this.HEIGHT = height;
        this.run();
    }
    public int getFps() {
    	return fps;
    }
    public void setFps(int fps) {
    	maxFps = fps;
    }
    private void GameInit(){
    	lastFrame = System.currentTimeMillis();
        setBounds(0, 0, WIDTH, HEIGHT);
        setResizable(true);
        setVisible(true);
        createBufferStrategy(2);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        strategy = getBufferStrategy();
        addKeyListener(this);
        setFocusable(true);
        setBackground(Color.WHITE);
        //setFont(new Font("Arial", Font.BOLD, 20));
        G2D.g = (Graphics2D) strategy.getDrawGraphics();
        init();
        //Run.setup();
    }
    private void keyAdd(int key) {
    	if(!hasKey(key))
    		KeyList.add(KeyList.size(), key);
    }
    private void keyRemove(int key) {
    	if(hasKey(key))
    		KeyList.remove(KeyList.indexOf(key));
    }
    public void setBack(Color c) {
    	setBackground(c);
    }
    public boolean hasKey(int key) {
    	return KeyList.contains(key);
    }
    public String getKeys() {
    	return keydata;
    }
    public void clearKeys() {
    	keydata = "";
    }
    public void setFont(String font, String style, int size) {
		int Istyle = 0;
		switch(style) {
		case "Plain":
			Istyle = Font.PLAIN;
			break;
		case "Bold":
			Istyle = Font.BOLD;
			break;
		case "Italic":
			Istyle = Font.ITALIC;
			break;
		}
		setFont(new Font(font, Istyle, size));
    }
    private void GameUpdate() {
    	//setFont(G2D.font);
    	G2D.g = (Graphics2D) strategy.getDrawGraphics();
    	G2D.g.clearRect(0,0,WIDTH, HEIGHT);
        fps = (int) (1f / dt);
        update();
        G2D.g.dispose();
        strategy.show();
    }
    private void run() {
        GameInit();
        while(isRunning){
            //new loop, clock the start
            startFrame = System.currentTimeMillis();
            //calculate delta time
            dt = (double)(startFrame - lastFrame)/1000;
            //update lastFrame for next dt
            lastFrame = startFrame;
            GameUpdate();
            rest = (1000/maxFps) - (System.currentTimeMillis() - startFrame);
            if(rest > 0){ //if we stayed within frame "budget", sleep away the rest of it
                try{ Thread.sleep(rest); }
                catch (Exception e){ e.printStackTrace(); }
            }
        }
    }
    public abstract void init();
    public abstract void update();
    
    @Override
    public void keyPressed(KeyEvent keyEvent) {
    	keyAdd(keyEvent.getKeyCode());
    	char c = keyEvent.getKeyChar();
    	if (c == KeyEvent.VK_BACK_SPACE) {
    		if(keydata.length() > 0)
    			keydata = keydata.substring(0, keydata.length() - 1);
    	}
    	else if (c != KeyEvent.CHAR_UNDEFINED && c != KeyEvent.VK_DELETE)
    		keydata += c;
    }
    @Override
    public void keyReleased(KeyEvent keyEvent) {keyRemove(keyEvent.getKeyCode());}
	@Override
	public void keyTyped(KeyEvent e) {}
	
	public void keypress(int code) {}
	public void keyrelease(int code) {}
}