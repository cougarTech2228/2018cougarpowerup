package org.sgl;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.GraphicsEnvironment;

public class G2D {
	protected static Graphics2D g;
	private String[] data;
	public G2D() {
		
	}
	public void setColor(Color c) {
		g.setColor(c);
	}
	public void draw(Vector[] points) {
		int[] x = new int[points.length];
		int[] y = new int[points.length];
		for(int i = 0; i < points.length; i++) {
			x[i] = (int) points[i].x;
			y[i] = (int) points[i].y;
		}
		g.fillPolygon(x, y, points.length);
	}
	public void drawLine(Vector point1, Vector point2) {
		g.drawLine((int)point1.x, (int)point1.y, (int)point2.x, (int)point2.y);
	}
	public void drawRect(Vector corner, Vector opCorner) {
		int[] x = {(int)corner.x, (int)opCorner.x, (int)opCorner.x, (int)corner.x};
		int[] y = {(int)corner.y, (int)corner.y, (int)opCorner.y, (int)opCorner.y};
		
		g.fillPolygon(x, y, 4);
		
	}
	public void drawText(String text, Vector start) {
		g.drawString(text, start.x, start.y + g.getFont().getSize() / 2);
	}
	
	
	public void printFontList() {
		for(String s : GraphicsEnvironment.getLocalGraphicsEnvironment().getAvailableFontFamilyNames())
		System.out.println();
	}
}
