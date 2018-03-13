package org.usfirst.frc.team2228.robot;

//import com.mindsensors.CANLight;

import edu.wpi.first.wpilibj.DriverStation;

public class CANLED {
//	int r = 255;
//	int g = 0;
//	int b = 0;
//
//	CANLight frameLights;
//	DriverStation ds;
//	DriverStation ds2;
//
//	public void colorInit() {
//		frameLights = new CANLight(15);
//		ds2 = ds.getInstance();
//	}
//
//	public void allianceColorLED() {
//		if (ds2.getAlliance() == DriverStation.Alliance.Red) {
//			System.out.println("Red Alliance Detected");
//			frameLights.showRGB(255, 0, 0);
//		} else if (ds2.getAlliance() == DriverStation.Alliance.Blue) {
//			System.out.println("Blue Alliance Detected");
//			frameLights.showRGB(0, 0, 255);
//		} else if (ds2.getAlliance() == DriverStation.Alliance.Invalid) {
//			System.out.println("No Alliance Detected");
//			frameLights.showRGB(255, 200, 0); // yellow
//		}
//	}
//
//	public void autonomousColorInit() {
//		USAColor();
//		// RainbowLED;
//	}
//
//	public void USAColor() {
//		frameLights.writeRegister(1, 0.5, 255, 0, 0); // red
//		frameLights.writeRegister(2, 0.5, 82, 101, 110); // white
//		frameLights.writeRegister(3, 0.5, 0, 0, 255); // blue
//		frameLights.cycle(1, 3);
//
//	}
//
//	public void rainbowShift() {
//
//		// for(g = 0; g < 255; g++){
//		// frameLights.showRGB(r, g, b);
//		// }
//		// for(r = 255; r > 0; r--){
//		// frameLights.showRGB(r, g, b);
//		// }
//		if (r == 255 && g < 255 && b == 0) {
//			frameLights.showRGB(r, g, b);
//			g++;
//			System.out.println(r);
//			System.out.println(g);
//			System.out.println(b);
//		}
//		if (g == 255 && r > 0 && b == 0) {
//			frameLights.showRGB(r, g, b);
//			r--;
//			System.out.println(r);
//			System.out.println(g);
//			System.out.println(b);
//		}
//		if (g == 255 && b < 255 && r == 0) {
//			b++;
//			frameLights.showRGB(r, g, b);
//			System.out.println(r);
//			System.out.println(g);
//			System.out.println(b);
//		}
//		if (b == 255 && g > 0 && r == 0) {
//			g--;
//			frameLights.showRGB(r, g, b);
//			System.out.println(r);
//			System.out.println(g);
//			System.out.println(b);
//		}
//		if (b == 255 && r < 255 && g == 0) {
//			r++;
//			frameLights.showRGB(r, g, b);
//			System.out.println(r);
//			System.out.println(g);
//			System.out.println(b);
//		}
//		if (r == 255 && b > 0 && g == 0) {
//			b--;
//			frameLights.showRGB(r, g, b);
//			System.out.println(r);
//			System.out.println(g);
//			System.out.println(b);
//		}
//
//	}
}
