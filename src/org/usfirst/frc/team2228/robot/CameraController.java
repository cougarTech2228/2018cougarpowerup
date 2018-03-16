package org.usfirst.frc.team2228.robot;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CameraController {
	private UsbCamera camera0;
	private VideoSink server;
	private UsbCamera camera1;
	private Toggler toggler;
	private DriverIF driverIF;
	private boolean cam0Used;
	private boolean cam1Used;

	public CameraController(DriverIF _driverIF) {
		toggler = new Toggler(2);
		this.driverIF = _driverIF;
		int intcam0 = 0;
		int intcam1 = 1;
		// camera0 = new UsbCamera("USB Camera " + intcam0, intcam0);
		camera0 = CameraServer.getInstance().startAutomaticCapture(0);
		// server = CameraServer.getInstance().addServer("serve_" + camera0.getName());
		// camera1 = new UsbCamera("USB Camera " + intcam1, intcam1);
		camera1 = CameraServer.getInstance().startAutomaticCapture(1);
		server = CameraServer.getInstance().getServer();
		// CameraServer.getInstance().addCamera(camera1);
		server.setSource(camera0);
	}

	public void cameraCommand() {
		if (toggler.toggle(driverIF.camSwitch()) == 0) {
			if (cam0Used) {
				server.setSource(camera1);
				cam0Used = false;
				System.out.println("Cam1");
			}

		} else {
			if (!cam0Used) {
				server.setSource(camera0);
				cam0Used = true;
				// System.out.println("Cam0");
			}
		}

	}
}
