/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robots.subsystems;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robots.commands.SetUSBCamera;

/**
 * Add your docs here.
 */

public class USBCamera extends Subsystem {
  private UsbCamera camera1;
  private UsbCamera cam2;
  private VideoSink server;
  private boolean camMode = false;

  public USBCamera() {
    camera1 = CameraServer.getInstance().startAutomaticCapture(0);
    cam2 = CameraServer.getInstance().startAutomaticCapture(1);
    // CameraServer.getInstance().startAutomaticCapture();
    // CameraServer.getInstance().getVideo(camera1);
    // CameraServer.getInstance().getVideo(cam2);
    // server = CameraServer.getInstance().getServer();



    
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    // setDefaultCommand(new SetUSBCamera());
  }

  public void setCamera1() {
    server.setSource(camera1);
  }

}
