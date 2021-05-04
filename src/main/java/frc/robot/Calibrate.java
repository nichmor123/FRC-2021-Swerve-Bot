// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class Calibrate {
  public Joystick baseJS = new Joystick(0);
  public DriveSubsystem m_drive = new DriveSubsystem(baseJS);

  public Calibrate() {
    m_drive.ZeroAll();
    System.out.println("All Motors Zeroed");
  }
}
