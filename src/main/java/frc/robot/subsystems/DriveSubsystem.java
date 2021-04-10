// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  private Joystick m_baseJS;

  public static final int frontLeftDriveId = 4;
  public static final int frontLeftCANCoderId = 20;
  public static final int frontLeftSteerId = 6;
  // put your can Id's here!
  public static final int frontRightDriveId = 8;
  public static final int frontRightCANCoderId = 21;
  public static final int frontRightSteerId = 6;
  // put your can Id's here!
  public static final int backLeftDriveId = 9;
  public static final int backLeftCANCoderId = 22;
  public static final int backLeftSteerId = 2;
  // put your can Id's here!

  public static final int backRightDriveId = 3;
  public static final int backRightCANCoderId = 23;
  public static final int backRightSteerId = 7;

  private final TalonFX frontLeftPower = new TalonFX(frontLeftDriveId);
  private final TalonFX frontLeftAngle = new TalonFX(frontLeftSteerId);

  private final TalonFX frontRightPower = new TalonFX(frontRightDriveId);
  private final TalonFX frontRightAngle = new TalonFX(frontRightSteerId);

  private final TalonFX backLeftPower = new TalonFX(frontLeftDriveId);
  private final TalonFX backLeftAngle = new TalonFX(backLeftSteerId);

  private final TalonFX backRightPower = new TalonFX(backRightDriveId);
  private final TalonFX backRightAngle = new TalonFX(backRightSteerId);

  public DriveSubsystem(Joystick baseJS) {
    this.m_baseJS = baseJS;
  }

  public void SwerveDrive() {
    double x = m_baseJS.getRawAxis(0);
    double y = m_baseJS.getRawAxis(1);
    double z = m_baseJS.getRawAxis(4);

    double FWD = -y;
    double STR = x;
    double RCW = z;

    double temp = -y * Math.cos(0) + x * Math.sin(0);

    STR = y * Math.sin(0) + x * Math.cos(0);
    FWD = temp;

    double L = RobotContainer.Length;
    double W = RobotContainer.Width;
    double R = Math.sqrt(Math.pow(L, 2) + Math.pow(W, 2));

    double A = STR - RCW * (L / W);
    double B = STR + RCW * (L / R);
    double C = FWD - RCW * (W / R);
    double D = FWD + RCW * (W / R);

    double ws1 = Math.sqrt(Math.pow(B, 2) + Math.pow(C, 2));
    double wa1 = Math.atan2(B, C) * 180 / Math.PI;

    double ws2 = Math.sqrt(Math.pow(B, 2) + Math.pow(D, 2));
    double wa2 = Math.atan2(B, D) * 180 / Math.PI;

    double ws3 = Math.sqrt(Math.pow(A, 2) + Math.pow(D, 2));
    double wa3 = Math.atan2(A, D) * 180 / Math.PI;

    double ws4 = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2));
    double wa4 = Math.atan2(A, C) * 180 / Math.PI;

    double max = ws1;

    if (ws2 > max) {
      max = ws2;
    }

    if (ws3 > max) {
      max = ws3;
    }

    if (ws4 > max) {
      max = ws4;
    }

    if (max > 1) {
      ws1 /= max;
      ws2 /= max;
      ws3 /= max;
      ws4 /= max;
    }

    frontRightPower.set(ControlMode.PercentOutput, ws1);
    frontRightAngle.set(ControlMode.PercentOutput, wa1);

    frontLeftPower.set(ControlMode.PercentOutput, ws2);
    frontLeftAngle.set(ControlMode.PercentOutput, wa2);

    backRightPower.set(ControlMode.PercentOutput, ws3);
    backRightAngle.set(ControlMode.PercentOutput, wa3);

    backLeftPower.set(ControlMode.PercentOutput, ws4);
    backLeftAngle.set(ControlMode.PercentOutput, wa4);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
