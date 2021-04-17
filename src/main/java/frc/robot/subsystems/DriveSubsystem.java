// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class DriveSubsystem extends SubsystemBase {
  private static final ControlMode MOTOR_OUTPUT = ControlMode.PercentOutput;

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

  public static final int backRightDriveId = 3;
  public static final int backRightCANCoderId = 23;
  public static final int backRightSteerId = 7;

  private final CANCoder frontLeft = new CANCoder(frontLeftCANCoderId);
  private final CANCoder frontRight = new CANCoder(frontRightCANCoderId);
  private final CANCoder backLeft = new CANCoder(backLeftCANCoderId);
  private final CANCoder backRight = new CANCoder(backRightCANCoderId);

  private final TalonFX frontLeftPower = new TalonFX(frontLeftDriveId);
  private final TalonFX frontLeftAngle = new TalonFX(frontLeftSteerId);

  private final TalonFX frontRightPower = new TalonFX(frontRightDriveId);
  private final TalonFX frontRightAngle = new TalonFX(frontRightSteerId);

  private final TalonFX backLeftPower = new TalonFX(frontLeftDriveId);
  private final TalonFX backLeftAngle = new TalonFX(backLeftSteerId);

  private final TalonFX backRightPower = new TalonFX(backRightDriveId);
  private final TalonFX backRightAngle = new TalonFX(backRightSteerId);

  private final double flInit = 0.0;
  private final double frInit = 0.0;
  private final double blInit = 0.0;
  private final double brInit = 0.0;

  private final double range = 10;

  private boolean isCalibrated = false;

  public DriveSubsystem(Joystick baseJS) {
    this.m_baseJS = baseJS;
  }

  public void SwerveDrive() {
    double x = m_baseJS.getRawAxis(0);
    double y = m_baseJS.getRawAxis(1);
    double z = m_baseJS.getRawAxis(2);

    double FWD = -y;
    double STR = x;
    double RCW = z;

    double temp = FWD * Math.cos(0) + STR * Math.sin(0);
    STR = -FWD * Math.sin(0) + STR * Math.cos(0);
    FWD = temp;

    double L = RobotContainer.Length;
    double W = RobotContainer.Width;
    double R = Math.sqrt(Math.pow(L, 2) + Math.pow(W, 2));

    double A = STR - RCW * (L / W);
    double B = STR + RCW * (L / R);
    double C = FWD - RCW * (W / R);
    double D = FWD + RCW * (W / R);

    double ws1 = Math.sqrt(Math.pow(B, 2) + Math.pow(C, 2));
    double wa1 = Math.atan2(B, C) - Math.PI / 4;

    double ws2 = Math.sqrt(Math.pow(B, 2) + Math.pow(D, 2));
    double wa2 = Math.atan2(B, D) - Math.PI / 4;

    double ws3 = Math.sqrt(Math.pow(A, 2) + Math.pow(D, 2));
    double wa3 = Math.atan2(A, D) - Math.PI / 4;

    double ws4 = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2));
    double wa4 = Math.atan2(A, C) - Math.PI / 4;

    double maxs = ws1;
    double maxa = wa1;

    // normalize the speed
    if (ws2 > maxs) {
      maxs = ws2;
    }

    if (ws3 > maxs) {
      maxs = ws3;
    }

    if (ws4 > maxs) {
      maxs = ws4;
    }

    if (maxs > 1) {
      ws1 /= maxs;
      ws2 /= maxs;
      ws3 /= maxs;
      ws4 /= maxs;
    }

    SmartDashboard.putNumber("ws1", ws1);
    SmartDashboard.putNumber("ws2", ws2);
    SmartDashboard.putNumber("ws3", ws3);
    SmartDashboard.putNumber("ws4", ws4);

    frontRightPower.set(MOTOR_OUTPUT, ws1);
    frontRightAngle.set(MOTOR_OUTPUT, wa1);

    frontLeftPower.set(MOTOR_OUTPUT, ws2);
    frontLeftAngle.set(MOTOR_OUTPUT, wa2);

    backLeftPower.set(MOTOR_OUTPUT, ws3);
    backLeftAngle.set(MOTOR_OUTPUT, wa3);

    backRightPower.set(MOTOR_OUTPUT, ws4);
    backRightAngle.set(MOTOR_OUTPUT, wa4);
  }

  public void Calibrate() {
    double fl = frontLeft.getAbsolutePosition();
    double fr = frontRight.getAbsolutePosition();

    double br = backLeft.getAbsolutePosition();
    double bl = backRight.getAbsolutePosition();

    if (fl <= flInit + range && fl >= flInit - range) {
      frontLeftAngle.set(MOTOR_OUTPUT, 0.5);
    }

    if (fr >= frInit + range && fr <= frInit - range) {
      frontRightAngle.set(MOTOR_OUTPUT, 0.5);
    }

    if (br >= brInit + range && br <= brInit - range) {
      backRightAngle.set(MOTOR_OUTPUT, 0.5);
    }

    if (bl >= blInit + range && bl <= blInit - range) {
      backLeftAngle.set(MOTOR_OUTPUT, 0.5);
    }
  }

  public boolean isCalibrated() {
    return isCalibrated;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
