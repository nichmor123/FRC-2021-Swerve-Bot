// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;
import frc.robot.modules.Talon;

public class DriveSubsystem extends SubsystemBase {
  private static final ControlMode MOTOR_OUTPUT = ControlMode.PercentOutput;

  /** Creates a new DriveSubsystem. */
  private Joystick m_baseJS;

  public static final int frontLeftDriveId = 3;
  public static final int frontLeftCANCoderId = 23;
  public static final int frontLeftSteerId = 7;
  // put your can Id's here!
  public static final int frontRightDriveId = 8;
  public static final int frontRightCANCoderId = 20;
  public static final int frontRightSteerId = 5;
  // put your can Id's here!
  public static final int backLeftDriveId = 9;
  public static final int backLeftCANCoderId = 21;
  public static final int backLeftSteerId = 2;

  public static final int backRightDriveId = 4;
  public static final int backRightCANCoderId = 20;
  public static final int backRightSteerId = 6;

  private final CANCoder frontRight = new CANCoder(frontRightCANCoderId);
  private final CANCoder frontLeft = new CANCoder(frontLeftCANCoderId);
  private final CANCoder backLeft = new CANCoder(backLeftCANCoderId);
  private final CANCoder backRight = new CANCoder(backRightCANCoderId);

  private final TalonFX frontLeftPower = new TalonFX(frontLeftDriveId);
  private final Talon frontLeftAngle = new Talon(frontLeftSteerId, TalonFXInvertType.Clockwise, NeutralMode.Brake);

  private final TalonFX frontRightPower = new TalonFX(frontRightDriveId);
  private final Talon frontRightAngle = new Talon(frontRightSteerId, TalonFXInvertType.Clockwise, NeutralMode.Brake);

  private final TalonFX backLeftPower = new TalonFX(backLeftDriveId);
  private final Talon backLeftAngle = new Talon(backLeftSteerId, TalonFXInvertType.Clockwise, NeutralMode.Brake);

  private final TalonFX backRightPower = new TalonFX(backRightDriveId);
  private final Talon backRightAngle = new Talon(backRightSteerId, TalonFXInvertType.Clockwise, NeutralMode.Brake);

  private boolean isCalibrated = false;

  public DriveSubsystem(Joystick baseJS) {
    this.m_baseJS = baseJS;
  }

  public void SwerveDrive() {
    double x = m_baseJS.getRawAxis(0);
    double y = m_baseJS.getRawAxis(1);
    double z = m_baseJS.getRawAxis(4);

    // the -x is to invert the x axis
    double FWD = -y;
    double STR = -x;
    double RCW = z;

    double temp = FWD * Math.cos(0) + STR * Math.sin(0);
    STR = -FWD * Math.sin(0) + STR * Math.cos(0);
    FWD = temp;

    double L = RobotContainer.Length;
    double W = RobotContainer.Width;
    double R = Math.sqrt(Math.pow(L, 2) + Math.pow(W, 2));

    double A = STR - RCW * (L / R);
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

    SmartDashboard.putNumber("max", max);

    // normalize the speed
    if (ws2 > max) {
      max = ws2;
    }

    if (ws3 > max) {
      max = ws3;
    }

    if (ws4 > max) {
      max = ws4;
    }

    // BE CAREFULL WHEN CHANGING THIS, KEEP IT AT ONE ALWAYS
    if (max > 1) {
      ws1 /= max;
      ws2 /= max;
      ws3 /= max;
      ws4 /= max;
    }

    // wa1, wa2, -37, -137
    if (wa1 < -20 && wa1 > 39) {
      ws1 = ws1 * -1.0;
      System.out.println("ws1 " + ws1);
    }

    if (wa2 < -120 && wa2 > -139) {
      ws2 = ws2 * -1.0;
      System.out.println("ws2 " + ws2);
    }

    // wa3, wa4, -37, -137
    if (wa3 < -20 && wa3 > -39) {
      ws3 = ws3 * -1.0;
      System.out.println("ws3 " + ws3);
    }

    if (wa4 < -120 && wa2 > 20) {
      ws4 = ws4 * -1.0;
      System.out.println("ws4 " + ws4);
    }

    // if z is greator than 0.15 then rotate
    if (z >= 0.15) {
      wa1 = 60;
      wa2 = 140;

      wa3 = -140;
      wa4 = -60;
    }

    if (z <= -0.15) {
      wa1 = 60;
      wa2 = 140;

      wa3 = -140;
      wa4 = -60;
    }

    frontRightAngle.setPosition(wa1);
    frontLeftAngle.setPosition(wa2);
    backLeftAngle.setPosition(wa3);
    backRightAngle.setPosition(wa4);

    frontRightPower.set(MOTOR_OUTPUT, ws1);
    frontLeftPower.set(MOTOR_OUTPUT, ws2);
    backLeftPower.set(MOTOR_OUTPUT, ws3);
    backRightPower.set(MOTOR_OUTPUT, ws4);

    SmartDashboard.putBoolean("ws1", false);
    SmartDashboard.putBoolean("ws2", false);
    SmartDashboard.putBoolean("ws3", false);
    SmartDashboard.putBoolean("ws4", false);

    SmartDashboard.putNumber("wa1", wa1);
    SmartDashboard.putNumber("wa2", wa2);
    SmartDashboard.putNumber("wa3", wa3);
    SmartDashboard.putNumber("wa4", wa4);

    SmartDashboard.putNumber("ws1", ws1);
    SmartDashboard.putNumber("ws2", ws2);
    SmartDashboard.putNumber("ws3", ws3);
    SmartDashboard.putNumber("ws4", ws4);

    SmartDashboard.putNumber("getPos", frontRight.getPosition());
    SmartDashboard.putNumber("right front cancoder", frontRight.getPosition());
  }

  public void setMode() {
    this.backRightPower.setNeutralMode(NeutralMode.Brake);
    this.backLeftPower.setNeutralMode(NeutralMode.Brake);

    this.frontLeftPower.setNeutralMode(NeutralMode.Brake);
    this.frontRightPower.setNeutralMode(NeutralMode.Brake);
  }

  // public void rotate(double _power) {
  // this.frontLeftAngle.setPosition(-160);
  // this.frontRightAngle.setPosition(160);

  // this.backRightAngle.setPosition(-130);
  // this.backLeftAngle.setPosition(-100);

  // this.frontLeftPower.set(ControlMode.PercentOutput, _power);
  // this.frontRightPower.set(ControlMode.PercentOutput, _power);

  // this.backRightPower.set(ControlMode.PercentOutput, _power);
  // this.backLeftPower.set(ControlMode.PercentOutput, _power);
  // }

  public void ZeroAll() {
    this.backLeftAngle.Zero();
    this.backRightAngle.Zero();
    this.frontLeftAngle.Zero();
    this.frontRightAngle.Zero();

    this.frontLeft.setPosition(0);
    this.frontRight.setPosition(0);
    this.backLeft.setPosition(0);
    this.backRight.setPosition(0);
  }

  public boolean isCalibrated() {
    return isCalibrated;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
