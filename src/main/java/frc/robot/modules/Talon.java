// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules;

import javax.annotation.Nullable;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import frc.robot.Constants;

/** Add your docs here. */
public class Talon extends TalonFX {
  private TalonFXConfiguration config = new TalonFXConfiguration();

  // remove if not needed
  private TalonFXInvertType invert = TalonFXInvertType.Clockwise;
  private NeutralMode currentNeutralMode = NeutralMode.Brake;

  /**
   * for driving here is the position snippet
   * https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/a9063fd8ca1a18b3730783ac5548541992b97f39/Java%20Talon%20FX%20(Falcon%20500)/PositionClosedLoop_AuxFeedForward/src/main/java/frc/robot/Robot.java#L225
   * 
   * @param id
   * @param _invert
   * @param _neutralMode
   */
  public Talon(int id, TalonFXInvertType _invert, NeutralMode _neutralMode) {
    super(id);

    this.setNeutralMode(_neutralMode);

    this.invert = _invert;
    this.currentNeutralMode = _neutralMode;

    this.config.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

    /*
     * Configure the Remote Talon's selected sensor as a remote sensor for the right
     * Talon
     * 
     * ALSO: this might need to be removed if its not useful
     */
    this.config.remoteFilter0.remoteSensorDeviceID = this.getDeviceID();
    this.config.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor;

    /**
     * Max out the peak output (for all modes). However you can limit the output of
     * a given PID object with configClosedLoopPeakOutput().
     */
    this.config.peakOutputForward = +1.0;
    this.config.peakOutputReverse = -1.0;

    /* FPID Gains for distance servo */
    this.config.slot0.kP = Constants.kGains_Distanc.kP;
    this.config.slot0.kI = Constants.kGains_Distanc.kI;
    this.config.slot0.kD = Constants.kGains_Distanc.kD;
    this.config.slot0.kF = Constants.kGains_Distanc.kF;
    this.config.slot0.integralZone = Constants.kGains_Distanc.kIzone;
    this.config.slot0.closedLoopPeakOutput = Constants.kGains_Distanc.kPeakOutput;
    this.config.slot0.allowableClosedloopError = 0;

    /**
     * 1ms per loop. PID loop can be slowed down if need be. For example, - if
     * sensor updates are too slow - sensor deltas are very small per update, so
     * derivative error never gets large enough to be useful. - sensor movement is
     * very slow causing the derivative error to be near zero.
     */
    int closedLoopTimeMs = 1;
    this.config.slot0.closedLoopPeriod = closedLoopTimeMs;
    this.config.slot1.closedLoopPeriod = closedLoopTimeMs;
    this.config.slot2.closedLoopPeriod = closedLoopTimeMs;
    this.config.slot3.closedLoopPeriod = closedLoopTimeMs;

    this.configAllSettings(config);

    /* Configure output and sensor direction */
    this.setInverted(invert);

    /*
     * Talon FX does not need sensor phase set for its integrated sensor This is
     * because it will always be correct if the selected feedback device is
     * integrated sensor (default value) and the user calls getSelectedSensor* to
     * get the sensor's position/velocity.
     * 
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
     * sensor-phase
     */
    // _leftMaster.setSensorPhase(true);
    // _rightMaster.setSensorPhase(true);

    /* Set status frame periods to ensure we don't have stale data */
    this.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
    this.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
    this.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Constants.kTimeoutMs);
    this.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, Constants.kTimeoutMs);
  }

  public void setPosition(double position) {
    double target_sensorUnits = position * Constants.kSensorUnitsPerRotation * Constants.kRotationsToTravel;
    double forwd = position * 0.10;

    this.set(TalonFXControlMode.Position, target_sensorUnits, DemandType.ArbitraryFeedForward, forwd);
  }

  public void Reconfigure(TalonFXConfiguration _config) {
    this.config = _config;

    this.configAllSettings(_config);
  }
}
