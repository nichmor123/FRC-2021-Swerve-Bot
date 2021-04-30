// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Pink Team TalonFX FRC Wrapper */
public class Talon extends TalonFX {
  private TalonFXConfiguration config = new TalonFXConfiguration();
  private int id;

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
  public Talon(int _id, TalonFXInvertType _invert, NeutralMode _neutralMode) {
    super(_id);

    this.configFactoryDefault();

    /* Config the sensor used for Primary PID and sensor direction */
    this.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
        Constants.kTimeoutMs);

    /* Ensure sensor is positive when output is positive */
    this.setSensorPhase(Constants.kSensorPhase);

    /**
     * Set based on what direction you want forward/positive to be. This does not
     * affect sensor phase.
     */
    this.setInverted(Constants.kMotorInvert);
    /*
     * Talon FX does not need sensor phase set for its integrated sensor This is
     * because it will always be correct if the selected feedback device is
     * integrated sensor (default value) and the user calls getSelectedSensor* to
     * get the sensor's position/velocity.
     * 
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
     * sensor-phase
     */
    // this.setSensorPhase(true);

    /* Config the peak and nominal outputs, 12V means full */
    this.configNominalOutputForward(0, Constants.kTimeoutMs);
    this.configNominalOutputReverse(0, Constants.kTimeoutMs);
    this.configPeakOutputForward(1, Constants.kTimeoutMs);
    this.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be neutral
     * within this range. See Table in Section 17.2.1 for native units per rotation.
     */
    this.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    this.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
    this.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
    this.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
    this.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);

    /* Set status frame periods to ensure we don't have stale data */
    this.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
    this.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
    this.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Constants.kTimeoutMs);
    this.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, Constants.kTimeoutMs);
  }

  public void setPosition(double position) {
    // double target_sensorUnits = position * Constants.kSensorUnitsPerRotation *
    // Constants.kRotationsToTravel;
    // double forwd = position * 0.10;

    // fix the unit conversion issue
    double targetPositionRotations = position * ((10 * 2612) / 180);
    this.set(TalonFXControlMode.Position, targetPositionRotations);

    SmartDashboard.putNumber("targetPositionRotations", targetPositionRotations);
  }

  /**
   * Determines if SensorSum or SensorDiff should be used for combining left/right
   * sensors into Robot Distance.
   * 
   * Assumes Aux Position is set as Remote Sensor 0.
   * 
   * configAllSettings must still be called on the master config after this
   * function modifies the config values.
   * 
   * @param masterInvertType Invert of the Master Talon
   * @param masterConfig     Configuration object to fill
   */
  void setRobotDistanceConfigs(TalonFXInvertType masterInvertType, TalonFXConfiguration masterConfig) {
    /**
     * Determine if we need a Sum or Difference.
     * 
     * The auxiliary Talon FX will always be positive in the forward direction
     * because it's a selected sensor over the CAN bus.
     * 
     * The master's native integrated sensor may not always be positive when forward
     * because sensor phase is only applied to *Selected Sensors*, not native sensor
     * sources. And we need the native to be combined with the aux (other side's)
     * distance into a single robot distance.
     */

    /*
     * THIS FUNCTION should not need to be modified. This setup will work regardless
     * of whether the master is on the Right or Left side since it only deals with
     * distance magnitude.
     */

    /* Check if we're inverted */
    if (masterInvertType == TalonFXInvertType.Clockwise) {
      /*
       * If master is inverted, that means the integrated sensor will be negative in
       * the forward direction. If master is inverted, the final sum/diff result will
       * also be inverted. This is how Talon FX corrects the sensor phase when
       * inverting the motor direction. This inversion applies to the *Selected
       * Sensor*, not the native value. Will a sensor sum or difference give us a
       * positive total magnitude? Remember the Master is one side of your drivetrain
       * distance and Auxiliary is the other side's distance. Phase | Term 0 | Term 1
       * | Result Sum: -1 *((-)Master + (+)Aux )| NOT OK, will cancel each other out
       * Diff: -1 *((-)Master - (+)Aux )| OK - This is what we want, magnitude will be
       * correct and positive. Diff: -1 *((+)Aux - (-)Master)| NOT OK, magnitude will
       * be correct but negative
       */

      this.config.diff0Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); // Local Integrated Sensor
      this.config.diff1Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice(); // Aux Selected Sensor
      this.config.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorDifference.toFeedbackDevice(); // Diff0
                                                                                                                 // -
                                                                                                                 // Diff1
    } else {
      /* Master is not inverted, both sides are positive so we can sum them. */
      this.config.sum0Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice(); // Aux Selected Sensor
      this.config.sum1Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); // Local IntegratedSensor
      this.config.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorSum.toFeedbackDevice(); // Sum0 +
                                                                                                          // Sum1
    }

    /*
     * Since the Distance is the sum of the two sides, divide by 2 so the total
     * isn't double the real-world value
     */
    this.config.primaryPID.selectedFeedbackCoefficient = 0.5;
  }

  public void Reconfigure(TalonFXConfiguration _config) {
    this.config = _config;

    this.configAllSettings(_config);
  }

  public void Zero() {
    this.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
    System.out.println("Encoders Zeroed for motor " + this.id);
  }
}
