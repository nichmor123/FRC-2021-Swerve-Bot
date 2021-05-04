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
import frc.robot.Gains;

/** Pink Team TalonFX FRC Wrapper */
public class Talon extends TalonFX {
  private TalonFXConfiguration config = new TalonFXConfiguration();
  private int id;

  // remove if not needed
  private TalonFXInvertType invert = TalonFXInvertType.Clockwise;
  private NeutralMode currentNeutralMode = NeutralMode.Brake;

  private PositionType currentPositionType = PositionType.HALF;

  private enum PositionType {
    HALF, FULL
  }

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

  public void setPosition(double _position, PositionType _type) {
    double unit = 180;

    SetPositionType(_type);

    if (this.currentPositionType == PositionType.FULL) {
      unit = 360;
    }

    // fix the unit conversion issue
    double targetPositionRotations = _position * (26220 / unit); // the base value is 26120
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

  public void SetPositionType(PositionType _type) {
    this.currentPositionType = _type;
  }

  public PositionType GetPositionType() {
    return this.currentPositionType;
  }

  public void Zero() {
    this.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
    System.out.println("Encoders Zeroed for motor " + this.id);
  }
}

class Constants {
  /**
   * Which PID slot to pull gains from. Starting 2018, you can choose from 0,1,2
   * or 3. Only the first two (0,1) are visible in web-based configuration.
   */
  public static final int kSlotIdx = 0;

  /**
   * Talon FX supports multiple (cascaded) PID loops. For now we just want the
   * primary one.
   */
  public static final int kPIDLoopIdx = 0;

  /* Choose so that Talon does not report sensor out of phase */
  public static boolean kSensorPhase = true;

  /**
   * Choose based on what direction you want to be positive, this does not affect
   * motor invert.
   */
  public static boolean kMotorInvert = false;

  /**
   * Number of joystick buttons to poll. 10 means buttons[1,9] are polled, which
   * is actually 9 buttons.
   */
  public final static int kNumButtonsPlusOne = 10;

  /**
   * How many sensor units per rotation. Using Talon FX Integrated Sensor.
   * 
   * @link https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
   */
  public final static int kSensorUnitsPerRotation = 2048;

  /**
   * Number of rotations to drive when performing Distance Closed Loop
   */
  public final static double kRotationsToTravel = 6;

  /**
   * Set to zero to skip waiting for confirmation. Set to nonzero to wait and
   * report to DS if action fails.
   */
  public final static int kTimeoutMs = 30;

  /**
   * Motor neutral dead-band, set to the minimum 0.1%.
   */
  public final static double kNeutralDeadband = 0.001;

  /**
   * Empirically measure what the difference between encoders per 360' Drive the
   * robot in clockwise rotations and measure the units per rotation. Drive the
   * robot in counter clockwise rotations and measure the units per rotation. Take
   * the average of the two.
   */
  public final static int kEncoderUnitsPerRotation = 2048;

  /**
   * PID Gains may have to be adjusted based on the responsiveness of control
   * loop. kF: 1023 represents output value to Talon at 100%, 6800 represents
   * Velocity units at 100% output Not all set of Gains are used in this project
   * and may be removed as desired.
   * 
   * kP kI kD kF Iz PeakOut
   */
  public final static Gains kGains_Distanc = new Gains(0.1, 0.0, 0.0, 0.0, 100, 0.50);
  public final static Gains kGains_Turning = new Gains(2.0, 0.0, 4.0, 0.0, 200, 1.00);
  public final static Gains kGains_Velocit = new Gains(0.1, 0.0, 20.0, 1023.0 / 6800.0, 300, 0.50);
  public final static Gains kGains_MotProf = new Gains(1.0, 0.0, 0.0, 1023.0 / 6800.0, 400, 1.00);

  /**
   * Gains used in Positon Closed Loop, to be adjusted accordingly Gains(kp, ki,
   * kd, kf, izone, peak output);
   */
  public static final Gains kGains = new Gains(0.15, 0.0, 1.0, 0.0, 0, 1.0);

  /** ---- Flat constants, you should not need to change these ---- */
  /*
   * We allow either a 0 or 1 when selecting an ordinal for remote devices [You
   * can have up to 2 devices assigned remotely to a talon/victor]
   */
  public final static int REMOTE_0 = 0;
  public final static int REMOTE_1 = 1;
  /*
   * We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1
   * is auxiliary
   */
  public final static int PID_PRIMARY = 0;
  public final static int PID_TURN = 1;
  /*
   * Firmware currently supports slots [0, 3] and can be used for either PID Set
   */
  public final static int SLOT_0 = 0;
  public final static int SLOT_1 = 1;
  public final static int SLOT_2 = 2;
  public final static int SLOT_3 = 3;
  /* ---- Named slots, used to clarify code ---- */
  public final static int kSlot_Distanc = SLOT_0;
  public final static int kSlot_Turning = SLOT_1;
  public final static int kSlot_Velocit = SLOT_2;
  public final static int kSlot_MotProf = SLOT_3;
}