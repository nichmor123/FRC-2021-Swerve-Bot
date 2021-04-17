// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

/* -------------------------------------------------------------------------- */
/*                      Motors Are Commented Out For Now                      */
/* -------------------------------------------------------------------------- */

public class Intake extends SubsystemBase {
  // private final TalonFX m_intake = new TalonFX(RobotContainer.m_IntakeID);
  private final double ActivatedSpeed = 0.5;

  /** Creates a new Intake. */
  public Intake(BooleanSupplier activate) {
    boolean startup = activate.getAsBoolean();

    if (startup) {
      // m_intake.set(ControlMode.PercentOutput, this.ActivatedSpeed);
    } else {
      // m_intake.set(ControlMode.PercentOutput, 0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
