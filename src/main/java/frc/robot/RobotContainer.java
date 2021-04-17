// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.NewSwerve;
import frc.robot.commands.SwerveDriveCommand;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static final int Length = 27;
  public static final int Width = 32;
  public static final int m_IntakeID = 9;

  private final Joystick controller = new Joystick(0);

  // private final DriveSubsystem m_driveSubsystem = new
  // DriveSubsystem(controller);

  private final SwerveDrivetrain drivetrain = new SwerveDrivetrain();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, controller));
    // m_driveSubsystem.setDefaultCommand(new NewSwerve(m_driveSubsystem));
  }
}
