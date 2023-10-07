// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.RobotContainer;


public class driverobot extends CommandBase {
  /** Creates a new driverobot. */
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  public driverobot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_robotDrive);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_robotDrive.drive(
      m_driverController.getLeftY(),//forwards
      m_driverController.getLeftX(),//sideways
      m_driverController.getRightX(),//rotation
      RobotContainer.getFieldOriented(),//field oriented
      RobotContainer.getRateLimit()//limit max speed
      );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
