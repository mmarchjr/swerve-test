// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.nio.file.Path;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.driverobot;
import frc.robot.subsystems.DriveSubsystem;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  SendableChooser<String> autoChooser = new SendableChooser<String>();
  SendableChooser<Boolean> fieldoriented = new SendableChooser<Boolean>();
  SendableChooser<Boolean> ratelimitChooser = new SendableChooser<Boolean>();
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Command DriveRobot = new driverobot();
DriveRobot.schedule();
    // Configure the button bindings
fieldoriented.setDefaultOption("field oriented", true);
fieldoriented.addOption("robot oriented", false);
SmartDashboard.putData("drive mode selector",fieldoriented);

ratelimitChooser.setDefaultOption("false", false);
ratelimitChooser.addOption("true", true);
SmartDashboard.putData("rate limit?", ratelimitChooser);

    configureButtonBindings();
File deploy = Filesystem.getDeployDirectory();
File pathfolder = new File(Path.of(deploy.getAbsolutePath(),"pathplanner").toString());
File[] listOfFiles = pathfolder.listFiles();

for (int i = 0; i < listOfFiles.length; i++) {
  if (listOfFiles[i].isFile()) {
    System.out.println("path:" + listOfFiles[i].getName());
    autoChooser.addOption(listOfFiles[i].getName().replace(".path", ""), listOfFiles[i].getName().replace(".path", ""));
  }
}
SmartDashboard.putData("Autonomous",autoChooser);
//String pathplannerlocation = ;

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    m_driverController.getLeftY(),//forwards
                    m_driverController.getLeftX(),//sideways
                    m_driverController.getRightX(),//rotation
                    fieldoriented.getSelected(),//field oriented
                    ratelimitChooser.getSelected()//limit max speed
                    )));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
   

    // An example trajectory to follow.  All units in meters.
    PathPlannerTrajectory exampleTrajectory = PathPlanner.loadPath(autoChooser.getSelected(),  new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    ProfiledPIDController thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, AutoConstants.kIXController, AutoConstants.kDXController),
            new PIDController(AutoConstants.kPYController, AutoConstants.kIYController, AutoConstants.kDYController),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
    Logger.getInstance().recordOutput("auto/Trajectory", exampleTrajectory);

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.setX());
  }
}
