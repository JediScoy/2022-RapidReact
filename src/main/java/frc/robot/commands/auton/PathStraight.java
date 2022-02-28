// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;



public class PathStraight extends SequentialCommandGroup{
    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  public Command pathStraight() {
    // Load the path
    Trajectory m_path = PathPlanner.loadPath("Straight", 5, 5);

    // 2. Defining PID Controllers for tracking trajectory
    PIDController xController = new PIDController(Constants.kPXController, 0, 0);
    PIDController yController = new PIDController(Constants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(Constants.kPThetaController, 0, 0, Constants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 3. Command to follow path from PathPlanner
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      m_path, 
      m_drivetrainSubsystem::getPose, 
      Constants.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::setModuleStates, 
      m_drivetrainSubsystem);

    // 4. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
      new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(m_path.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() -> m_drivetrainSubsystem.stop()));
  }
} // end of class