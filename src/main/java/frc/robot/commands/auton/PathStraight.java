// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** FRC 3603
 * Experimental 
 * */

package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.*;

public class PathStraight extends PPSwerveControllerCommand {
  /** Creates a new GenericAutonomousCommand. */
  private final Drivetrain drivetrain;

  public PathStraight(Drivetrain drivetrain, PathPlannerTrajectory path) {
    // Use addRequirements() here to declare subsystem dependencies.

    super(
      path,
      drivetrain::getPose, // Functional interface to feed supplier
      drivetrain.m_kinematics,

      // Position controllers
      new PIDController(kPXController, 0, 0),
      new PIDController(kPYController, 0, 0),
      drivetrain.thetaController,
      drivetrain::setModuleStates,
      drivetrain
    );
    
    drivetrain.thetaController.enableContinuousInput(-Math.PI, Math.PI);

    this.drivetrain = drivetrain;
    drivetrain.resetOdometry(path.getInitialPose());

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
