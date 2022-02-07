// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ExampleAuton extends CommandBase {
  private final DrivetrainSubsystem drivetrain;

  
  /** Creates a new ExampleAuton. */
  public ExampleAuton(DrivetrainSubsystem subsystem) {
    this.drivetrain = subsystem;

 // The subsystem the command runs on
    addRequirements(subsystem);
 
    
    // This will load the file "Square.path" and generate it with a max velocity of 8 m/s and a max acceleration of 5 m/s^2
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("Square", 8, 5);

    // Sample the state of the path at 1 second
    // To access PathPlanner specific information, such as holonomic rotation, the state must be cast to a PathPlannerState
    PathPlannerState exampleState = (PathPlannerState) examplePath.sample(1);

}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
