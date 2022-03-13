// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PathPlannerBased;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Launcher;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// Autonomous place holder for testing purposes
public class CG_DriveV extends SequentialCommandGroup {
  PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("PATHV", 4, 2);

    public CG_DriveV(Drivetrain drivetrain, Index indexMotors, Intake intakeMotor, Launcher launcher) {
      addCommands(
        // Start the Launcher - speedFront is first double, speedBack is second
            new InstantCommand(()
            -> drivetrain.resetOdometry(trajectory1.getInitialPose())),
            drivetrain.createCommandForTrajectory(trajectory1).andThen(() 
            -> drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0)
            ))
        ); // end of add commands


    } // end of Public Blue
  } // end class