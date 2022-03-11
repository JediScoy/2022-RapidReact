// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import frc.robot.commands.IndexSpeed;
import frc.robot.commands.IntakeSpeed;
import frc.robot.commands.LauncherSpeed;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Launcher;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;


// Autonomous place holder for testing purposes
public class CGLaunch2_DriveStraight extends SequentialCommandGroup {
  PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("Straight", 3, 1);

    public CGLaunch2_DriveStraight(Drivetrain drivetrain, Index indexMotors, Intake intakeMotor, Launcher launcher) {
      addCommands(
        // Start the Launcher - speedFront is first double, speedBack is second
        new LauncherSpeed(launcher, 0.40, 0.45).withTimeout(1),
        new SequentialCommandGroup(
            // Maintain Launcher speed
            new LauncherSpeed(launcher, 0.40, 0.45).withTimeout(2).alongWith(
              // Index the ball #1 into the running Launcher
              new IndexSpeed(indexMotors, 0.50).withTimeout(0.5)),
                // Robot drives in a straight path to the next ball
                new ParallelCommandGroup(
                  // Maintain Launcher speed
                  new LauncherSpeed(launcher, 0.35, 0.40),
                  // Intake ball #2 if needed
                  new IntakeSpeed(intakeMotor, 0.50),
                  // Index ball #2 into already running Launcher
                  new IndexSpeed(indexMotors, 0.50),
            new InstantCommand(()
            -> drivetrain.resetOdometry(trajectory1.getInitialPose())),
            drivetrain.createCommandForTrajectory(trajectory1).andThen(() 
            -> drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0)
            ))))
        ); // end of add commands


    } // end of Public Blue
  } // end class