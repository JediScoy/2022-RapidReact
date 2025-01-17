// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PathPlannerBased;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Launcher;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CG_1BallDriveStraight extends SequentialCommandGroup {
  /** Creates a new S1_2BallCommandGroup. */
  public CG_1BallDriveStraight(Drivetrain drivetrain, Launcher launch) {
    PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("1Ball", 4, 2);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new CGLauncherBall1(launch, null, null),
      // new CGLauncherBall1(Launcher, Intake, Index),
      // new LaunchHigh(launch).withTimeout(0.75),
      new InstantCommand(()
        -> drivetrain.resetOdometry(trajectory1.getInitialPose())),
        drivetrain.createCommandForTrajectory(trajectory1).andThen(() 
        -> drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0)))
    );
  }
}