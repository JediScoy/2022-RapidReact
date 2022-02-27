// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import frc.robot.commands.auton.PathStraight;
import frc.robot.commands.IndexSpeed;
import frc.robot.commands.IntakeSpeed;
import frc.robot.commands.LauncherSequence;
import frc.robot.commands.LauncherSpeed;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Launcher;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.io.IOException;
import java.util.Optional;

// FIXME Autonomous place holder for testing purposes
public class Blue3 extends SequentialCommandGroup {
  
    public Blue3(DrivetrainSubsystem drivetrain, Index indexMotors, Intake intakeMotor, Launcher launcher) {
      addCommands(
        // Launch the first ball - speedFront is first double, speedBack is second
        new LauncherSpeed(launcher, 0.5, 0.5).withTimeout(3),
        // Index the ball into the launcher
        new IndexSpeed(indexMotors, 0.5).withTimeout(2),
        // Robot drives in a straight path to the next ball
        new PathStraight().withTimeout(2),
        // Intake the second ball
        new IntakeSpeed(intakeMotor, 0.5).withTimeout(2),
        // Start the launcher wheels
        new LauncherSpeed(launcher, 0.3, 0.5).withTimeout(2),
        // Index the ball into the launcher
        new IndexSpeed(indexMotors, 0.5).withTimeout(2) 
      ); // end of add commands
 
    }  
    
  } // end class