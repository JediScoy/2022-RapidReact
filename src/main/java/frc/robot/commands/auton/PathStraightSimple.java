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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.io.IOException;
import java.util.Optional;


public class PathStraightSimple extends CommandBase{
    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

    // 1. Load the path from Path Planner
    Trajectory examplePath = PathPlanner.loadPath("Straight",5,5);

    // 2. Defining PID Controllers for tracking trajectory

    // 3. Command to follow path from PathPlanner

    // 4. Add some init and wrap-up, and return everything
       
}

