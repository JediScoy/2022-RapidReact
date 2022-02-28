// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;
import frc.robot.commands.IndexSpeed;
import frc.robot.commands.IntakeSpeed;
import frc.robot.commands.LauncherSequence;
import frc.robot.commands.LauncherSpeed;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Launcher;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** FIXME Autonomous place holder for testing purposes
 * Runs the Launcher for second, then runs the launcher again along with index motors with index, launcher, intake, index
*/ 
public class Blue1 extends SequentialCommandGroup {
    // Basically Chad's full launch sequence
    public Blue1(DrivetrainSubsystem drivetrain, Index indexMotors, Intake intakeMotor, Launcher launcher) {
      addCommands(
    new LauncherSequence(launcher, intakeMotor, indexMotors)
      ); // end of addCommands
 
    }  
    
  } // end class