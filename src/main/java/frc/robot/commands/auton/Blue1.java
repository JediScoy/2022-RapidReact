// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;
import frc.robot.commands.IndexSpeed;
import frc.robot.commands.IntakeSpeed;
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
  
    public Blue1(DrivetrainSubsystem drivetrain, Index indexMotors, Intake intakeMotor, Launcher launcher) {
      addCommands(
        new LauncherSpeed(launcher, 0.40, 0.45).withTimeout(1),
        new SequentialCommandGroup(
          new LauncherSpeed(launcher, 0.40, 0.45).withTimeout(0.5).alongWith(
            new IndexSpeed(indexMotors, 0.5).withTimeout(0.5)),
              new ParallelCommandGroup (
                new LauncherSpeed(launcher, 0.35, 0.40),
                new IntakeSpeed(intakeMotor, -0.5),
                new IndexSpeed(indexMotors, 0.5)
              ) // end of ParallelCommandGroup
        ) // end of SequentialCommandGroup 
      ); // end of addCommands
 
    }  
    
  } // end class