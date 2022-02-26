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

public class Blue2 extends ParallelCommandGroup {
  
    public Blue2(DrivetrainSubsystem drivetrain, Index indexMotors, Intake intakeMotor, Launcher launcher) {
      addCommands(
          new LauncherSpeed(launcher, 0.4, 0.6).withTimeout(5).andThen(new LauncherSpeed(launcher, 0.4, 0.6),
          new IntakeSpeed(intakeMotor, 0.5),
          new IndexSpeed(indexMotors, 0.5)
          )); // end of add commands
 
    }  
    
  } // end class