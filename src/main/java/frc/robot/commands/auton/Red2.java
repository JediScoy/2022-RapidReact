// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;
import frc.robot.commands.LauncherSequence2;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Launcher;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// FIXME Autonomous place holder for testing purposes
public class Red2 extends SequentialCommandGroup {

    public Red2(DrivetrainSubsystem drivetrain, Index indexMotors, Intake intakeMotor, Launcher launcher) {
      addCommands(
        new LauncherSequence2(launcher, intakeMotor, indexMotors)
      ); // End of commands
      
    }  
    
  } // end class