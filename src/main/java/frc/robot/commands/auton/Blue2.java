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

/**  FIXME Autonomous place holder for testing purposes
 * Currently runs the Index and Intake at the same time
*/

public class Blue2 extends SequentialCommandGroup {

  public Blue2(DrivetrainSubsystem drivetrain, Index indexMotors, Intake intakeMotor, Launcher launcher) {
    addCommands(
        new LauncherSpeed(launcher, 0.35, 0.40).withTimeout(0.75), // changed from 1 to 0.75
          new SequentialCommandGroup(
            new LauncherSpeed(launcher, 0.35, 0.40).withTimeout(0.25).alongWith( //changed from 0.5 to 0.25
              new IndexSpeed(indexMotors, 0.5).withTimeout(0.25)), //changed from 0.5 to 0.25
                new ParallelCommandGroup (
                  new LauncherSpeed(launcher, 0.36, 0.42), // added 0.02
                  new IntakeSpeed(intakeMotor, 0.5),
                  new IndexSpeed(indexMotors, 0.5)
                  )));

  }    
    
} // end class

