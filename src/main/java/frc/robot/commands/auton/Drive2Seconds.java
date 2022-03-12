// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Drive2Seconds extends SequentialCommandGroup {
  /** Creates a new Drive2Seconds. */
  public Drive2Seconds(Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // Wait command will stop the paralleldeadlinegroup, other conditions could subsituted for time to make the group stop
    addCommands(new ParallelDeadlineGroup(new WaitCommand(8), new DriveCommand(drivetrain, () -> {return 0;}, () -> {return 0.7;}, () -> {return 0.0;})));
  }
}
