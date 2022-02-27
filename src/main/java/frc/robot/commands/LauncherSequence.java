package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

/** TODO These values and sequence may not reflect the most recent
 * Hoping to replace the multi-command sequence in robot container with this 
 * It will allow easier recycing of launcer code for auton
 * */

public class LauncherSequence extends SequentialCommandGroup {
    
    public LauncherSequence(Launcher launcher, Intake intakeMotor, Index indexMotors) {
      new SequentialCommandGroup(
        new LauncherSpeed(launcher, 0.40, 0.45).withTimeout(1),
          new SequentialCommandGroup(
            new LauncherSpeed(launcher, 0.40, 0.45).withTimeout(0.5).alongWith(
              new IndexSpeed(indexMotors, 0.5).withTimeout(0.5)),
                new ParallelCommandGroup (
                  new LauncherSpeed(launcher, 0.35, 0.40),
                  new IntakeSpeed(intakeMotor, -0.5),
                  new IndexSpeed(indexMotors, 0.5)
      )));
    }
    
  } // end class