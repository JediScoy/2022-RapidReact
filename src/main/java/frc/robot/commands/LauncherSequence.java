package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

public class LauncherSequence extends ParallelCommandGroup {
    
    public LauncherSequence(Launcher launcher, Intake intakeMotor, Index indexMotors) {
      new LauncherSpeed(launcher, 0.20, 0.50).withTimeout(5).andThen(new LauncherSpeed(launcher, 0.25, 0.25),
      new IntakeSpeed(intakeMotor, 0.5),
      new IndexSpeed(indexMotors, 0.5)
      );
    }
    
  } // end class