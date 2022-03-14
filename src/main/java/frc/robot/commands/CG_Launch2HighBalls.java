package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

/** These values and sequence may not reflect the most recent
 * Hoping to replace the multi-command sequence in robot container with this 
 * It will allow easier recycing of launcer code for auton
 * */

public class CGLauncherBall2 extends SequentialCommandGroup {
    
    public CGLauncherBall2(Launcher launcher, Intake intakeMotor, Index indexMotors) {
     addCommands(
      new LaunchHigh(launcher),
      new IndexSpeed(indexMotors, 0.5),
      new IntakeSpeed(intakeMotor, 0.5),
      new LaunchHigh(launcher),
      new IndexSpeed(indexMotors, 0.5)
      ); // end of add commands
    }
    
  } // end class