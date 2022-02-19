package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;

// Assumes at least one ball is in the machine 
// 1. Start the launcher
// 2. Delay (time) seconds
// 3. Index the ball into the running launch wheels for a set time
// 4. Intake the second ball slighly higher for a set time
// 5. Index the second ball into the running launch wheels for a set time
// 6. Stop launch motors upon release


public class LaunchSequence extends SequentialCommandGroup {
  private double m_timeout;

  public void launchSequence(Index indexMotors, double timeout, Intake intakeMotor) {
      m_timeout=timeout;
      addCommands(
        // new LauncherSpeed(launcher, 0.20, 0.20),
        new SequentialCommandGroup(
          new IndexCommand(indexMotors, 0.5),
          new IntakeCommand(intakeMotor, 0.5),
          new IndexCommand(indexMotors, 0.5)
      )
    ); // new SequentialCommandGroup(new startIndex(), new setLauncherSpeed());
  }


    @Override
    public void initialize() {
    }
  
    @Override
    public void end(boolean interrupted){ 
     // subsystem.setLauncherSpeed(0, 0);
    }
    
    
  } // end class