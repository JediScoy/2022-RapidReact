package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Launcher;

// Assumes at least one ball is in the machine 
// 1. Start the launcher
// 2. Delay (time) seconds
// 3. Index the ball into the running launch wheels for a set time
// 4. Intake the second ball slighly higher for a set time
// 5. Index the second ball into the running launch wheels for a set time
// 6. Stop launch motors upon release


public class LaunchSequence extends CommandBase {
    
    // The subsystem the command runs on
    private final Launcher subsystem;
    private double speedBack; 
    private double speedFront;

    public launcherSequence(Launcher subsystem, double speedFront, double speedBack) {
      this.subsystem = subsystem;
      this.speedFront = speedFront;
      this.speedBack = speedBack;
      addRequirements(subsystem); // clever way to call in case you change the name of the subsystem
    }
  
    @Override
    public void initialize() {
      subsystem.setLauncherSpeed(speedFront, speedBack);
    }
  
    @Override
    public void end(boolean interrupted){ 
      subsystem.setLauncherSpeed(0, 0);
    }
    
    
  } // end class