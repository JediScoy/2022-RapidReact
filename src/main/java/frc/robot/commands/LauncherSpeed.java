package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Launcher;

public class LauncherSpeed extends CommandBase {
    
    // The subsystem the command runs on
    private final Launcher subsystem;
    private double speedBottom; // Does this need to be seperated into Top and Bottom?
    private double speedTop;

    public LauncherSpeed(Launcher subsystem, double speedTop, double speedBottom) {
      this.subsystem = subsystem;
      this.speedTop = speedTop;
      this.speedBottom = speedBottom;
      addRequirements(subsystem); // clever way to call in case you change the name of the subsystem
    }
  
    @Override
    public void initialize() {
      subsystem.setLauncherSpeed(speedTop, speedBottom);
    }
  
    @Override
    public void end(boolean interrupted){ 
      subsystem.setLauncherSpeed(0, 0);
    }
    
    
  } // end class