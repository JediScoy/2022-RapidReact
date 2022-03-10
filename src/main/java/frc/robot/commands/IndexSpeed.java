package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Index;

public class IndexSpeed extends CommandBase {
  
    // The subsystem the command runs on
    private final Index subsystem;

    private double speed = 0.5;

    public IndexSpeed(Index subsystem, double speed) {
      this.subsystem = subsystem;
      this.speed = speed;
      addRequirements(subsystem);
    }
  
    @Override
    public void initialize() {
      subsystem.startIndex(speed);
    }
  
    public void execute() {
      subsystem.setIndexSpeed();
    }

    @Override
    public void end(boolean interrupted){ 
      subsystem.stopIndex();
      
    }

  } // end class