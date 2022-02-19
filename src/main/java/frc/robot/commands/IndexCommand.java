package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Index;

public class IndexCommand extends CommandBase {
  
    // The subsystem the command runs on
    private final Index subsystem;

    public static double indexSpeed = 0.5;

    public IndexCommand(Index subsystem, double indexSpeed) {
      this.subsystem = subsystem;
      // this.speed = speed;
      addRequirements(subsystem);
    }
  
    @Override
    public void initialize() {
      subsystem.startIndex(indexSpeed);
    }
  
    @Override
    public void end(boolean interrupted){ 
      subsystem.stopIndex();
      
    }

  } // end class