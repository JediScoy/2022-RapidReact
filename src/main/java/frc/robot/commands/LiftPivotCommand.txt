package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftPivot;


public class LiftPivotCommand extends CommandBase{
    
     
        // The subsystem the command runs on
        private final LiftPivot subsystem;
    
         private double speed;
    
         public LiftPivotCommand(LiftPivot subsystem, double speed) {
          this.subsystem = subsystem;
          this.speed = speed;
           addRequirements(subsystem);
         }
    
     @Override
     public void initialize() {
          subsystem.startLiftPivot(speed);
     }
    
     @Override
     public void end(boolean interrupted){ 
           subsystem.stopLiftPivot();
       
     }
    
        
} // End class