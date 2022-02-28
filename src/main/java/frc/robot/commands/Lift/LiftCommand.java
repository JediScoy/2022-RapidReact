package frc.robot.commands.Lift;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lift;

public class LiftCommand extends CommandBase{
 
	// The subsystem the command runs on
	private final Lift subsystem;

 	private double speed;

 	public LiftCommand(Lift subsystem, double speed) {
  	this.subsystem = subsystem;
  	this.speed = speed;
   	addRequirements(subsystem);
 	}

 @Override
 public void initialize() {
  	subsystem.startLift(speed);
 }

 @Override
 public void end(boolean interrupted){ 
   	subsystem.stopLift();
   
 }

} // End class
