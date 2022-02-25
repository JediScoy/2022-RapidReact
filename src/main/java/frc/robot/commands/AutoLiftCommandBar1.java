package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Lift;

public class AutoLiftCommandBar1 extends CommandBase{
 
	// The subsystem the command runs on
	private final Lift subsystem;

 	private double speed;

	private int bar1HeightLeft = 0;   //FIXME add encoder value for bar 1 height of left arm
	private int bar1HeightRight = 0;  //FIXME add encoder value for bar 1 height of right arm
	
	private final WPI_TalonFX leftLiftMotor = new WPI_TalonFX(Constants.LEFT_LIFT_MOTOR);
	private final WPI_TalonFX rightLiftMotor = new WPI_TalonFX(Constants.RIGHT_LIFT_MOTOR);

 	public AutoLiftCommandBar1(Lift subsystem, double speed) {
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
   	if (leftLiftMotor.getSelectedSensorPosition() >= bar1HeightLeft && 
	   rightLiftMotor.getSelectedSensorPosition() >= bar1HeightRight) {
		   return;
	   }
 }

} // End class
