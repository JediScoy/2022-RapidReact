package frc.robot.commands.Lift;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Lift;

public class LockLiftCommandBar1 extends CommandBase{
 
	// The subsystem the command runs on
	private final Lift subsystem;

 	private double speed;

	// encoder value for bar 1 height of left arm
	private double lockHeightLeft = -11000; 
	// encoder value for bar 1 height of right arm  
	private double lockHeightRight = -6250;  

	//private int leftLiftMotorEncoder = leftLiftMotor.get
	
	private final WPI_TalonFX leftLiftMotor = new WPI_TalonFX(Constants.LEFT_LIFT_MOTOR);
	private final WPI_TalonFX rightLiftMotor = new WPI_TalonFX(Constants.RIGHT_LIFT_MOTOR);

 	public LockLiftCommandBar1(Lift subsystem, double speed) {
  	this.subsystem = subsystem;
  	this.speed = speed;
   	addRequirements(subsystem);
 	}

 @Override
 public void initialize() {
  	subsystem.startLift(speed);
 }

 @Override
 public boolean isFinished() {
	if (leftLiftMotor.getSelectedSensorPosition() >= lockHeightLeft &&
	   rightLiftMotor.getSelectedSensorPosition() >= lockHeightRight) {
		  return false;
	}
	else {
		return true;
	}
}
 
 @Override
 public void end(boolean interrupted){ 
			subsystem.stopLift();
	   }


} // End class
