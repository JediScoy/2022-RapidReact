package frc.robot.commands.Lift;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Lift;

public class LockLiftCommandBar2 extends CommandBase{
 
	// The subsystem the command runs on
	private final Lift subsystem;

 	private double speed;

	// encoder value for bar 2 height of left arm
	private double lockHeightLeft2 = -1000; //FIXME change to encoder value when lock arm engages on bar 2
	// encoder value for bar 2 height of right arm  
	private double lockHeightRight2 = -1000;  //FIXME change to encoder value when lock arm engages on bar 2

	//private int leftLiftMotorEncoder = leftLiftMotor.get
	
	private final WPI_TalonFX leftLiftMotor = new WPI_TalonFX(Constants.LEFT_LIFT_MOTOR);
	private final WPI_TalonFX rightLiftMotor = new WPI_TalonFX(Constants.RIGHT_LIFT_MOTOR);

 	public LockLiftCommandBar2(Lift subsystem, double speed) {
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
	if (leftLiftMotor.getSelectedSensorPosition() >= lockHeightLeft2 &&
	   rightLiftMotor.getSelectedSensorPosition() >= lockHeightRight2) {
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
