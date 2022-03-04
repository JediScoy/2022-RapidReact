package frc.robot.commands.Lift;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Lift;

public class LockLiftCommandBar1 extends CommandBase{
 
	// The subsystem the command runs on
	private final Lift subsystem;

 	private double speed;

	// encoder value for when the left lock arm engages on bar #1 - change value if needed
	private double lockHeightLeft = -11000; 
	// encoder value for when the left lock arm engages on bar #1 - change value if needed 
	private double lockHeightRight = -6250;  
	
	//calling climber motors
	private final WPI_TalonFX leftLiftMotor = new WPI_TalonFX(Constants.LEFT_LIFT_MOTOR);
	private final WPI_TalonFX rightLiftMotor = new WPI_TalonFX(Constants.RIGHT_LIFT_MOTOR);

	//command requirements
 	public LockLiftCommandBar1(Lift subsystem, double speed) {
  		this.subsystem = subsystem;
  		this.speed = speed;
   		addRequirements(subsystem);
 	}

 //starts command
 @Override
 public void initialize() {
  	subsystem.startLift(speed);
 }

 //end command conditions
 @Override
 public boolean isFinished() {
	 /**  if the encoder values of the left/right climbing motors are less than or equal to the height of the bar 1, keep
	running motors until it reaches variable values declared above */
	if (leftLiftMotor.getSelectedSensorPosition() >= lockHeightLeft &&
	   rightLiftMotor.getSelectedSensorPosition() >= lockHeightRight) {
		  return false;
	}
	else {
		// stop motors when encoder values surpass variable values
		return true;
	}
}
 
//ends command
 @Override
 public void end(boolean interrupted){ 
			subsystem.stopLift();
	   }


} // End class
