
package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;

// Shuffleboard imports
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard; 
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

// CTRE imports
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;

public class Lift extends SubsystemBase {
  private final WPI_TalonFX leftLiftMotor = new WPI_TalonFX(Constants.LEFT_LIFT_MOTOR);
  private final WPI_TalonFX rightLiftMotor = new WPI_TalonFX(Constants.RIGHT_LIFT_MOTOR);
  
  // NOTE you must use WPI_TalonFX subclass rather than TalonFX when grouping motors... 60 minutes of my life wasted
  //private MotorControllerGroup liftMotors = new MotorControllerGroup(
    //new WPI_TalonFX(Constants.LEFT_LIFT_MOTOR), 
    //new WPI_TalonFX(Constants.RIGHT_LIFT_MOTOR));

public Lift() {
  
  //leftLiftMotor settings
  leftLiftMotor.setInverted(true);
  leftLiftMotor.setNeutralMode(NeutralMode.Brake);
  //rightLiftMotor settings
  rightLiftMotor.setInverted(false);
  rightLiftMotor.setNeutralMode(NeutralMode.Brake);

  //calling reset encoders function
   resetEncoders();

}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Dashboard stuff would go here

   
    //adding encoder values from lift motors to the dashboard to find out climbing heights
    SmartDashboard.putNumber("Left Lift Arm", leftLiftMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Lift Arm", rightLiftMotor.getSelectedSensorPosition());
  }

  // Sets the Lift system speed as a percentage between -1 and 1. Speed is given actual value in the RobotContrainer.java button code
  // function to lift both climbing arms at once
  public void startLift(double speed){
    //liftMotors.set(speed);
    leftLiftMotor.set(speed);
    rightLiftMotor.set(speed);
   }

   // function for lifting only the left climbing arm
   public void startLeftLift(double speed){
    leftLiftMotor.set(speed);
   }

   // function for lifting only the right climbing arm
   public void startRightLift(double speed){
    rightLiftMotor.set(speed);
   }

   // resets encoder values on lift motors
   public void resetEncoders(){
     leftLiftMotor.setSelectedSensorPosition(0);
     rightLiftMotor.setSelectedSensorPosition(0);
   }

   //stops lift motors
  public void stopLift(){
    //liftMotors.set(0);
    leftLiftMotor.set(0);
    rightLiftMotor.set(0);
  }
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

} // End of Class
