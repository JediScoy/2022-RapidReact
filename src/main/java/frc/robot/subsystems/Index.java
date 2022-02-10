package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// Shuffleboard imports
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class Index extends SubsystemBase {

// NOTE you must use WPI_TalonFX subclass rather than TalonFX when grouping motors... 60 minutes of my life wasted
private final WPI_TalonFX frontIndexMotor = new WPI_TalonFX(Constants.FRONT_INDEX_MOTOR);
private final WPI_TalonFX backIndexMotor = new WPI_TalonFX(Constants.BACK_INDEX_MOTOR);
// private final MotorController indexMotors;


private final MotorControllerGroup indexMotors = 
    new MotorControllerGroup(frontIndexMotor, backIndexMotor);
    
  

  public Index(double speed){
      frontIndexMotor.set(ControlMode.PercentOutput, speed); // Need to fix so when the button is pressed it sets the speed for moto group
      backIndexMotor.set(ControlMode.PercentOutput, speed);
  }
  public void stopLauncher() {
    frontIndexMotor.set(ControlMode.PercentOutput, 0);
    backIndexMotor.set(ControlMode.PercentOutput, 0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

} // End of Class