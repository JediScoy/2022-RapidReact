package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

// Shuffleboard imports
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts; // Displaying data?
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard; // Displaying data?
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab; // Displaying data?

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import edu.wpi.first.wpilibj.motorcontrol.MotorController;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class Index extends SubsystemBase {

// NOTE you must use WPI_TalonFX subclass rather than TalonFX when grouping motors... 60 minutes of my life wasted
private final WPI_TalonFX midIndexMotor = new WPI_TalonFX(Constants.MID_INDEX_MOTOR);
private final WPI_TalonFX highIndexMotor = new WPI_TalonFX(Constants.HIGH_INDEX_MOTOR);
// private final MotorController indexMotors;

private final MotorControllerGroup indexMotors = 
    new MotorControllerGroup(midIndexMotor, highIndexMotor);
    
  

  public Index() {
   // indexMotors = indexMotors.setNeutralMode(NeutralMode.Coast);

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