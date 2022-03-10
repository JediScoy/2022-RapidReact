package frc.robot.subsystems;
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


public class Intake extends SubsystemBase {
  
  private TalonFX intakeMotor;

  public Intake() {
    intakeMotor = new TalonFX(Constants.INTAKE_MOTOR);
    intakeMotor.setInverted(false);
    intakeMotor.setNeutralMode(NeutralMode.Coast);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Dashboard stuff would go here
  }

  // Sets the Intake system speed as a percentage between -1 and 1. Speed is given actual value in the RobotContrainer.java button code
  public void startIntake(double speed){
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setIntakeSpeed() {
    intakeMotor.set(ControlMode.PercentOutput, 0.5);
  }

  public void stopIntake(){
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


} // End of Class

