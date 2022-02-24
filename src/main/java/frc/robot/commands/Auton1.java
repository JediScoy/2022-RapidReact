package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;

import frc.robot.subsystems.DrivetrainSubsystem;

public class Auton1 extends CommandBase{
    TrajectoryConfig trajectoryConfig;
    Trajectory trajectory;
    PIDController xController;
    PIDController yController;
    DrivetrainSubsystem m_drivetrainSubsystem;
    SwerveControllerCommand swerveControllerCommand;
public Auton1(DrivetrainSubsystem m_drivetrainSubsystem) {
    this.m_drivetrainSubsystem = m_drivetrainSubsystem;
    trajectoryConfig = new TrajectoryConfig(
        Constants.MAX_VELOCITY_METERS_PER_SECOND, //original called for max speed (just in case im making one of the dumb physics mistakes)
        Constants.MAX_ACCELERATION_METERS_SECOND_SQUARED)
                .setKinematics(Constants.m_kinematics);

        trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                    new Translation2d(1, 0),
                    new Translation2d(1, -1)),
            new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
            trajectoryConfig
            );
    
            xController = new PIDController(Constants.kPXController, 0, 0);
            yController = new PIDController(Constants.kPYController, 0, 0);
            ProfiledPIDController thetaController = new ProfiledPIDController(
                    Constants.kPThetaController, 0, 0, Constants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);
                
           swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                m_drivetrainSubsystem::getPose,
                Constants.m_kinematics,
                xController,
                yController,
                thetaController,
                m_drivetrainSubsystem::setModuleStates,//?
                m_drivetrainSubsystem);

              

}
            @Override
            public void execute(){
              
                   m_drivetrainSubsystem.resetOdometry(trajectory.getInitialPose());
                  
                    m_drivetrainSubsystem.stop();
        

            }
    
  
    // 2. Generate trajectory
    
    // 3. Define PID controllers for tracking trajectory
    
    // 4. Construct command to follow trajectory
   

    // 5. Add some init and wrap-up, and return everything
   
}
