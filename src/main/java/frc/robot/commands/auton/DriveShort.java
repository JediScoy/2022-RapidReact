package frc.robot.commands.auton;

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

import frc.robot.subsystems.Drivetrain;

public class DriveShort extends CommandBase{
boolean isFin = false;
    TrajectoryConfig trajectoryConfig;
    Trajectory trajectory;
    PIDController xController;
    PIDController yController;
    Drivetrain m_drivetrain;
    SwerveControllerCommand swerveControllerCommand;

public void driveShort(Drivetrain m_drivetrain) {

        addRequirements(m_drivetrain);
    this.m_drivetrain = m_drivetrain;
    trajectoryConfig = new TrajectoryConfig(
      Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, //original called for max speed (just in case im making one of the dumb physics mistakes)
      Drivetrain.MAX_ACCELERATION_METERS_SECOND_SQUARED)
                .setKinematics(Constants.m_kinematics);

        trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                    new Translation2d(.33, 0),
                    new Translation2d(.66, 0)),
            new Pose2d(1, 0, Rotation2d.fromDegrees(0)), //full rotation?
            trajectoryConfig
            );
    
            xController = new PIDController(0, 0, 0);
            yController = new PIDController(0, 0, 0);
            ProfiledPIDController thetaController = new ProfiledPIDController(
                    Constants.kPThetaController, 0, 0, Constants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);
                
            swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                m_drivetrain::getPose,
                Constants.m_kinematics,
                xController,
                yController,
                thetaController,
                m_drivetrain::setModuleStates,//?
                m_drivetrain);

              
  // Reset odometry to the starting pose of the trajectory.
  m_drivetrain.resetOdometry(trajectory.getInitialPose());

  // Run path following command, then stop at the end.
  // return swerveControllerCommand.andThen(() -> m_drivetrainSubsystem.drive(0, 0, 0, false));
}



            @Override
            public void execute(){
              

                m_drivetrain.resetOdometry(trajectory.getInitialPose());
                swerveControllerCommand.execute();
                isFin = true;
                m_drivetrain.stop();
        
            }

            @Override
            public void end(boolean interrupted) {
                m_drivetrain.stop();
            }
   
            @Override
            public boolean isFinished() {
                    if (isFin) {
                          return true;
                    }
                    else{
                            return false;
                    }
                
            }
}