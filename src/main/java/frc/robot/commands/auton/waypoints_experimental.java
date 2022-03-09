package frc.robot.commands.auton;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

// https://www.chiefdelphi.com/t/waymaker-a-new-waypoint-editor-for-path-planning/396363/4
public class waypoints_experimental {
    private Trajectory trajectoryForPath(List<Pose2d> path, boolean reversed) {
    
        TrajectoryConfig config =
            new TrajectoryConfig(Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                                 Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
                .setKinematics(Constants.m_kinematics)
                .setReversed(reversed);
    
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
          path,
          config);
    
          return trajectory;
      }
}
