// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.*;

import java.util.List;

/** Add your docs here. */
public abstract class DriveSegmentBaseCommand extends SwerveControllerCommand{
    private Drivetrain _drivetrain;
    private Pose2d _initialPose;
    private Rotation2d _endRotation;

    /** Creates a new Auto10Feet. */
    public DriveSegmentBaseCommand(Drivetrain drivetrain,
                    List<Translation2d> waypoints,
                    Rotation2d startRotation,
                    Rotation2d endRotation,
                    boolean stopAtEnd) {
        super(getTrajectory(waypoints, getDefaultTrajectoryConfig(drivetrain, stopAtEnd)),
                drivetrain::getPose, // Functional interface to feed supplier
                drivetrain.getKinematics(),
                // Position controllers
                new PIDController(1, 0, 0),
                new PIDController(1, 0, 0),
                getThetaController(),
                () -> endRotation,
                drivetrain::setModuleStates,
                drivetrain);
        _drivetrain = drivetrain;
        var firstWaypoint = waypoints.get(0);
        _initialPose = new Pose2d(firstWaypoint.getX(), firstWaypoint.getY(), startRotation); //getTrajectoryRotation(waypoints));
        _endRotation = endRotation;
    }

    @Override
    public void initialize() {
        super.initialize();
        // Reset odometry to the starting pose of the trajectory.
        _drivetrain.resetOdometry(_initialPose);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _drivetrain.zeroGyroscope(_endRotation);
    }

    private static ProfiledPIDController getThetaController() {
        var profileConstraints = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                MAX_ANGULAR_ACCELERATION * Math.PI / 180 * 5);
        var thetaController = new ProfiledPIDController(7, 0, 0, profileConstraints);
        thetaController.enableContinuousInput(Math.PI * -1, Math.PI);
        return thetaController;
    }

    private static TrajectoryConfig getDefaultTrajectoryConfig(Drivetrain drivetrain, boolean stopAtEnd) {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                MAX_VELOCITY_METERS_PER_SECOND,
                MAX_ACCELERATION_METERS_PER_SECOND_SQUARED*2);
        // Add kinematics to ensure max speed is actually obeyed
        config.setKinematics(drivetrain.getKinematics());
        if (stopAtEnd)
            config.setEndVelocity(0.0);
        return config;
    }

    private static Trajectory getTrajectory(List<Translation2d> waypoints, TrajectoryConfig config) {
        if (waypoints.size() < 2) {
            return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0, 0, new Rotation2d(0)),
                    List.of(),
                    new Pose2d(0, 0, new Rotation2d(0)),
                    config);
        }
        var rotation = getTrajectoryRotation(waypoints);
        var interiorPoints = waypoints.subList(1, waypoints.size() - 1);
        var startPoint = waypoints.get(0);
        var endPoint = waypoints.get(waypoints.size() - 1);
        return TrajectoryGenerator.generateTrajectory(
                new Pose2d(startPoint.getX(), startPoint.getY(), rotation),
                interiorPoints,
                new Pose2d(endPoint.getX(), endPoint.getY(), rotation),
                config);
    }

    private static Rotation2d getTrajectoryRotation(List<Translation2d> waypoints) {
        if (waypoints.size() < 2)
            return Rotation2d.fromDegrees(0);
        var startPoint = waypoints.get(0);
        var endPoint = waypoints.get(waypoints.size() - 1);
        double xdist = endPoint.getX() - startPoint.getX();
        double ydist = endPoint.getY() - startPoint.getY();
        double angle = Math.atan2(ydist, xdist);
        return new Rotation2d(angle);
    }
}
