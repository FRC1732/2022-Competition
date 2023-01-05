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
import static frc.robot.Constants.*;

import java.util.List;

/** Add your docs here. */
public abstract class DriveSegmentBaseCommand extends HilltopSwerveControllerCommand{
    private AutoSwerveDriveSubsystem _drivetrain;
    private Pose2d _initialPose;
    private boolean _resetPostion;
    private Rotation2d _startRotation;

    /** Creates a new Auto10Feet. */
    public DriveSegmentBaseCommand(AutoSwerveDriveSubsystem drivetrain,
                    List<Translation2d> waypoints,
                    Rotation2d startRotation,
                    Rotation2d endRotation,
                    boolean stopAtEnd,
                    boolean resetPosition) {
        super(getDefaultTrajectoryConfig(drivetrain, stopAtEnd),
                new Pose2d(waypoints.get(1), endRotation),
                drivetrain::getPose, // Functional interface to feed supplier
                drivetrain.getKinematics(),
                // Position controllers
                new PIDController(1, 0, 0),
                new PIDController(1, 0, 0),
                getThetaController(),
                drivetrain::setModuleStates,
                drivetrain);
        _drivetrain = drivetrain;
        var firstWaypoint = waypoints.get(0);
        _initialPose = new Pose2d(firstWaypoint.getX(), firstWaypoint.getY(), startRotation); //getTrajectoryRotation(waypoints));
        _resetPostion = resetPosition;
        _startRotation = startRotation;
    }

    @Override
    public void initialize() {
        super.initialize();
        
        if (_resetPostion)
        {
            _drivetrain.zeroGyroscope(_startRotation.times(-1));
            _drivetrain.resetOdometry(_initialPose);
        }
        // if (!_resetPostion)
        //     _initialPose = new Pose2d(_drivetrain.getPose().getX(), _drivetrain.getPose().getY(), _drivetrain.getPose().getRotation());//_startRotation);
        // Reset odometry to the starting pose of the trajectory.        
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        // if (_resetPostion)
            // _drivetrain.zeroGyroscope(_endRotation);
    }

    private static ProfiledPIDController getThetaController() {
        var profileConstraints = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                MAX_ANGULAR_ACCELERATION * Math.PI / 180 * 5);
        var thetaController = new ProfiledPIDController(7, 0, 0, profileConstraints);
        thetaController.enableContinuousInput(Math.PI * -1, Math.PI);
        return thetaController;
    }

    private static TrajectoryConfig getDefaultTrajectoryConfig(AutoSwerveDriveSubsystem drivetrain, boolean stopAtEnd) {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                MAX_VELOCITY_METERS_PER_SECOND,
                MAX_ACCELERATION_METERS_PER_SECOND_SQUARED*3);
        // Add kinematics to ensure max speed is actually obeyed
        config.setKinematics(drivetrain.getKinematics());
        if (stopAtEnd)
            config.setEndVelocity(0.0);
        return config;
    }

    

    protected static final Pose2d WAYPOINT_A = new Pose2d(2.9238, 0.41186, Rotation2d.fromDegrees(-21));
    protected static final Pose2d WAYPOINT_B = new Pose2d(0.82910 + -0.15, 0.661008, Rotation2d.fromDegrees(0));
    protected static final Pose2d WAYPOINT_C = new Pose2d(1.653747 + -.2, 3.27440, Rotation2d.fromDegrees(-112));
    protected static final Pose2d WAYPOINT_D = new Pose2d(1.7598, 2.1827, Rotation2d.fromDegrees(-44));
    protected static final Pose2d WAYPOINT_E = new Pose2d(1.51203 - 0.2, 6.7617 + 0.5, Rotation2d.fromDegrees(-45));
    protected static final Pose2d WAYPOINT_F = new Pose2d(5.16403, 2.18839, Rotation2d.fromDegrees(-135));
    protected static final Pose2d WAYPOINT_G = new Pose2d(6.42539, 3.4711, Rotation2d.fromDegrees(-135));
    protected static final Pose2d WAYPOINT_H = new Pose2d(1.7778, 0, Rotation2d.fromDegrees(1.5));
    protected static final Pose2d WAYPOINT_I = new Pose2d(4.5541, 1.21188, Rotation2d.fromDegrees(-111));
    protected static final Pose2d WAYPOINT_J = new Pose2d(1.8958, 0.8403, Rotation2d.fromDegrees(-21));
    protected static final Pose2d WAYPOINT_K = new Pose2d(0.651352, 1.2713, Rotation2d.fromDegrees(-21));
    protected static final Pose2d WAYPOINT_L = new Pose2d(0.634897, 0, Rotation2d.fromDegrees(0));
    protected static final Pose2d WAYPOINT_M = new Pose2d(1.099517, 1.241778, Rotation2d.fromDegrees(-23.5));  
    protected static final Pose2d WAYPOINT_N = new Pose2d(1.75, 0.653031, Rotation2d.fromDegrees(0.8));
    protected static final Pose2d WAYPOINT_O = new Pose2d(2.5047367, 3.2231969, Rotation2d.fromDegrees(-63.45));
    protected static final Pose2d WAYPOINT_P = new Pose2d(5.896412, 2.678778, Rotation2d.fromDegrees(-123));
    protected static final Pose2d WAYPOINT_Q = new Pose2d(0, 0, Rotation2d.fromDegrees(-88.5));
    protected static final Pose2d WAYPOINT_R = new Pose2d(0, 1.3, Rotation2d.fromDegrees(-90));
    protected static final Pose2d WAYPOINT_S = new Pose2d(6.896412, 3.678778, Rotation2d.fromDegrees(-123));
    protected static final Pose2d WAYPOINT_T = new Pose2d(1.51203 - 0.2 + .4, 6.7617 + 0.5 + -.4, Rotation2d.fromDegrees(-45)); //back up for Adit
    
}
