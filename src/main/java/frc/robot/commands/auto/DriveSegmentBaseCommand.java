// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.*;

/** Add your docs here. */
public abstract class DriveSegmentBaseCommand extends SwerveControllerCommand{
    private Drivetrain drivetrain;
    private Trajectory trajectory;

    /** Creates a new Auto10Feet. */
    public DriveSegmentBaseCommand(Drivetrain drivetrain, Trajectory trajectory) {
        super(trajectory,
                drivetrain::getPose, // Functional interface to feed supplier
                drivetrain.getKinematics(),
                // Position controllers
                new PIDController(1, 0, 0),
                new PIDController(1, 0, 0),
                getThetaController(),
                drivetrain::setModuleStates,
                drivetrain);
                this.drivetrain = drivetrain;
                this.trajectory = trajectory;
    }

    private static ProfiledPIDController getThetaController() {
        var profileConstraints = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SECOND/4,
                MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        var thetaController = new ProfiledPIDController(1, 0, 0, profileConstraints);
        thetaController.enableContinuousInput(Math.PI * -1, Math.PI);
        return thetaController;
    }

    @Override
    public void initialize() {
        super.initialize();
        // Reset odometry to the starting pose of the trajectory.
        drivetrain.resetOdometry(trajectory.getInitialPose());
    }

    protected static TrajectoryConfig getDefaultTrajectoryConfig(Drivetrain drivetrain, boolean stopAtEnd) {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                MAX_VELOCITY_METERS_PER_SECOND / 4,
                MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        // Add kinematics to ensure max speed is actually obeyed
        config.setKinematics(drivetrain.getKinematics());
        if (stopAtEnd)
            config.setEndVelocity(0.0);
        return config;
    }
}
