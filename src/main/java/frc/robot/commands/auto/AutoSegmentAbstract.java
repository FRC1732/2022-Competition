// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public abstract class AutoSegmentAbstract implements AutoSegment {
    private Drivetrain drivetrain;
    private String name;

    /** Creates a new Auto10Feet. */
    public AutoSegmentAbstract(Drivetrain drivetrain, String name) {
        this.drivetrain = drivetrain;
        this.name = name;
    }

    public String getName() {
        return name;
    }

    public Command getCommand() {
        return getCommand(true);
    }

    public Command getCommand(boolean stopAtEnd) {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                Drivetrain.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(drivetrain.getKinematics());

        if (stopAtEnd) {
            config.setEndVelocity(0.0);
        }

        Trajectory trajectory = defineTrajactory(config);

        TrapezoidProfile.Constraints trapezoidProfile = new TrapezoidProfile.Constraints(
                Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                Drivetrain.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        var thetaController = new ProfiledPIDController(1, 0, 0, trapezoidProfile);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                drivetrain::getPose, // Functional interface to feed supplier
                drivetrain.getKinematics(),
                // Position controllers
                new PIDController(1, 0, 0),
                new PIDController(1, 0, 0),
                thetaController,
                drivetrain::setModuleStates,
                drivetrain);

        // Reset odometry to the starting pose of the trajectory.
        drivetrain.resetOdometry(trajectory.getInitialPose());

        return swerveControllerCommand;
    }

    abstract Trajectory defineTrajactory(TrajectoryConfig config);
}
