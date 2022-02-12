// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean HARDWARE_CONFIG_HAS_DRIVETRAIN = true;
    public static final boolean HARDWARE_CONFIG_HAS_INTAKE = true;
    public static final boolean HARDWARE_CONFIG_HAS_INDEX = true;
    public static final boolean HARDWARE_CONFIG_HAS_SHOOTER = true;
    public static final boolean HARDWARE_CONFIG_HAS_LIMELIGHT = false;
    public static final boolean HARDWARE_CONFIG_HAS_AUTOS = false;
    public static final boolean HARDWARE_CONFIG_HAS_SERVOS = false;
    public static final boolean HARDWARE_CONFIG_HAS_FEEDER = true;
    public static final boolean HARDWARE_CONFIG_HAS_CENTERER = true;

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5207;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5207;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 11;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 15;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 19;

    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(139.5278); //practicebot
    // public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(142-6); //compbot

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 12;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 16;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 20;

   public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(144.2137); //practicebot
    // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(166+180+5); //compbot

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 14;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 18;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 22;

   public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(226.3419); //practicebot
    // public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(308); //compbot

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 13;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 17;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 21;

    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(245.0254); //practicebot
    // public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(283+2); //compbot

    public static final int SHOOTER_LEFT = 27;
    public static final int SHOOTER_RIGHT = 28;
    public static final double SHOOTER_SPEED = 0.5;
    public static final double FLYWHEEL_GEAR_RATIO = 36.0 / 24.0;

    public static final int INDEXER_MOTOR = 24;
    public static final double FOWARDS_INDEX_SPEED = 0.5; 
    public static final double FOWARDS_INDEX_SPEED_SLOW = 0.15;
    public static final double REVERSE_INDEX_SPEED = -0.5;

    public static final int INTAKE = 23;
    public static final double INTAKE_FWD_SPEED = 0.5;
    public static final double INTAKE_REVERSE_SPEED = -0.5;

    public static final int CENTERER = 30;
    public static final double CENTERER_REVERSE_SPEED = -0.5;
    public static final double CENTERER_FORWARD_SPEED = 0.5;

    public static final int FEEDER = 29;
    public static final double FEEDER_FWD_SPEED = 0.75;
    public static final double FEEDER_BACKWARD_SPEED = -0.5;

    public enum SwervePosition {
        FrontRight, FrontLeft, BackRight, BackLeft;
    }

    public static final double MAX_SPEED = 4.0;
    public static final double MAX_ANGULAR_VELOCITY = Math.PI * 3; // radians per second
    public static final double MAX_ANGULAR_ACCELERATION = 10.0 * 2 * Math.PI; // radians per second squared

    /**
     * Used a power limiter factor. A value between 0 and 1 with 0 as full off and 1
     * being 100% allowed.
     * 
     * For example, a value of 0.25 will cap the power to the drive trian at 25%;
     */
    public static final double TRAINING_WHEELS = 0.75;
}
