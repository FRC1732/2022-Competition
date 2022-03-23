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
    
    // ================== SUBSYSTEM CONFIG ==================
    public static final boolean HARDWARE_CONFIG_HAS_DRIVETRAIN = false;
    public static final boolean HARDWARE_CONFIG_HAS_INTAKE = false;
    public static final boolean HARDWARE_CONFIG_HAS_INDEX = false;
    public static final boolean HARDWARE_CONFIG_HAS_SHOOTER = false;
    public static final boolean HARDWARE_CONFIG_HAS_LIMELIGHT = false;
    public static final boolean HARDWARE_CONFIG_HAS_AUTOS = false;
    public static final boolean HARDWARE_CONFIG_HAS_SERVOS = false;
    public static final boolean HARDWARE_CONFIG_HAS_FEEDER = false;
    public static final boolean HARDWARE_CONFIG_HAS_CENTERER = false;
    public static final boolean HARDWARE_CONFIG_HAS_CLIMBER = false;
    public static final boolean HARDWARE_CONFIG_HAS_COLORSENSOR = true;
    
    // ================== CAN IDS ==================
    public static final int CAN_PNEUMATIC_ID = 5;
    public static final int CAN_FRONT_LEFT_MODULE_DRIVE_MOTOR = 11;
    public static final int CAN_FRONT_RIGHT_MODULE_DRIVE_MOTOR = 12;
    public static final int CAN_BACK_RIGHT_MODULE_DRIVE_MOTOR = 13;
    public static final int CAN_BACK_LEFT_MODULE_DRIVE_MOTOR = 14;

    public static final int CAN_FRONT_LEFT_MODULE_STEER_MOTOR = 15;
    public static final int CAN_FRONT_RIGHT_MODULE_STEER_MOTOR = 16;
    public static final int CAN_BACK_RIGHT_MODULE_STEER_MOTOR = 17;
    public static final int CAN_BACK_LEFT_MODULE_STEER_MOTOR = 18;

    public static final int CAN_FRONT_LEFT_MODULE_STEER_ENCODER = 19;
    public static final int CAN_FRONT_RIGHT_MODULE_STEER_ENCODER = 20;
    public static final int CAN_BACK_RIGHT_MODULE_STEER_ENCODER = 21;
    public static final int CAN_BACK_LEFT_MODULE_STEER_ENCODER = 22;

    public static final int CAN_INTAKE_MOTOR = 23;
    public static final int CAN_INDEXER_MOTOR = 24;
    public static final int CAN_SHOOTER_MOTOR_LEFT = 27;
    public static final int CAN_SHOOTER_MOTOR_RIGHT = 28;
    public static final int CAN_FEEDER_MOTOR = 29;
    public static final int CAN_CENTERER_MOTOR = 30;

    public static final int CAN_CLIMBER_LEFT_ARM_ONE_MOTOR_ID = 25;
    public static final int CAN_CLIMBER_RIGHT_ARM_ONE_MOTOR_ID = 32;
    public static final int CAN_CLIMBER_LEFT_ARM_TWO_MOTOR_ID = 26;
    public static final int CAN_CLIMBER_RIGHT_ARM_TWO_MOTOR_ID = 31;

    
    /**
     * Used a power limiter factor. A value between 0 and 1 with 0 as full off and 1
     * being 100% allowed.
     * 
     * For example, a value of 0.25 will cap the power to the drive trian at 25%;
     */
    public static final double TRAINING_WHEELS = 1.00;

    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is
     * useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;

    // The formula for calculating the theoretical maximum velocity is:
    // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
    // pi
    // By default this value is setup for a Mk4 standard module using Falcon500s to
    // drive.
    // An example of this constant for a Mk4 L2 module with NEOs to drive is:
    // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
    // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight
     * line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.96824;
    public static final double MIN_VELOCITY_METERS_PER_SECOND = 0.125;

    /**
     * Auto swerve wants a max acceleration.
     * FIXME: do we have a better option than the max velocity?
     */
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1;

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
    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also
    // replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
    public static final double MAX_ANGULAR_VELOCITY = Math.PI * 3;              // radians per second
    public static final double MIN_ANGULAR_VELOCITY = Math.PI * 0.15;              // radians per second
    public static final double MAX_ANGULAR_ACCELERATION = Math.PI * 10.0 * 2;   // radians per second squared

    public static final double PRACTICE_FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(140.1278);
    public static final double COMPETITION_FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(138.8);

    public static final double PRACTICE_FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(177.1837);
    public static final double COMPETITION_FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(352.2);

    public static final double PRACTICE_BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(221.4019);
    public static final double COMPETITION_BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.39);

    public static final double PRACTICE_BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(244.2654);
    public static final double COMPETITION_BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(-41.36);

    // ================== SHOOTER CONSTANTS ==================
    public static final double TARGET_RPM_NEAR = 2000.0; //3250.0; @todo different for pbot
    public static final double TARGET_RPM_FAR = 1900.0; // @todo different for pbot
    public static final double FLYWHEEL_GEAR_RATIO = 1.0; //36.0 / 24.0; @todo different for pbot
    public static final double FLYWHEEL_TICKS_TO_ROTATIONS_COEFFICIENT = 1.0 / 2048.0 * FLYWHEEL_GEAR_RATIO;
    public static final double FLYWHEEL_TICKS_TO_RPM_COEFFICIENT = FLYWHEEL_TICKS_TO_ROTATIONS_COEFFICIENT
            * (1000.0 / 100.0) * (60.0);
    // Calculated: set to 4/battery voltage output, measure speed, set this to
    // (4-static_constant)/speed
    public static final double FLYWHEEL_FEEDFORWARD_COEFFICIENT = 0.00215; //0.0012 @todo different for pbot
    public static final double FLYWHEEL_STATIC_FRICTION_CONSTANT = 0; //0.23; minimum voltage to spin shooter @todo different for pbot 
    public static final double FLYWHEEL_ALLOWABLE_ERROR = 50.0;

    public static final double PRACTICE_FLYWHEEL_P = 0.4;
    public static final double PRACTICE_FLYWHEEL_I = 0.0;
    public static final double PRACTICE_FLYWHEEL_D = 0.0;
    public static final double PRACTICE_FLYWHEEL_CURRENT_LIMIT = 10.0;

    public static final double COMPETITION_FLYWHEEL_P = 0.3;
    public static final double COMPETITION_FLYWHEEL_I = 0.0;
    public static final double COMPETITION_FLYWHEEL_D = 0.0;
    public static final double COMPETITION_FLYWHEEL_CURRENT_LIMIT = 10.0;

    public static final double HOOD_CHANGE_DISTANCE = 8.75;
    public static final double HOOD_CHANGE_DISTANCE_THRESHOLD = 0.25;

    // ================== INDEXER CONSTANTS ==================
    public static final double FOWARDS_INDEX_SPEED = -0.15; // @todo setting inverted didn't work properly
    public static final double FOWARDS_INDEX_SPEED_SLOW = -0.1;
    public static final double REVERSE_INDEX_SPEED = 0.15;

    // ================== INTAKE CONSTANTS ==================
    public static final double INTAKE_FWD_SPEED = -0.75;
    public static final double INTAKE_REVERSE_SPEED = 0.2;

    // ================== CENTERER CONSTANTS ==================
    public static final double CENTERER_REVERSE_SPEED = -0.25;
    public static final double CENTERER_FORWARD_SPEED = 0.75;

    // ================== CLIMBER CONSTANTS ==================
    public static final double CLIMBER_UP_SPEED = .75;
    public static final double CLIMBER_DOWN_SPEED = -.5;
    public static final double CLIMBER_HOME_POSITION_ZERO = 0.0;
    public static final double CLIMBER_FRONT_EXTEND_TARGET_POSITION = 100.0;
    public static final double CLIMBER_BACK_EXTEND_TARGET_POSITION = 105.0;
    public static final double CLIMBER_FRONT_RETRACT_TARGET_POSITION = 25.0;
    public static final double CLIMBER_BACK_RETRACT_TARGET_POSITION = 25.0;


    // ================== FEEDER CONSTANTS ==================
    public static final double FEEDER_FWD_SPEED = 0.75;
    public static final double FEEDER_BACKWARD_SPEED = -0.5;

    // ================== LIMELIGHT CONSTANTS ==================
    public static final int LL_LEDSTATE_USE = 0;
    public static final int LL_LEDSTATE_OFF = 1;
    public static final int LL_LEDSTATE_BLINK = 2;
    public static final int LL_LEDSTATE_ON = 3;
    public static final int LL_CAMMODE_VISION = 0;
    public static final int LL_CAMMODE_DRIVER = 1;
    public static final double LIMELIGHT_HEIGHT = 2.23333333333333333;
    
    // ================== INTAKE PNEUMATIC CONSTANTS ==================
    public static final int INTAKE_SOLENOID_CHANNEL_LEFT = 9-8;
    public static final int INTAKE_SOLENOID_CHANNEL_RIGHT = 8-8;

    // ================== SHOOTER PNEUMATIC CONSTANTS ==================
    public static final int SHOOTER_SOLENOID_CHANNEL_HOOD = 10-8;

    // ================== CLIMBER PNEUMATIC CONSTANTS ==================
    public static final int CLIMBER_SOLENOID_CHANNEL_BOTH_TILTER = 11-8;
    public static final int CLIMBER_SOLENOID_CHANNEL_STATIONARY_BRAKE_ONE = 12-8;
    public static final int CLIMBER_SOLENOID_CHANNEL_MOVING_BRAKE_TWO = 13-8;


    public enum RobotDesignation {
        NONE,
        PRACTICE,
        COMPETITION,
    };

    public enum ShuffleBoardLogging
    {
        NONE,
        DEBUG,
        COMPETITION,
    };
}
