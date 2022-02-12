// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Drivetrain extends SubsystemBase {
        /**
         * The maximum angular velocity of the robot in radians per second.
         * <p>
         * This is a measure of how fast the robot can rotate in place.
         */
        // Here we calculate the theoretical maximum angular velocity. You can also
        // replace this with a measured amount.
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                        Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

        private final SwerveDriveKinematics m_kinematics;
        private final AHRS m_navx;
        private SwerveDriveOdometry m_odometry;
        private ShuffleboardTab tab, gTab;
        private boolean IS_VERBOSE = false;
        private final SwerveModule m_frontLeftModule, m_frontRightModule, m_backLeftModule, m_backRightModule;
        private ChassisSpeeds m_chassisSpeeds;

        public Drivetrain() {
                tab = Shuffleboard.getTab("Drivetrain");
                gTab = Shuffleboard.getTab("Gyro Control Test");
                configureShuffleboardComponents();

                // By default we use a Pigeon for our gyroscope. But if you use another
                // gyroscope, like a NavX, you can change this.
                // The important thing about how you configure your gyroscope is that rotating
                // the robot counter-clockwise should
                // cause the angle reading to increase until it wraps back over to zero.
                // Remove if you are using a Pigeon
                // private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
                // Uncomment if you are using a NavX
                m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

                m_kinematics = new SwerveDriveKinematics(
                                // Front left
                                new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                                DRIVETRAIN_WHEELBASE_METERS / 2.0),
                                // Front right
                                new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                                -DRIVETRAIN_WHEELBASE_METERS / 2.0),
                                // Back left
                                new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                                DRIVETRAIN_WHEELBASE_METERS / 2.0),
                                // Back right
                                new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                                -DRIVETRAIN_WHEELBASE_METERS / 2.0));

                m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation());
                m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
                
                m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                                // This parameter is optional, but will allow you to see the current state of
                                // the module on the dashboard.
                                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(0, 0),
                                // This can either be STANDARD or FAST depending on your gear configuration
                                Mk4SwerveModuleHelper.GearRatio.L2,
                                // This is the ID of the drive motor
                                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                                // This is the ID of the steer motor
                                FRONT_LEFT_MODULE_STEER_MOTOR,
                                // This is the ID of the steer encoder
                                FRONT_LEFT_MODULE_STEER_ENCODER,
                                // This is how much the steer encoder is offset from true zero (In our case,
                                // zero is facing straight forward)
                                FRONT_LEFT_MODULE_STEER_OFFSET);

                // We will do the same for the other modules
                m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(2, 0),
                                Mk4SwerveModuleHelper.GearRatio.L2,
                                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                                FRONT_RIGHT_MODULE_STEER_MOTOR,
                                FRONT_RIGHT_MODULE_STEER_ENCODER,
                                FRONT_RIGHT_MODULE_STEER_OFFSET);

                m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(4, 0),
                                Mk4SwerveModuleHelper.GearRatio.L2,
                                BACK_LEFT_MODULE_DRIVE_MOTOR,
                                BACK_LEFT_MODULE_STEER_MOTOR,
                                BACK_LEFT_MODULE_STEER_ENCODER,
                                BACK_LEFT_MODULE_STEER_OFFSET);

                m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(6, 0),
                                Mk4SwerveModuleHelper.GearRatio.L2,
                                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                                BACK_RIGHT_MODULE_STEER_MOTOR,
                                BACK_RIGHT_MODULE_STEER_ENCODER,
                                BACK_RIGHT_MODULE_STEER_OFFSET);
        }

        /**
         * Sets the gyroscope angle to zero. This can be used to set the direction the
         * robot is currently facing to the
         * 'forwards' direction.
         */
        public void zeroGyroscope() {
                m_navx.zeroYaw();
        }

        public void resetOdometry(Pose2d pose) {
                m_odometry.resetPosition(pose, getGyroscopeRotation());
        }

        public Rotation2d getGyroscopeRotation() {

                if (m_navx.isMagnetometerCalibrated()) {
                        // We will only get valid fused headings if the magnetometer is calibrated
                        return Rotation2d.fromDegrees(m_navx.getFusedHeading());
                }

                // We have to invert the angle of the NavX so that rotating the robot
                // counter-clockwise makes the angle increase.
                return Rotation2d.fromDegrees(360.0 - m_navx.getYaw() * -1);
        }

        public void drive(ChassisSpeeds chassisSpeeds) {
                m_chassisSpeeds = chassisSpeeds;
        }

        public SwerveDriveKinematics getKinematics() {
                return m_kinematics;
        }

        @Override
        public void periodic() {
                SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

                m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[0].angle.getRadians());
                m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[1].angle.getRadians());
                m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[2].angle.getRadians());
                m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[3].angle.getRadians());
        }

        public void setModuleStates(SwerveModuleState[] desiredStates) {
                SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_VELOCITY_METERS_PER_SECOND);

                m_frontLeftModule.set(
                                desiredStates[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                desiredStates[0].angle.getRadians());
                m_frontRightModule.set(
                                desiredStates[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                desiredStates[1].angle.getRadians());
                m_backLeftModule.set(
                                desiredStates[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                desiredStates[2].angle.getRadians());
                m_backRightModule.set(
                                desiredStates[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                desiredStates[3].angle.getRadians());
        }

        public Pose2d getPose() {
                return m_odometry.getPoseMeters();
        }

        private void configureShuffleboardComponents() {
                gTab.getComponents().clear();

                /* Display 6-axis Processed Angle Data */
                gTab.addBoolean("IMU_Connected", m_navx::isConnected).withPosition(1, 1);
                gTab.addBoolean("IMU_IsCalibrating", m_navx::isCalibrating).withPosition(2, 1);
                gTab.addNumber("IMU_Yaw", m_navx::getYaw).withPosition(2, 1);
                gTab.addNumber("IMU_Pitch", m_navx::getPitch).withPosition(2, 2);
                gTab.addNumber("IMU_Roll", m_navx::getRoll).withPosition(2, 3);

                gTab.addNumber("IMU_TotalYaw", m_navx::getAngle).withPosition(3, 1);
                gTab.addNumber("IMU_YawRateDPS", m_navx::getRate).withPosition(3, 2);

                /* Display tilt-corrected, Magnetometer-based heading (requires */
                /* magnetometer calibration to be useful) */

                gTab.addNumber("IMU_CompassHeading", m_navx::getCompassHeading);

                /* Display 9-axis Heading (requires magnetometer calibration to be useful) */
                gTab.addNumber("IMU_FusedHeading", m_navx::getFusedHeading);

                if (IS_VERBOSE) {

                        /* These functions are compatible w/the WPI Gyro Class, providing a simple */
                        /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP */

                        // gTab.addNumber("IMU_TotalYaw", m_navx::getAngle);
                        // gTab.addNumber("IMU_YawRateDPS", m_navx::getRate);

                        /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */

                        gTab.addNumber("IMU_Accel_X", m_navx::getWorldLinearAccelX);
                        gTab.addNumber("IMU_Accel_Y", m_navx::getWorldLinearAccelY);
                        gTab.addBoolean("IMU_IsMoving", m_navx::isMoving);
                        gTab.addBoolean("IMU_IsRotating", m_navx::isRotating);

                        /* Display estimates of velocity/displacement. Note that these values are */
                        /* not expected to be accurate enough for estimating robot position on a */
                        /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
                        /* of these errors due to single (velocity) integration and especially */
                        /* double (displacement) integration. */

                        gTab.addNumber("Velocity_X", m_navx::getVelocityX);
                        gTab.addNumber("Velocity_Y", m_navx::getVelocityY);
                        gTab.addNumber("Displacement_X", m_navx::getDisplacementX);
                        gTab.addNumber("Displacement_Y", m_navx::getDisplacementY);

                        /* Display Raw Gyro/Accelerometer/Magnetometer Values */
                        /* NOTE: These values are not normally necessary, but are made available */
                        /* for advanced users. Before using this data, please consider whether */
                        /* the processed data (see above) will suit your needs. */

                        gTab.addNumber("RawGyro_X", m_navx::getRawGyroX);
                        gTab.addNumber("RawGyro_Y", m_navx::getRawGyroY);
                        gTab.addNumber("RawGyro_Z", m_navx::getRawGyroZ);
                        gTab.addNumber("RawAccel_X", m_navx::getRawAccelX);
                        gTab.addNumber("RawAccel_Y", m_navx::getRawAccelY);
                        gTab.addNumber("RawAccel_Z", m_navx::getRawAccelZ);
                        gTab.addNumber("RawMag_X", m_navx::getRawMagX);
                        gTab.addNumber("RawMag_Y", m_navx::getRawMagY);
                        gTab.addNumber("RawMag_Z", m_navx::getRawMagZ);
                        gTab.addNumber("IMU_Temp_C", m_navx::getTempC);

                        /* Sensor Board Information */
                        gTab.addString("FirmwareVersion", m_navx::getFirmwareVersion);

                        /* Quaternion Data */
                        /* Quaternions are fascinating, and are the most compact representation of */
                        /* orientation data. All of the Yaw, Pitch and Roll Values can be derived */
                        /* from the Quaternions. If interested in motion processing, knowledge of */
                        /* Quaternions is highly recommended. */
                        gTab.addNumber("QuaternionW", m_navx::getQuaternionW);
                        gTab.addNumber("QuaternionX", m_navx::getQuaternionX);
                        gTab.addNumber("QuaternionY", m_navx::getQuaternionY);
                        gTab.addNumber("QuaternionZ", m_navx::getQuaternionZ);

                        /* Connectivity Debugging Support */
                        gTab.addNumber("IMU_Byte_Count", m_navx::getByteCount);
                        gTab.addNumber("IMU_Update_Count", m_navx::getUpdateCount);
                }
        }
}