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
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class Drivetrain extends SubsystemBase {
        private final SwerveDriveKinematics m_kinematics;
        private final AHRS m_navx;
        private SwerveDriveOdometry m_odometry;
        private ShuffleboardTab tab;
        private boolean IS_VERBOSE = false;
        private final SwerveModule m_frontLeftModule, m_frontRightModule, m_backLeftModule, m_backRightModule;
        private SwerveModuleState[] m_desiredStates;
        private ChassisSpeeds m_chassisSpeeds;

        // Logging
        private NetworkTableEntry odometryXEntry;
        private NetworkTableEntry odometryYEntry;
        private NetworkTableEntry odometryAngleEntry;

        public Drivetrain() {
                zeroGyroscope();
                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

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
                m_desiredStates = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

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
                                PRACTICE_FRONT_LEFT_MODULE_STEER_OFFSET);
                // COMPETITION_FRONT_LEFT_MODULE_STEER_OFFSET);

                // We will do the same for the other modules
                m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(2, 0),
                                Mk4SwerveModuleHelper.GearRatio.L2,
                                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                                FRONT_RIGHT_MODULE_STEER_MOTOR,
                                FRONT_RIGHT_MODULE_STEER_ENCODER,
                                PRACTICE_FRONT_RIGHT_MODULE_STEER_OFFSET);
                // COMPETITION_FRONT_RIGHT_MODULE_STEER_OFFSET);

                m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(4, 0),
                                Mk4SwerveModuleHelper.GearRatio.L2,
                                BACK_LEFT_MODULE_DRIVE_MOTOR,
                                BACK_LEFT_MODULE_STEER_MOTOR,
                                BACK_LEFT_MODULE_STEER_ENCODER,
                                PRACTICE_BACK_LEFT_MODULE_STEER_OFFSET);
                // COMPETITION_BACK_LEFT_MODULE_STEER_OFFSET);

                m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(6, 0),
                                Mk4SwerveModuleHelper.GearRatio.L2,
                                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                                BACK_RIGHT_MODULE_STEER_MOTOR,
                                BACK_RIGHT_MODULE_STEER_ENCODER,
                                PRACTICE_BACK_RIGHT_MODULE_STEER_OFFSET);
                // COMPETITION_BACK_RIGHT_MODULE_STEER_OFFSET);

                configureShuffleboardComponents();
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

                // We will only get valid fused headings if the magnetometer is calibrated
                // We have to invert the angle of the NavX so that rotating the robot
                // counter-clockwise makes the angle increase.
                return m_navx.isMagnetometerCalibrated() ? Rotation2d.fromDegrees(m_navx.getFusedHeading())
                                : Rotation2d.fromDegrees(360.0 - m_navx.getYaw() * -1);
        }

        public void drive(ChassisSpeeds chassisSpeeds) {
                m_desiredStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        }

        public SwerveDriveKinematics getKinematics() {
                return m_kinematics;
        }

        @Override
        public void periodic() {
                SwerveDriveKinematics.desaturateWheelSpeeds(m_desiredStates, MAX_VELOCITY_METERS_PER_SECOND);

                m_frontLeftModule.set(
                                m_desiredStates[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                m_desiredStates[0].angle.getRadians());
                m_frontRightModule.set(
                                m_desiredStates[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                m_desiredStates[1].angle.getRadians());
                m_backLeftModule.set(
                                m_desiredStates[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                m_desiredStates[2].angle.getRadians());
                m_backRightModule.set(
                                m_desiredStates[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                m_desiredStates[3].angle.getRadians());

                // @todo should we check the actual current values instead?
                m_odometry.update(getGyroscopeRotation(),
                                m_desiredStates[0],
                                m_desiredStates[1],
                                m_desiredStates[2],
                                m_desiredStates[3]);

                Pose2d pose = getPose();
                odometryXEntry.setDouble(pose.getX());
                odometryYEntry.setDouble(pose.getY());
                odometryAngleEntry.setDouble(pose.getRotation().getDegrees());
        }

        public void setModuleStates(SwerveModuleState[] desiredStates) {
                m_desiredStates = desiredStates;
        }

        public Pose2d getPose() {
                return m_odometry.getPoseMeters();
        }

        private void configureShuffleboardComponents() {
                tab.getComponents().clear();

                odometryXEntry = tab.add("X", 0.0)
                                .withPosition(8, 0)
                                .withSize(1, 1)
                                .getEntry();
                odometryYEntry = tab.add("Y", 0.0)
                                .withPosition(8, 1)
                                .withSize(1, 1)
                                .getEntry();
                odometryAngleEntry = tab.add("Angle", 0.0)
                                .withPosition(8, 2)
                                .withSize(1, 1)
                                .getEntry();

                /* Display 6-axis Processed Angle Data */
                tab.addBoolean("IMU_Connected", m_navx::isConnected);
                tab.addBoolean("IMU_IsCalibrating", m_navx::isCalibrating);
                tab.addNumber("IMU_Yaw", m_navx::getYaw);
                tab.addNumber("IMU_Pitch", m_navx::getPitch);
                tab.addNumber("IMU_Roll", m_navx::getRoll);

                tab.addNumber("IMU_TotalYaw", m_navx::getAngle);
                tab.addNumber("IMU_YawRateDPS", m_navx::getRate);

                /* Display tilt-corrected, Magnetometer-based heading (requires */
                /* magnetometer calibration to be useful) */
                tab.addNumber("IMU_CompassHeading", m_navx::getCompassHeading);

                /* Display 9-axis Heading (requires magnetometer calibration to be useful) */
                tab.addNumber("IMU_FusedHeading", m_navx::getFusedHeading);

                if (IS_VERBOSE) {

                        /* These functions are compatible w/the WPI Gyro Class, providing a simple */
                        /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP */

                        // tab.addNumber("IMU_TotalYaw", m_navx::getAngle);
                        // tab.addNumber("IMU_YawRateDPS", m_navx::getRate);

                        /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */

                        tab.addNumber("IMU_Accel_X", m_navx::getWorldLinearAccelX);
                        tab.addNumber("IMU_Accel_Y", m_navx::getWorldLinearAccelY);
                        tab.addBoolean("IMU_IsMoving", m_navx::isMoving);
                        tab.addBoolean("IMU_IsRotating", m_navx::isRotating);

                        /* Display estimates of velocity/displacement. Note that these values are */
                        /* not expected to be accurate enough for estimating robot position on a */
                        /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
                        /* of these errors due to single (velocity) integration and especially */
                        /* double (displacement) integration. */

                        tab.addNumber("Velocity_X", m_navx::getVelocityX);
                        tab.addNumber("Velocity_Y", m_navx::getVelocityY);
                        tab.addNumber("Displacement_X", m_navx::getDisplacementX);
                        tab.addNumber("Displacement_Y", m_navx::getDisplacementY);

                        /* Display Raw Gyro/Accelerometer/Magnetometer Values */
                        /* NOTE: These values are not normally necessary, but are made available */
                        /* for advanced users. Before using this data, please consider whether */
                        /* the processed data (see above) will suit your needs. */

                        tab.addNumber("RawGyro_X", m_navx::getRawGyroX);
                        tab.addNumber("RawGyro_Y", m_navx::getRawGyroY);
                        tab.addNumber("RawGyro_Z", m_navx::getRawGyroZ);
                        tab.addNumber("RawAccel_X", m_navx::getRawAccelX);
                        tab.addNumber("RawAccel_Y", m_navx::getRawAccelY);
                        tab.addNumber("RawAccel_Z", m_navx::getRawAccelZ);
                        tab.addNumber("RawMag_X", m_navx::getRawMagX);
                        tab.addNumber("RawMag_Y", m_navx::getRawMagY);
                        tab.addNumber("RawMag_Z", m_navx::getRawMagZ);
                        tab.addNumber("IMU_Temp_C", m_navx::getTempC);

                        /* Sensor Board Information */
                        tab.addString("FirmwareVersion", m_navx::getFirmwareVersion);

                        /* Quaternion Data */
                        /* Quaternions are fascinating, and are the most compact representation of */
                        /* orientation data. All of the Yaw, Pitch and Roll Values can be derived */
                        /* from the Quaternions. If interested in motion processing, knowledge of */
                        /* Quaternions is highly recommended. */
                        tab.addNumber("QuaternionW", m_navx::getQuaternionW);
                        tab.addNumber("QuaternionX", m_navx::getQuaternionX);
                        tab.addNumber("QuaternionY", m_navx::getQuaternionY);
                        tab.addNumber("QuaternionZ", m_navx::getQuaternionZ);

                        /* Connectivity Debugging Support */
                        tab.addNumber("IMU_Byte_Count", m_navx::getByteCount);
                        tab.addNumber("IMU_Update_Count", m_navx::getUpdateCount);
                }
        }

        public void stop() {
        }
}