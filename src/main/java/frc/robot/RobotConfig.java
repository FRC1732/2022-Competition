// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.RobotDesignation;

/** Add your docs here. */
public class RobotConfig {

    public static double FRONT_LEFT_MODULE_STEER_OFFSET,
            FRONT_RIGHT_MODULE_STEER_OFFSET,
            BACK_LEFT_MODULE_STEER_OFFSET,
            BACK_RIGHT_MODULE_STEER_OFFSET,
            FLYWHEEL_P,
            FLYWHEEL_I,
            FLYWHEEL_D,
            FLYWHEEL_CURRENT_LIMIT;

    public RobotConfig(RobotDesignation designation) {
        switch (designation) {
            case PRACTICE:
                FRONT_LEFT_MODULE_STEER_OFFSET = Constants.PRACTICE_FRONT_LEFT_MODULE_STEER_OFFSET;
                FRONT_RIGHT_MODULE_STEER_OFFSET = Constants.PRACTICE_FRONT_RIGHT_MODULE_STEER_OFFSET;
                BACK_LEFT_MODULE_STEER_OFFSET = Constants.PRACTICE_BACK_LEFT_MODULE_STEER_OFFSET;
                BACK_RIGHT_MODULE_STEER_OFFSET = Constants.PRACTICE_BACK_RIGHT_MODULE_STEER_OFFSET;
                FLYWHEEL_P = Constants.PRACTICE_FLYWHEEL_P;
                FLYWHEEL_I = Constants.PRACTICE_FLYWHEEL_I;
                FLYWHEEL_D = Constants.PRACTICE_FLYWHEEL_D;
                FLYWHEEL_CURRENT_LIMIT = Constants.PRACTICE_FLYWHEEL_CURRENT_LIMIT;
                break;
            case COMPETITION:
            default: // Designation not specified - default to COMPETITION values
                FRONT_LEFT_MODULE_STEER_OFFSET = Constants.COMPETITION_FRONT_LEFT_MODULE_STEER_OFFSET;
                FRONT_RIGHT_MODULE_STEER_OFFSET = Constants.COMPETITION_FRONT_RIGHT_MODULE_STEER_OFFSET;
                BACK_LEFT_MODULE_STEER_OFFSET = Constants.COMPETITION_BACK_LEFT_MODULE_STEER_OFFSET;
                BACK_RIGHT_MODULE_STEER_OFFSET = Constants.COMPETITION_BACK_RIGHT_MODULE_STEER_OFFSET;
                FLYWHEEL_P = Constants.COMPETITION_FLYWHEEL_P;
                FLYWHEEL_I = Constants.COMPETITION_FLYWHEEL_I;
                FLYWHEEL_D = Constants.COMPETITION_FLYWHEEL_D;
                FLYWHEEL_CURRENT_LIMIT = Constants.COMPETITION_FLYWHEEL_CURRENT_LIMIT;                
                break;

        }
    }

}
