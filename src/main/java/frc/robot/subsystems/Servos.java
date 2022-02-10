// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Servo;

public class Servos extends SubsystemBase {
    private Servo servoX;
    private Servo servoY;
    private double servoPositionX;
    private double servoPositionY;

    /** Creates a new Limelight. */
    public Servos() {
        servoX = new Servo(0); // Servo connected to roboRIO PWM 0, x-axis
        servoY = new Servo(1); // Servo connected to roboRIO PWM 1, y-axis
        servoPositionX = 0.5;
        servoPositionY = 0.3;
        servoX.setPosition(servoPositionX);
        servoY.setPosition(servoPositionY);
    }

    public double getServoX() {
        return servoX.getPosition();
    }

    public double getServoY() {
        return servoY.getPosition();
    }

    public void incrementSetServoX() {
        servoPositionX += 0.05;
        servoX.set(servoPositionX);
    }

    public void incrementSetServoY() {
        servoPositionY += 0.05;
        servoY.set(servoPositionY);
    }

    public void decrementSetServoX() {
        servoPositionX -= 0.05;
        servoX.set(servoPositionX);
    }

    public void decrementSetServoY() {
        servoPositionY -= 0.05;
        servoY.set(servoPositionY);
    }

    @Override
    public void periodic() {
        // read values periodically
    }
}
