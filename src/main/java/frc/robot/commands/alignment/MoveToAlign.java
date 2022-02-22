// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.alignment;

/** Add your docs here. */
public interface MoveToAlign {
    void move(Direction direction);

    void stop();

    public enum Direction {
        Left, Right, None;
    }
}
