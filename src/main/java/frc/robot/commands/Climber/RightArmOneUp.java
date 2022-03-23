// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class RightArmOneUp extends CommandBase {
  Climber climber;
  /** Creates a new rightArmUp. */
  public RightArmOneUp(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(climber);
    this.climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.climberRightArmOneUp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.climberRightArmOneStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
