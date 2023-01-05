// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.RobotDesignation;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.FileReader;
import java.io.BufferedReader;

import com.team1732.autolib.Library;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command m_testCommand;

  private RobotContainer m_robotContainer;
  private RobotConfig m_robotConfig;

  File f;
  BufferedWriter bw;
  FileWriter fw;
  FileReader fr;
  BufferedReader br;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotConfig = new RobotConfig(readFile());
    m_robotContainer = new RobotContainer(m_robotConfig);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    Library classUnderTest = new Library();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null && classUnderTest.someLibraryMethod()) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    LiveWindow.disableAllTelemetry();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    if(m_testCommand != null){
      m_testCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

   // m_testCommand = m_robotContainer.provideTestCommand();

    if(m_testCommand != null) {
      m_testCommand.schedule();
    }
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  public RobotDesignation readFile() {
    RobotDesignation rd;
    try {
      f = new File("/home/lvuser/Robot.txt");
      fr = new FileReader(f);
    } catch (IOException e) {
      e.printStackTrace();
    }
    br = new BufferedReader(fr);
    String type = "";

    try {
      type = br.readLine();
      br.close();
      fr.close();
    } catch (IOException e) {
      e.printStackTrace();
    }

    // RobotDesignation should default to competition if not specified
    if (type.toUpperCase().equals("PRACTICE")) {
      rd = RobotDesignation.PRACTICE;
    } else {
      rd = RobotDesignation.COMPETITION;
    }

    System.out.println();
    System.out.println("_.~\"~._.~\"~._.~\"~._.~\"~._");
    System.out.println();
    System.out.println(type.toUpperCase() + " ROBOT BOOTING");
    System.out.println();
    System.out.println("\"~._.~\"~._.~\"~._.~\"~._.~");
    System.out.println();
    
    return rd;

    // try {
    //   f = new File("/home/lvuser/Robot.txt");
    //   f.createNewFile();
    //   fw = new FileWriter(f);
    // } catch (IOException e) {
    //   // TODO Auto-generated catch block
    //   e.printStackTrace();
    // }
    // bw = new BufferedWriter(fw);

    // try {
    //   bw.write("competition");
    //   bw.close();
    //   fw.close();
    // } catch (IOException e) {
    //   // TODO Auto-generated catch block
    //   e.printStackTrace();
    // }
  }
}
