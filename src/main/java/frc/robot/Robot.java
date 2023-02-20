// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in th
 * e TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  //private final Drivetrain drivetrain = new Drivetrain();
  private RobotContainer m_robotContainer;
  Drivetrain drivetrain;
  Joystick joystick;
  public double jxArray[] = new double[750];
  public double tAngleArray[] = new double[750];
  public double dPowerArray[] = new double[750];
  public boolean triggerArray[] = new boolean[750];
  public boolean b2Array[] = new boolean[750];
  public int idx = 750;
  public int idxr = 750;
  public int flag;
  //private RobotContainer m_robotContainer;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_robotContainer=new RobotContainer();
    this.drivetrain = m_robotContainer.drivetrain;
    this.joystick = m_robotContainer.joystick;
    drivetrain.resetDEncoders();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    drivetrain.gyroPutPitch();
    drivetrain.gyroPutRoll();
    drivetrain.gyroPutYaw();
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
    drivetrain.resetDEncoders();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      }else{if(flag==1){idxr=0;flag=0;}}
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double jX=0;
    double tAngle=0;
    double dPower=0;
    boolean b2=false;
    boolean trigger=false;
    if(idxr<750){jX=jxArray[idxr];tAngle=tAngleArray[idxr];dPower=dPowerArray[idxr];trigger=triggerArray[idxr];b2=b2Array[idxr];idxr++;System.out.println(idxr);}
    drivetrain.RobotMove(tAngle, dPower, jX, trigger, b2);
  }
  @Override
  public void teleopInit() {
    drivetrain.gyroSetYaw(0);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double jX;
    double jY;
    double tAngle;
    double dPower;
    boolean b2;
    boolean trigger;
    if(joystick.getRawButton(13)){drivetrain.CANtest();}
    else{
      jX=joystick.getX();
      jY=joystick.getY()*-1;
      b2=joystick.getRawButton(2);
      trigger=joystick.getTrigger();
      double[] cTpResult = drivetrain.cTp(jX, jY);
      tAngle=cTpResult[0];
      dPower=cTpResult[1];
      if(joystick.getRawButton(11)){idx=0;}
      if(idx==749){flag=1;}
      if(idx<750){jxArray[idx]=jX;tAngleArray[idx]=tAngle;dPowerArray[idx]=dPower;triggerArray[idx]=trigger;b2Array[idx]=b2;idx++;System.out.println(idx);}
      drivetrain.RobotMove(tAngle, dPower, jX, trigger, b2);
    }
  }
  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
