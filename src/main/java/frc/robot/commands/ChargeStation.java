// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class ChargeStation extends CommandBase {
  Drivetrain drivetrain;
  double LRoll;
  double cold;
  double SYaw;

  /** Creates a new ChargeStation. */
  public ChargeStation(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {LRoll=drivetrain.gyroGetRoll();cold=1;SYaw=drivetrain.gyroGetYaw();}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double e=drivetrain.gyroGetRoll()-LRoll;
    if(Math.abs(e)>Constants.ChargeStationConstants.climbDeg){cold=0;}
    double p = Constants.ChargeStationConstants.gain*e;
    p=(p>Constants.ChargeStationConstants.clip)?Constants.ChargeStationConstants.clip:((p<-Constants.ChargeStationConstants.clip)?-Constants.ChargeStationConstants.clip:p);
    if(cold==1) p=Constants.ChargeStationConstants.start;
    double lp; 
    double rp;
    lp = p+(Constants.ChargeStationConstants.lrgain*(drivetrain.gyroGetYaw()-SYaw));
    rp = p-(Constants.ChargeStationConstants.lrgain*(drivetrain.gyroGetYaw()-SYaw));
    if(cold==1){drivetrain.RobotLRMove(90, lp, rp);}
    else{
      if(p<0){drivetrain.RobotLRMove(270, -lp, -rp);}
      else{drivetrain.RobotLRMove(90, lp, rp);}}
  }
  @Override
  public void end(boolean interrupted) {}
  @Override
  public boolean isFinished() {return false;}
}