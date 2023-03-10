// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class ChargeStation extends CommandBase {
  Drivetrain drivetrain;
  double lPitch;
  double cold;

  /** Creates a new ChargeStation. */
  public ChargeStation(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {lPitch=drivetrain.gyroGetPitch();cold=1;}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double p;
    double e;
    double t;
    e=-(drivetrain.gyroGetPitch())-lPitch;
    if(Math.abs(e)>Constants.ChargeStationConstants.climbDeg){cold=0;}
    p = Constants.ChargeStationConstants.gain*e;
    p=(p>Constants.ChargeStationConstants.clip)?Constants.ChargeStationConstants.clip:((p<-Constants.ChargeStationConstants.clip)?-Constants.ChargeStationConstants.clip:p);
    if((drivetrain.getTEncoderPostion(0)<Constants.ChargeStationConstants.angleClip)&&(drivetrain.getTEncoderPostion(1)<Constants.ChargeStationConstants.angleClip)&&(drivetrain.getTEncoderPostion(2)<Constants.ChargeStationConstants.angleClip)&&(drivetrain.getTEncoderPostion(3)<Constants.ChargeStationConstants.angleClip)){
      for(int i=0;i<8;i+=2){
      if(cold==1){drivetrain.setSpeed(Constants.ChargeStationConstants.start, i);}
      else{drivetrain.setSpeed(p, i);}
      }
    }
    else{
      for(int i=0;i<8;i+=2){
        if(drivetrain.getTEncoderPostion((i/2)+1)>Constants.ChargeStationConstants.angleClip){
          t=Constants.ChargeStationConstants.gain*drivetrain.getTEncoderPostion((i/2)+1);
          t=(t>Constants.mT)?Constants.mT:((t<-Constants.mT)?-Constants.mT:t);
          drivetrain.setSpeed(t, i+1);
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
