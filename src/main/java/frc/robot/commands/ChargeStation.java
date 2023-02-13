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
    double a1,a2,a3,a4;
    a1=drivetrain.getTEncoderPostion(0);
    a2=drivetrain.getTEncoderPostion(1);
    a3=drivetrain.getTEncoderPostion(2);
    a4=drivetrain.getTEncoderPostion(3);
    if (a1>180) a1=360-a1;
    if (a2>180) a2=360-a2;
    if (a3>180) a3=360-a3;
    if (a4>180) a4=360-a4;
    if((a1<Constants.ChargeStationConstants.angleClip)&&(a2<Constants.ChargeStationConstants.angleClip)&&(a3<Constants.ChargeStationConstants.angleClip)&&(a4<Constants.ChargeStationConstants.angleClip)){
    for(int i=0;i<8;i+=2){
      drivetrain.setSpeed(0, i+1);
      if(cold==1){drivetrain.setSpeed(Constants.ChargeStationConstants.start, i);}
      else{drivetrain.setSpeed(-p, i);}
      }
    }
    else{
      for(int i=0;i<8;i+=2){
        drivetrain.setSpeed(0, i);
        double a;
        a=drivetrain.getTEncoderPostion(i/2);
        if(a>180) a-=360;
          if(Math.abs(a)>Constants.ChargeStationConstants.angleClip){
            t=Constants.ChargeStationConstants.tgain*a;
            t=(t>Constants.mT)?Constants.mT:((t<-Constants.mT)?-Constants.mT:t);
            drivetrain.setSpeed(t, i+1);
        }else{drivetrain.setSpeed(0, i+1);}
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
