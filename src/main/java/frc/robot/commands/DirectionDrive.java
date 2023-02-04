// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DirectionDrive extends CommandBase {
  Drivetrain drivetrain;
  double angle;
  double speed;
  double distance;
  double tPower;
  boolean c = false;
  double cAngle;
  double dAngle;
  double dir;
  /** Creates a new DirectionDrive. */
  public DirectionDrive(Drivetrain drivetrain, double angle, double speed, double distance){
    this.drivetrain = drivetrain;
    this.angle=angle;
    this.speed=speed;
    this.distance=distance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize(){drivetrain.resetDEncoders();}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    for(int i=1;i<8;i+=2){
      cAngle=drivetrain.getTEncoderPostion((i-1)/2);
      if(angle-cAngle>90){angle=Math.abs(angle-180);speed*=-1;}
      double[] deltaM = drivetrain.deltaMod(angle, cAngle);
      dAngle=deltaM[0];
      dir=deltaM[1];
      tPower=Constants.tF*dAngle/180;
      if(Math.abs(tPower)>Constants.mT){tPower=Constants.mT*tPower/Math.abs(tPower);drivetrain.setSpeed(tPower, i);}
      if(Math.abs(dAngle)>Constants.turnInProgress){c= false;continue;}
      if(drivetrain.getDEncoderPosition(i-1)<distance){drivetrain.setSpeed(Constants.dF*speed*dir, i-1); c=false;}
      else{c=true;}
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return c;
  }
}
