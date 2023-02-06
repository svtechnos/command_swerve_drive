// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DirectionDrive extends CommandBase {
  Drivetrain drivetrain;
  double tAngle;
  double speed;
  double distance;
  double tPower;
  double cAngle;
  double dAngle;
  double dir;
  /** Creates a new DirectionDrive. */
  public DirectionDrive(Drivetrain drivetrain, double tAngle, double speed, double distance){
    this.drivetrain = drivetrain;
    this.tAngle=tAngle;
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
      System.out.println("DriveEncoder 0: "+drivetrain.getDEncoderPosition(0));
      double[] deltaM = drivetrain.deltaMod(tAngle, cAngle);
      dAngle=deltaM[0];
      dir=deltaM[1];
      double tPower=Constants.tF*dAngle/180;
      if(Math.abs(tPower)>Constants.mT){tPower=Constants.mT*tPower/Math.abs(tPower);}
      drivetrain.setSpeed(tPower, i);
      if(Math.abs(dAngle)<Constants.turnInProgress){drivetrain.setSpeed(Constants.dF*speed*dir, i-1);}
      else{drivetrain.setSpeed(0, i-1);}
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {drivetrain.stopMotors();drivetrain.resetDEncoders();}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetrain.getDEncoderPosition(0)>distance;
  }
}
