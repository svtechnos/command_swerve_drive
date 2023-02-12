// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class MacroRecord extends CommandBase {
  private Joystick joystick;
  private Drivetrain drivetrain;
  private Timer timer;
  private double jX;
  private double jY;
  private double dPower;
  private double dAngle;
  private double cAngle;
  private double tAngle;
  private double dir;
  private double fdF;
  private double duration;
  private boolean jTrig;
  private boolean jTwo;
  /** Creates a new MacroRecord. */
  public MacroRecord(double duration, Drivetrain drivetrain, Joystick joystick) {
    this.joystick = joystick;
    this.duration=duration;
    this.drivetrain=drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    jX = joystick.getX();
    jY = joystick.getY()*-1;
    jTrig = joystick.getTrigger();
    jTwo = joystick.getRawButton(2);
    System.out.println(jX+" "+jY+" "+jTrig+" "+jTwo);
    fdF = (joystick.getTrigger())?Constants.dF*2:Constants.dF;
    double[] cTpResult = drivetrain.cTp(jX, jY);
    tAngle=cTpResult[0];
    dPower=cTpResult[1];
    for(int i=1;i<8;i+=2){
      cAngle=drivetrain.getTEncoderPostionGyro((i-1)/2);
      if(joystick.getRawButton(2)){
        if(jX<0){cAngle=drivetrain.getTEncoderPostion((i-1)/2);dPower = (Constants.twF*(Math.abs(jX)));
          if(i==1){tAngle=315;}if(i==3){tAngle=45;}if(i==5){tAngle=135;}if(i==7){tAngle=225;}}
        else if(jX>0){cAngle=drivetrain.getTEncoderPostion((i-1)/2);dPower = (Constants.twF*(Math.abs(jX)));
          if(i==1){tAngle=135;}if(i==3){tAngle=225;}if(i==5){tAngle=315;}if(i==7){tAngle=45;}}}
      double[] deltaM = drivetrain.deltaMod(tAngle, cAngle);
      dAngle=deltaM[0];
      dir=deltaM[1];
      if(dPower<Constants.dPowerMin){dAngle = 0;dPower = 0;}      
      double tPower=Constants.tF*dAngle/180;
      if(Math.abs(tPower)>Constants.mT){tPower=Constants.mT*tPower/Math.abs(tPower);}
      drivetrain.setSpeed(tPower, i);
      if(Math.abs(dAngle)<Constants.turnInProgress){drivetrain.setSpeed(fdF*dPower*dir, i-1);}
      else{drivetrain.setSpeed(0, i-1);}
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
