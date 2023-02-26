// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  private WPI_TalonSRX t[] = new WPI_TalonSRX[4];//encoders
  private CANSparkMax m[] = new CANSparkMax[8];//motors
  private RelativeEncoder e[] = new RelativeEncoder[4];//drive encoders
  private Pigeon2 gyro;
  private Joystick joystick;
  public Drivetrain(Joystick joystick) {
    this.joystick = joystick;
    gyro = new Pigeon2(Constants.gDeviceID, "rio");
    for(int i=0;i<8;i++){
      m[i] = new CANSparkMax(Constants.mDeviceID[i], MotorType.kBrushless);
      m[i].restoreFactoryDefaults();
      m[i].setIdleMode(IdleMode.kBrake);
      if(i%2==0){
      t[i/2]=new WPI_TalonSRX(Constants.tDeviceID[i/2]);
      t[i/2].configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
      m[i].setOpenLoopRampRate(Constants.dremp);
      e[i/2] = m[i].getEncoder();
      e[i/2].setPositionConversionFactor(1/20);}
      else{m[i].setOpenLoopRampRate(Constants.tremp);}
    }
  }
  public double[] deltaMod(double t, double c){
    double d = t-c;
    double dir;
    d = (Math.abs(d)>=180)?-(((360*d)/(Math.abs(d)))-d):d;
    if(Math.abs(d)>=90) {d=-(((180*d)/(Math.abs(d)))-d);dir=-1;}
    else{dir= 1;}
    d=(Math.abs(d)<Constants.angleThresh)?0:d;
    return new double[] {-d, dir};
  }
  public void resetDEncoders(){
    e[0].setPosition(0);
    e[1].setPosition(0);
    e[2].setPosition(0);
    e[3].setPosition(0);
  }
  public double getTEncoderPostion(int tEncNum){return (((t[tEncNum].getSelectedSensorPosition())*360/4096)+(Constants.tOffset[(tEncNum)]))%360;}
  public double getTEncoderPostionGyro(int tEncNum){return (((t[tEncNum].getSelectedSensorPosition())*360/4096)+(Constants.tOffset[(tEncNum)])+(gyro.getYaw()))%360;}
  public double getDEncoderPosition(int dEncNum){return e[dEncNum].getPosition();}
  public void setSpeed(double speed, int motor){m[motor].set(speed);}
  public void gyroPutPitch(){SmartDashboard.putNumber("Pitch", gyro.getPitch());}
  public void gyroPutRoll(){SmartDashboard.putNumber("Roll", gyro.getRoll());}
  public void gyroPutYaw(){SmartDashboard.putNumber("Yaw", gyro.getYaw());}
  public double gyroGetPitch(){return gyro.getPitch();}
  public double gyroGetRoll(){return gyro.getRoll();}
  public double gyroGetYaw(){return gyro.getYaw();}
  public void gyroSetYaw(double angle){gyro.setYaw(angle);}
  public void stopMotors(){
    for(int i=0;i<8;i++){
      m[i].setOpenLoopRampRate(Constants.fremp);
      m[i].stopMotor();
      if(i%2==0){m[i].setOpenLoopRampRate(Constants.dremp);}
      else{m[i].setOpenLoopRampRate(Constants.tremp);}
    }
  }
  public double[] cTp(double x, double y){
    double magnitude = Math.sqrt(((x * x) + (y * y))/2);
    double degrees = Math.toDegrees(Math.atan2(y, x));
    if (degrees < 0) {degrees = 360 + degrees;}
    return new double[] {degrees, magnitude};
  }
  public void RobotMove(double tAngle, double dPower, double jX, boolean trigger, boolean b2){
    double fdF;
    double dAngle;
    double cAngle;
    double dir;
    fdF = (trigger)?Constants.dF*2:Constants.dF;
    for(int i=1;i<8;i+=2){
      cAngle=getTEncoderPostionGyro((i-1)/2);
      if(b2){
        if(jX<0){cAngle=getTEncoderPostion((i-1)/2);dPower = (Constants.twF*(Math.abs(jX)));
          if(i==1){tAngle=45;}if(i==3){tAngle=135;}if(i==5){tAngle=225;}if(i==7){tAngle=315;}}
        else if(jX>0){cAngle=getTEncoderPostion((i-1)/2);dPower = (Constants.twF*(Math.abs(jX)));
          if(i==1){tAngle=225;}if(i==3){tAngle=315;}if(i==5){tAngle=45;}if(i==7){tAngle=135;}}}
      double[] deltaM = deltaMod(tAngle, cAngle);
      dAngle=deltaM[0];
      dir=deltaM[1];
      if(dPower<Constants.dPowerMin){dAngle = 0;dPower = 0;}
      double tPower=Constants.tF*dAngle/180;
      if(Math.abs(tPower)>Constants.mT){tPower=Constants.mT*tPower/Math.abs(tPower);}
      setSpeed(tPower, i);
      if(Math.abs(dAngle)<Constants.turnInProgress){setSpeed(fdF*dPower*dir, i-1);}
      else{setSpeed(0, i-1);}
    }
  }
  public void RobotLRMove(double tAngle, double ldPower, double rdPower){
    double dAngle;
    double cAngle;
    double dir;
    for(int i=1;i<8;i+=2){
      cAngle=getTEncoderPostion((i-1)/2);
      double[] deltaM = deltaMod(tAngle, cAngle);
      dAngle=deltaM[0];
      dir=deltaM[1];
      if(rdPower<Constants.dPowerMin){dAngle = 0;rdPower = 0;}
      if(ldPower<Constants.dPowerMin){dAngle = 0;ldPower = 0;}
      double tPower=Constants.tF*dAngle/180;
      if(Math.abs(tPower)>Constants.mT){tPower=Constants.mT*tPower/Math.abs(tPower);}
      setSpeed(tPower, i);
      if((i==3)||(i==1)){
        if(Math.abs(dAngle)<Constants.turnInProgress){setSpeed(Constants.dF*rdPower*dir, i-1);}
        else{setSpeed(0, i-1);}
      }else{
        if(Math.abs(dAngle)<Constants.turnInProgress){setSpeed(Constants.dF*ldPower*dir, i-1);}
        else{setSpeed(0, i-1);}
      }
    }
  }
  public void CANtest(){
    if(joystick.getRawButton(5)){setSpeed(0.1, 0);}
    else{setSpeed(0, 0);}
    if(joystick.getRawButton(6)){setSpeed(0.1, 1);System.out.println("TurnMotor 1 encoder:"+getTEncoderPostion(0));}
    else{setSpeed(0, 1);}
    if(joystick.getRawButton(7)){setSpeed(0.1, 2);}
    else{setSpeed(0, 2);}
    if(joystick.getRawButton(8)){setSpeed(0.1, 3);System.out.println("TurnMotor 2 encoder:"+getTEncoderPostion(1));}
    else{setSpeed(0, 3);}
    if(joystick.getRawButton(9)){setSpeed(0.1, 4);}
    else{setSpeed(0, 4);}
    if(joystick.getRawButton(10)){setSpeed(0.1, 5);System.out.println("TurnMotor 3 encoder:"+getTEncoderPostion(2));}
    else{setSpeed(0, 5);}
    if(joystick.getRawButton(11)){setSpeed(0.1, 6);}
    else{setSpeed(0, 6);}
    if(joystick.getRawButton(12)){setSpeed(0.1, 7);System.out.println("TurnMotor 4 encoder:"+getTEncoderPostion(3));}
    else{setSpeed(0, 7);}
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}