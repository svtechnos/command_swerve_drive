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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  private static final int mDeviceID[] = {1,2,3,4,5,6,7,8};//even motors(ID) are turn
  private static final int tDeviceID[] = {21,23,24,22};//encoder device ids
  private static final double tOffset[] = {90.97, 175.61,308.057,301.03};//turn motor offsets
  private static final int gDeviceID = 11;
  private WPI_TalonSRX t[] = new WPI_TalonSRX[4];//encoders
  private CANSparkMax m[] = new CANSparkMax[8];//motors
  private RelativeEncoder e[] = new RelativeEncoder[4];//drive encoders
  private Pigeon2 gyro;
  public Drivetrain() {
    gyro = new Pigeon2(gDeviceID, "rio");
    for(int i=0;i<8;i++){
      m[i] = new CANSparkMax(mDeviceID[i], MotorType.kBrushless);
      m[i].restoreFactoryDefaults();
      m[i].setIdleMode(IdleMode.kBrake);
      if(i%2==0){
      t[i/2]=new WPI_TalonSRX(tDeviceID[i/2]);
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
    //System.out.println("c: "+c+" t: "+t+" d: "+d+" dir: "+dir);
    return new double[] {-d, dir};
  }
  public void resetDEncoders(){
    e[0].setPosition(0);
    e[1].setPosition(0);
    e[2].setPosition(0);
    e[3].setPosition(0);
  }
  public double getTEncoderPostion(int tEncNum){return (((t[tEncNum].getSelectedSensorPosition())*360/4096)+(tOffset[(tEncNum)]))%360;}
  public double getTEncoderPostionGyro(int tEncNum){return (((t[tEncNum].getSelectedSensorPosition())*360/4096)+(tOffset[(tEncNum)])+(gyro.getYaw()))%360;}
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
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
