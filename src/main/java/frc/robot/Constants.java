// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double angleThresh = 4;
  public static final double dPowerMin = 0.06;//min drive power
  public static final double rT = 0.1;//rotate thresh
  public static final double tF = 0.8;//turn factor
  public static final double dF = 0.5;//drive factor
  public static final double tremp=0.1;//turn ramp
  public static final double dremp=0.5;//drive ramp
  public static final double fremp=0.2;//falling ramp
  public static final double mT = 0.4;//max turn speed
  public static final double twF = 0.6;//twist factor
  public static final double turnInProgress = 20;
  public static final int MacroTime = 500;
  public static final int mDeviceID[] = {1,2,3,4,5,6,7,8};//even motors(ID) are turn
  public static final int tDeviceID[] = {21,23,24,22};//absolute encoder device ids
  public static final double tOffset[] = {0.97, 85.61,218.057,211.03};//turn motor offsets
  public static final int gDeviceID = 11;//gyro device ID
  public static final double armLengths[] = {80, 58, 53.3};//0 is base to shouler, 1 is shoulder to elbow, 2 is elbow to claw, this is all in centimeters
  public static final double heightTopInch = 75.0; // Height of the retro-reflective tape on the top peg
  public static final double heightLimeInch = 24.0; // Height of the limelight camera (To be changed later)
  public static final double angleLimeDegrees = 25.0; // The camera's YAngle (tilted up) (To be changed later)
  public static final double limelightPlacementOffsetInch = 10.0; // Offset from limelight placement and the front of the robot (To be changed later)
  public static final double topPegDistanceDeductInch = 45.75; //6in added to account for bumper
  public static final double bottomPegDistanceDeductInch = 28.75; //6in added to account for bumper
  public static final double LimeLightAimAssistG = 2;
  public static final double armdelta = 1;
  public static class ChargeStationConstants {
    //add these to the dashboard
    public static final double gain = 0.025;
    public static final double tgain = 0.007;
    public static final double clip = 0.5;
    public static final double start = 0.23;
    public static final double climbDeg = 10;
    public static final double angleClip = 5;
    public static final double lrgain = 0.02;
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}