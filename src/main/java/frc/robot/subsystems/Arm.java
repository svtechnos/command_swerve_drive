// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.Math;
import frc.robot.Constants;


public class Arm extends SubsystemBase {
  // Only one instance of this class will be created per robot

  // Initaite the joystick here
  Joystick arm_joystick; // different joystick
  PS4Controller examplePS4 = new PS4Controller(0); // 0 is the USB Port to be used as indicated on the Driver Station
  XboxController exampleXbox = new XboxController(0); // 0 is the USB Port to be used as indicated on the Driver Station
  
  // limits 
  private double elbow_low=320; 
  private double elbow_high=1600;
  private double shoulder_low=40; 
  private double shoulder_high=2000;
  private double claw_low=780; 
  private double claw_high=1450;
  private double wrist_low=500; 
  private double wrist_high=1800;
  private double ajoy_delta=0.2;
  private double ajoy_speed=10;
  private double claw_init=250; 
  private double wrist_init=498; 
  private double elbow_init=1600; 
  private double shoulder_init=2000; 
  private int preset_locktime_max=20;

  // initialization 
  private double claw_current=claw_init; 
  private double wrist_current=wrist_init; 
  private double elbow_current=elbow_init; 
  private double shoulder_current=shoulder_init; 
  private double a_zero=773.0; 
  private double a_ninety=1731;
  private double b_ninety=1447;
  private double b_zero=940.1;
  private double claw;
  private double wrist;
  private double elbow;
  private double shoulder;

  private double ajx;
  private double ajy;
  private double ajz;
  private double ajt;
  private double ajf;
  private double ajh;
  private double ajv;
  private boolean ab1;
  private boolean ab2;
  private boolean ab3;
  private boolean ab4;
  private boolean ab5;
  private boolean ab6;
  private boolean ab7;
  private boolean ab8;
  private boolean ab9;
  private boolean ab10;
  private boolean ab11;
  private boolean preset_triggerd;
  private long loop_n=0;
  private boolean lock_for_preset=false;
  private int preset_locktime=0;
  private boolean sp_hori_mode=false;
  private boolean sp_vert_mode=false ;
  private String mode="normal";
  





  private int waitcount=0;
  private double claw_a;
  private double claw_b;
  private double[] armlens={0.8,0.66,0.533};
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  /*NetworkTableEntry FTape = table.pipeline(0);
  NetworkTableEntry FTagR = table.pipeline(1);
  NetworkTableEntry FtagB = table.pipeline(2);
  table.getEntry("pipeline").setNumber(<pipeline_index>);
  */

  //servos
  PWM s0=new PWM(0);
  PWM s1=new PWM(1);
  PWM s2=new PWM(2);
  PWM s3=new PWM(3);
  // putting number in dashboard
  public void arm_pos(){
    SmartDashboard.putNumber("DB/Slider 0", elbow_current);
    SmartDashboard.putNumber("DB/Slider 1", shoulder_current);
    SmartDashboard.putNumber("DB/Slider 2", wrist_current);
    SmartDashboard.putNumber("DB/Slider 3", claw_current);
  
  }
  
  // Initialize motors here

  public Arm(Joystick arm_joystick) {
    this.arm_joystick = arm_joystick;
  }




  @Override
  public void periodic() {
  
    // This method will be called once per scheduler run
    // Did we press a new button? set it in the instance state
    // call the setMotor() on all the correspinding motors in a specfic order
    // handle the state machine logic also here
    loop_n=loop_n+1;
    if (lock_for_preset) {
      preset_locktime=preset_locktime-1;
      if (preset_locktime==0) {
        lock_for_preset=false;
      }
    }
    else {
      preset_triggerd=check_joystick();
      if (preset_triggerd) {
        lock_for_preset=true;
        preset_locktime=preset_locktime_max;
      }
    }
    
    if (preset_triggerd) {

      System.out.println(" loop "+loop_n);}

    if (lock_for_preset) {

        System.out.println("plt "+preset_locktime);}

    
    arm_pos();
    // looking at difference from joystick so if joystick moves a tiny bit, nothing will happen
    if (Math.abs(ajx) > ajoy_delta) //ajoy_delta is value that is compared with  
    {
    if (elbow_current >= elbow_low) //checks if its in limit
    {if (elbow_current <= elbow_high) //checks if its in limit
      {elbow_current=elbow_current+ajx*ajoy_speed;} //changes position
      else {elbow_current=elbow_high;}} //limit 
    else {elbow_current=elbow_low;} //limit
    
    }
    
    //same thing with shoulder, claw, and wrist....
    if (Math.abs(ajy) >ajoy_delta) 
    {
      if (shoulder_current >= shoulder_low) 
      {if (shoulder_current <= shoulder_high)
        {shoulder_current=shoulder_current+ajy*ajoy_speed;}
        else {shoulder_current=shoulder_high;}}
      else {shoulder_current=shoulder_low;}
      
      }
      if (Math.abs(ajt) >ajoy_delta)
      {
        if (wrist_current >= wrist_low) 
        {if (wrist_current <= wrist_high)
          {wrist_current=wrist_current+ajt*ajoy_speed;}
          else {wrist_current=wrist_high;}}
        else {wrist_current=wrist_low;}
        }  
      
        // calculate based on min/max
      claw_b=(claw_low+claw_high)/2;
      claw_a=claw_b-claw_low;
      //claw_current=(ajz)*claw_a+claw_b;


   
    
    if (ab3){
      mode="hmode";
      wrist_current=500;
      elbow_current=557;
      shoulder_current=38;

    }
    if (ab4){
      mode="vmode";
      shoulder_current=479;

    }
    if (ab5){//restingv
      mode="normal";
        if (preset_locktime <1) {
          elbow_current=320;
        }
        else {
        shoulder_current=1700;
        }
        

    }
    if (ab6){//restingv
      mode="vmode";
        shoulder_current=1991;
        }
        
    if (ab9){//restingv
          mode="vmode";
            shoulder_current=1991;
            }
  
    if (ab1){
      claw_current=1100;
    }
    else{claw_current=250;}

    if (ab11){
      // values: 300, 1469
      if (wrist_current > 1700){
        wrist_current = 500;
      }
      else{
        wrist_current = 1800;
      }
    }
   
    //  else{mode="normal";}

    if (ab7){
      mode="hmode";
    }
    if (ab8){
      mode="vmode";
    }
    if (ab2){
      mode="normal";
    }
     System.out.println("ajh "+ ajh+" ajv"+ajv+"mode "+ mode);
if (mode=="hmode"){
  elbow_current=shoulder_current*0.554+490;
}
if (mode=="vmode"){
  if (shoulder_current>1023){
    elbow_current=1600;
  }
  else {
    elbow_current=shoulder_current*0.554+1035;
  }
  
}

    set_servos(claw_current,wrist_current,elbow_current,shoulder_current);
    
  }

  public void neutral() {
    set_servos(claw_init,wrist_init,elbow_init,shoulder_init);
  }
 
  public boolean check_joystick() {

    ajx=arm_joystick.getX();
    ajy=arm_joystick.getY();
    ajz=arm_joystick.getRawAxis(3);
    ajt=arm_joystick.getRawAxis(2);
    ajf=arm_joystick.getRawAxis(4);
    ajh=arm_joystick.getRawAxis(5);
    ajv=arm_joystick.getRawAxis(6);
    ab1=arm_joystick.getRawButton(1);
    ab2=arm_joystick.getRawButton(2);
    ab3=arm_joystick.getRawButton(3);
    ab4=arm_joystick.getRawButton(4);
    ab5=arm_joystick.getRawButton(5);
    ab6=arm_joystick.getRawButton(6);
    ab7=arm_joystick.getRawButton(7);
    ab8=arm_joystick.getRawButton(8);
    ab9=arm_joystick.getRawButton(9);
    ab10=arm_joystick.getRawButton(10);
    ab11=arm_joystick.getRawButton(11);
    return ((ab1) || (ab2) || (ab3) || (ab4) ||(ab5)||(ab6)||(ab7)||(ab8)||(ab9)||(ab10)|| (ab11) ); }
    
  
  
  public void servo_W(PWM s, final Double pwm_v) {
    s.setBounds(5000, 0, 0, 0, 0);
    s.setPeriodMultiplier(PWM.PeriodMultiplier.k4X);
    //System.out.println(pwm_v+ " pos "+s.getPosition());
    s.setRaw(pwm_v.intValue());
    //System.out.println(pwm_v.intValue());
 
  }


  public void set_servos(double claw_set, double wrist_set, double elbow_set, double shoulder_set ) {
    //System.out.print(claw_set);
    servo_W(s0, claw_set); //claw_init
    servo_W(s1, wrist_set); //wrist_init 
    servo_W(s2, elbow_set); //elbow joint 
    servo_W(s3, shoulder_set); //shoulder_init
  }

}