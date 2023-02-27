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
  private double wrist_low=1; 
  private double wrist_high=2000;
  private double ajoy_delta=0.2;
  private double ajoy_speed=10;
  private double claw_init=800.0; 
  private double wrist_init=300.0; 
  private double elbow_init=320.0; 
  private double shoulder_init=1745; 

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
  private boolean ab1;
  private boolean ab2;
  private boolean ab3;
  private boolean ab4;
  private boolean ab5;
  private boolean ab6;
  private boolean ab11;

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

  // array of ints variable for each position

  // Current button pressed - target state of the arm expressed as a set of angles
  // for each arm

  // Constants to be defined here - move it to the Constants file later
  /*
   * Predefiend positions require a combination of motors to be in a specific
   * positions
   * Neutral
   * Grab cone from the kiosk
   * Drop cone on the tall pole int[]
   * Drop cone on the short pole int[]
   * Drop cube on the tall table int[]
   * Drop cube on the short table int[]
   */

  // Mapping between buttons and predefined positions
  // 4 numbers go in an order. 4 numbers for the values and 4 numbers for the
  // order. We will do 1 at a time

  /** Creates a new Arm. */

  public Arm(Joystick arm_joystick) {
    this.arm_joystick = arm_joystick;
  }

  //  1 for both a and b, 2 for a, 3 for b
  public void ismoveok(double cura, double curb, double wanta, double wantb){
    double delta=Constants.armdelta;
    double deltb=Constants.armdelta;
    if(wanta<cura){delta*=-1;}
    else if(wanta==cura){delta*=0;}

    if(wantb<curb){deltb*=-1;}
    else if(wantb==curb){deltb*=0;}

    if(getx(cura+delta,curb+deltb)<1.21&&gety(cura+delta,curb+deltb)>0){
      servo_W(s3, cura+delta);
      servo_W(s2, curb+deltb);
      return;
    }
    if(getx(cura+delta,curb)<1.21&&gety(cura+delta,curb)>0){
      servo_W(s3, cura+delta);
      servo_W(s2, curb);
      return;
    }
    if(getx(cura,deltb+curb)<1.21&&gety(cura, deltb+curb)>0){
      servo_W(s3, cura);
      servo_W(s2, curb+deltb);
      return;
    }
    return;
  }

  public boolean isok(double a, double b){
    if(getx(a,b)<1.21&&gety(a,b)>0){
      return true;
    }
    return false;
  }

  public double conv_a(double a){
    double temp = ((a-a_zero)/(a_ninety-a_zero)) *(90);
    temp= temp*((2*3.14)/360);
    return temp;
  }
  public double conv_b(double b){
    double temp= ((b-b_zero)/(b_ninety-b_zero)) *(90);
    temp= temp*((2*3.14)/360);
    return temp;
  }

  public double getx(double a, double b){
    double da=conv_a(a);
    double db=conv_b(b);
    return armlens[1]*Math.cos(da)+armlens[2]*Math.cos(db);
  }

  public double gety(double a, double b){
    double da=conv_a(a);
    double db=conv_b(b);
    System.out.println("sin: " +Math.sin(da));
    return armlens[0]+armlens[1]*Math.sin(da)+armlens[2]*Math.sin(db);
  }


  @Override
  public void periodic() {
    
    
    // This method will be called once per scheduler run
    // Did we press a new button? set it in the instance state
    // call the setMotor() on all the correspinding motors in a specfic order
    // handle the state machine logic also here
    ajx=arm_joystick.getX();
    ajy=arm_joystick.getY();
    ajz=arm_joystick.getRawAxis(3);
    ajt=arm_joystick.getRawAxis(2);
    ab1=arm_joystick.getRawButton(1);
    ab2=arm_joystick.getRawButton(2);
    ab3=arm_joystick.getRawButton(3);
    ab4=arm_joystick.getRawButton(4);
    ab11=arm_joystick.getRawButton(11);
    
    arm_pos();
    // looking at difference from joystick so if joystick moves a tiny bit, nothing will happen
    if (Math.abs(ajy) > ajoy_delta) //ajoy_delta is value that is compared with  
    {
    if (elbow_current >= elbow_low) //checks if its in limit
    {if (elbow_current <= elbow_high) //checks if its in limit
      {elbow_current=elbow_current+ajy*ajoy_speed;} //changes position
      else {elbow_current=elbow_high;}} //limit 
    else {elbow_current=elbow_low;} //limit
    
    }
    
    //same thing with shoulder, claw, and wrist....
    if (Math.abs(ajx) >ajoy_delta) 
    {
      if (shoulder_current >= shoulder_low) 
      {if (shoulder_current <= shoulder_high)
        {shoulder_current=shoulder_current+ajx*ajoy_speed;}
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
      wrist_current=1476;
      elbow_current=542;
      shoulder_current=38;

    }

    if (ab4){//resting
        wrist_current=1476;
        shoulder_current=2000;
        elbow_current=1600;//318

    }
    if (ab1){
      claw_current=1100;
    }
    else{claw_current=500;}

    if (ab11){
      // values: 300, 1469
      if (wrist_current > 1800){
        wrist_current = 300;
      }
      else{
        wrist_current = 1469;
      }
    }
    set_servos(claw_current,wrist_current,elbow_current,shoulder_current);
    
    //System.out.println(" claw="+claw_current+" ajz="+ajz);
    /*if(waitcount==20){
      ismoveok(shoulder_current, elbow_current, 0,0);
      waitcount=0;
    }*/
    System.out.println("apwm: "+shoulder_current+" bpwm: "+ elbow_current);
    System.out.println("a: "+conv_a(shoulder_current)+" b: "+ conv_b(elbow_current));
    System.out.println("x: "+getx(shoulder_current,elbow_current)+" y: "+gety(shoulder_current,elbow_current));
  }

  public void neutral() {
    set_servos(claw_init,wrist_init,elbow_init,shoulder_init);
  }
 
  public void servo_W(PWM s, final Double pwm_v) {
    s.setBounds(5000, 0, 0, 0, 0);
    s.setPeriodMultiplier(PWM.PeriodMultiplier.k4X);
    //System.out.println(pwm_v+ " pos "+s.getPosition());
    s.setRaw(pwm_v.intValue());
    //System.out.println(pwm_v.intValue());
 
  }


  public void set_servos(double claw_set, double wrist_set, double elbow_set, double shoulder_set ) {
    System.out.print(claw_set);
    servo_W(s0, claw_set); //claw_init
    servo_W(s1, wrist_set); //wrist_init 
    servo_W(s2, elbow_set); //elbow joint 
    servo_W(s3, shoulder_set); //shoulder_init
  }

}