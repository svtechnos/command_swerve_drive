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


public class Arm extends SubsystemBase {
  // Only one instance of this class will be created per robot

  // Initaite the joystick here
  Joystick armJoystick;
  PS4Controller examplePS4 = new PS4Controller(0); // 0 is the USB Port to be used as indicated on the Driver Station
  XboxController exampleXbox = new XboxController(0); // 0 is the USB Port to be used as indicated on the Driver Station
  private double claw_init=1450.0; 
  private double wrist_init=265.08; 
  private double elbow_init=447.7; 
  private double shoulder_init=10.0; 
  private double a_zero=773.0;
  private double a_ninety=1731;
  private double b_ninety=940.1;
  private double b_zero=1447;
  private double claw;
  private double wrist;
  private double elbow;
  private double shoulder;
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

  public Arm(Joystick armJoystick) {
    this.armJoystick = armJoystick;
  }

  /*  1 for both a and b, 2 for a, 3 for b
  public int ismoveok(double cura, double curb, double wanta, double wantb){
    double delta=Constants.armdelta;
    double deltb=Constants.armdelta;
    if(wanta<cura){delta*=-1;}
    else if(wanta==cura){delta*=0;}

    if(wantb<curb){deltb*=-1;}
    else if(wantb==curb){deltb*=0;}

    if(getx(cura+delta,curb+deltb)<1.21&&gety(cura+delta,curb+deltb)>0){
      return 1;
    }
    if(getx(cura+delta,curb)<1.21&&gety(cura+delta,curb)>0){
      return 2;
    }
    return 3;
  }*/
  public boolean isok(double a, double b){
    if(getx(a,b)<1.21&&gety(a,b)>0){
      return true;
    }
    return false;
  }

  public double conv_a(double a){
    return (a-a_zero)*((a_ninety-a_zero)*90)/360;
  }
  public double conv_b(double b){
    return (b-b_zero)*((b_ninety-b_zero)*90)/360;
  }

  public double getx(double a, double b){
    a=conv_a(a);
    b=conv_b(b);
    return armlens[1]*Math.cos(a)+armlens[2]*Math.cos(b);
  }

  public double gety(double a, double b){
    a=conv_a(a);
    b=conv_b(b);
    return armlens[0]+armlens[1]*Math.sin(a)+armlens[2]*Math.sin(b);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Did we press a new button? set it in the instance state
    // call the setMotor() on all the correspinding motors in a specfic order
    // handle the state machine logic also here
    elbow = SmartDashboard.getNumber("DB/Slider 0", elbow_init);
    claw = SmartDashboard.getNumber("DB/Slider 1", claw_init);
    wrist = SmartDashboard.getNumber("DB/Slider 2", wrist_init);
    shoulder = SmartDashboard.getNumber("DB/Slider 3", shoulder_init);

    System.out.println("x: "+getx(shoulder, elbow)+" y: "+gety(shoulder, elbow));
    //System.out.println(elbow_init);
    /*XboxController xboxController = new XboxController(0);
    double leftX = xboxController.getRawAxis(0);
    double leftY = xboxController.getRawAxis(1);
    double rightX = xboxController.getRawAxis(4);
    double rightY = xboxController.getRawAxis(5);
    */
    servo_W(s0, claw); //claw_init
    servo_W(s1, wrist); //wrist_init 
    servo_W(s2, elbow); //elbow joint 
    servo_W(s3, shoulder); //shoulder_init
    System.out.println("wrist="+wrist+" elbow="+elbow+" shoulder="+shoulder+" claw="+claw);

  }

  public void neutral() {
    servo_W(s0, claw_init); //claw_init
    servo_W(s1, wrist_init); //wrist_init 
    servo_W(s2, elbow_init); //elbow joint 
    servo_W(s3, shoulder_init); //shoulder_init
  }

  
  public void servo_W(PWM s, final Double pwm_v) {
    s.setBounds(5000, 0, 0, 0, 0);
    s.setPeriodMultiplier(PWM.PeriodMultiplier.k4X);
    s.setRaw(pwm_v.intValue());
    //System.out.println(pwm_v.intValue());
 
  }

  public void default_position(){
    servo_W(s1, 0.0);
  }
}