// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class MacroPlay extends CommandBase {
  Drivetrain drivetrain;
  int idxr = 0;
  /** Creates a new MacroPlay. */
  public MacroPlay(Drivetrain drivetrain) {
    this.drivetrain=drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double jX=0;
    double tAngle=0;
    double dPower=0;
    boolean b2=false;
    boolean trigger=false;
    if(idxr<750){jX=Robot.jxArray[idxr];tAngle=Robot.tAngleArray[idxr];dPower=Robot.dPowerArray[idxr];trigger=Robot.triggerArray[idxr];b2=Robot.b2Array[idxr];idxr++;System.out.println(idxr);}
    drivetrain.RobotMove(tAngle, dPower, jX, trigger, b2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {drivetrain.RobotMove(0, 0, 0, false, false);}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return idxr>Constants.MacroTime;
  }
}
