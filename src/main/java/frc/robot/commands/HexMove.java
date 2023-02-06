// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.ejml.equation.IntegerSequence.For;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HexMove extends SequentialCommandGroup {
  /** Creates a new HexMove. */
  public HexMove(Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    for(int i=0;i<=300;i+=5){
    addCommands(
        new DirectionDrive(drivetrain, i, 0.2, 0.1)
      ); }}
}
