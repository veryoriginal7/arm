// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ArmControl extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Arm m_subsystem;
  private final double xx;
  private final double yy;
  private final int rev;
  private final double nyy;
  private final double nxx;
  /**
   * Creates a new ArmControl.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmControl(Arm subsystem, xx,yy,rev,nyy,nxx) {
    m_subsystem = subsystem;
    this.xx = xx;
    this.yy = yy;
    this.rev = rev;
    this.nyy = nyy;
    this.nxx = nxx;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.reset;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.nextOutputAngle(double yy, double xx);
    subsystem.nextOutputdistance(int rev,double nxx, double nyy)
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean isFinished) {
    subsystem.noMove();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return subsystem.limit();
  }
}
