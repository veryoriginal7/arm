// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  public Arm() {}
  private final CANSparkMax m_Arm = new CANSparkMax(1,kBrushless);
  private final CANSparkMax m_Arm2 = new CANSparkMax(1,kBrushless);
  private final RelativeEncoder m_encoder = m_Arm.getAlternateEncoder(Type.kQuadrature, 42);
  private final PIDController m_PID = new PIDController(1,0.1,0.1);
  private final PIDController m_PID2 = new PIDController(1,0.1,0.1);
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public void nextOutputAngle(double y, double x){
    double calculate = m_PID.calculate(m_encoder.getPostion()*360,Math.atan2(y,x));
    m_Arm.set(calculate);
  }
  public void nextOutputdistance(int time,double nX, double nY){
    double calculate2 = m_PID2.calculate(3.6*time+2,Math.sqrt((nX*nX)+(nY*nY)));
    m_Arm2.set(calculate);}
  
  public void reset(){
    m_encoder.setPosition(0)
  }

  public boolean limit (){
    (m_encoder.getPostion()*360)>180 ||
    3.6*time+2;

  } 
  public void noMove(){
    m_Arm.set(0);
    m_Arms2.set(0);
  }
  }


