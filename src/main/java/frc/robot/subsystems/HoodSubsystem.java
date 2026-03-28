package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkParameters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AutoAim;
import frc.robot.Constants.HoodConstants;

public class HoodSubsystem extends SubsystemBase{
private final SparkMax linearActuator = new SparkMax(HoodConstants.sparkMaxID, MotorType.kBrushed);
private AutoAim autoAim;
private final DutyCycleEncoder encoder = new DutyCycleEncoder(HoodConstants.encoderChannel);
private final PIDController pidController = new PIDController(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD);

 public HoodSubsystem(AutoAim autoAim) {

 SmartDashboard.putNumber("Hood Position", 0);
 encoder.setInverted(true);
 this.autoAim = autoAim;
 }

 public void setPosition(double position){
   linearActuator.set(pidController.calculate(encoder.get(), MathUtil.clamp(position, HoodConstants.hoodMinPosition, HoodConstants.hoodMaxPosition)));
 }

 public void setHood(double speed){
   linearActuator.set(speed);
 }

 @Override
 public void periodic(){
  SmartDashboard.putNumber("Hood Encoder", encoder.get());
  SmartDashboard.putNumber("AutoAim hub distance", autoAim.getHubDistance());
  SmartDashboard.putNumber("AutoAim hub rotation", autoAim.getHubRotation());
  SmartDashboard.putNumber("goal pose X", autoAim.goalPositionWithTOF().getX());
  SmartDashboard.putNumber("goal pose Y", autoAim.goalPositionWithTOF().getY());
  //setPosition(SmartDashboard.getNumber("Hood Position", 0));
 }

 public void setAutoAimHoodPosition(){
   setPosition(autoAim.calculateHoodAngle());
 }
}