package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AutoAim;

public class HoodSubsystem extends SubsystemBase{
private final Servo leftServo;
private final Servo rightServo;
private AutoAim autoAim;


 public HoodSubsystem(AutoAim autoAim) {


 SmartDashboard.putNumber("Hood Position", 0);
 leftServo = new Servo(8);
 leftServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

 rightServo = new Servo(9);
 rightServo.setBoundsMicroseconds(2000, 1550, 1500, 1450, 1000);
 this.autoAim = autoAim;
 }

 public void setPosition(){
  leftServo.setPosition(SmartDashboard.getNumber("Hood Position", 0));
  rightServo.setPosition(SmartDashboard.getNumber("Hood Position", 0));
 }

 @Override
 public void periodic(){
 // leftServo.setPosition(SmartDashboard.getNumber("Hood Position", 0));
  //rightServo.setPosition(SmartDashboard.getNumber("Hood Position", 0));
  SmartDashboard.putNumber("AutoAim hub distance", autoAim.getHubDistance());
 }

 public void setHoodPosition(){
  leftServo.setPosition(autoAim.calculateHoodAngle());
  rightServo.setPosition(autoAim.calculateHoodAngle());
 }
}