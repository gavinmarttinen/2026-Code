package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase{
private final Servo leftServo;
private final Servo rightServo;
private final DigitalInput limitSwitch = new DigitalInput(3);
 double m_speed;
 double m_length;
 double setPos;
 double curPos;
 /**
 * Parameters for L16-R Actuonix Linear Actuators
 *
 * @param channel PWM channel used to control the servo
 * @param length max length of the servo [mm]
 * @param speed max speed of the servo [mm/second]
 */
 public HoodSubsystem() {
 leftServo = new Servo(1);
 leftServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
 m_length = 100;
 m_speed = 32;

 rightServo = new Servo(2);
 rightServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
 }
 /**
 * Run this method in any periodic function to update the position estimation of your
servo
 *
 * @param setpoint the target position of the servo [mm]
 */
 public void setPosition(double setpoint){
 setPos = MathUtil.clamp(setpoint, 0, m_length);
 leftServo.setSpeed( (setPos/m_length *2)-1);
 rightServo.setSpeed( (setPos/m_length *2)-1);
 }
 double lastTime = 0;
 /**
 * Run this method in any periodic function to update the position estimation of your
servo
 */
 public void updateCurPos(){
 double dt = Timer.getFPGATimestamp() - lastTime;
 if (curPos > setPos + m_speed *dt){
 curPos -= m_speed *dt;
 } else if(curPos < setPos - m_speed *dt){
 curPos += m_speed *dt;
 }else{
 curPos = setPos;
 }
 }
 /**
 * Current position of the servo, must be calling {@link #updateCurPos()
updateCurPos()} periodically
 *
 * @return Servo Position [mm]
 */
 public double getPosition(){
 return curPos;
 }
 /**
 * Checks if the servo is at its target position, must be calling {@link #updateCurPos()
updateCurPos()} periodically
 * @return true when servo is at its target
 */
 public boolean isFinished(){
 return curPos == setPos;
 }

 public void homeLinearActuator(){
   leftServo.setSpeed(-0.1);
   rightServo.setSpeed(-0.1);
   if(limitSwitch.get()){
      leftServo.setSpeed(0);
      rightServo.setSpeed(0);
      curPos = 0;
   }
 }
}