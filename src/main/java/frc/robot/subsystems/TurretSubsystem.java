package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.databind.exc.MismatchedInputException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
private final TalonFX turretMotor = new TalonFX(TurretConstants.turretMotorID);
double turretDeg;

//double fullRange = 0;

AnalogPotentiometer potentiometer = new AnalogPotentiometer(3,-581,581);
private double setpoint = 0;
    
public TurretSubsystem(){

SmartDashboard.putNumber("Full Range", 0);

var talonFXConfigs = new TalonFXConfiguration();
talonFXConfigs.CurrentLimits.StatorCurrentLimit = 60;
talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 30;
talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

var slot0Configs = talonFXConfigs.Slot0;
slot0Configs.kS = TurretConstants.kS;
slot0Configs.kV = TurretConstants.kV;
slot0Configs.kA = TurretConstants.kA;
slot0Configs.kP = TurretConstants.kP;
slot0Configs.kI = TurretConstants.kI;
slot0Configs.kD = TurretConstants.kD;

var motionMagicConfigs = talonFXConfigs.MotionMagic;
motionMagicConfigs.MotionMagicCruiseVelocity = 50;
motionMagicConfigs.MotionMagicAcceleration = 600;
motionMagicConfigs.MotionMagicJerk = 1600;

talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

//talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.turretMaximumRotation;
//talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.turretMinimumRotation;
turretMotor.getConfigurator().apply(talonFXConfigs);
zeroFromPotentiometer();
boolean potentiometerConnected = potentiometer.get()==-90 ? false : true;
SmartDashboard.putBoolean("Potentiometer Connected", potentiometerConnected);
//turretMotor.setPosition(degreesToRotations(90));
SmartDashboard.putNumber("Turret Position", 0);
}


@Override
public void periodic(){
    SmartDashboard.putNumber("pot value", potentiometer.get());
    SmartDashboard.putNumber("turret motor position", turretMotor.getPosition().getValueAsDouble());
    turretDeg = turretMotor.getPosition().getValueAsDouble()*TurretConstants.gearRatio*TurretConstants.degreesPerRev;
    SmartDashboard.putNumber("turret Degrees", turretDeg);
    //fullRange = SmartDashboard.getNumber("Full Range", 0);
    SmartDashboard.putNumber("clamped setpoint", setpoint);
}

 public void setPosition(){
  setTurretPosition(SmartDashboard.getNumber("Turret Position", 0));
 }

private double clampTurretRotation(double degrees) {
    MathUtil.clamp(degrees, TurretConstants.turretMinimumRotation, TurretConstants.turretMaximumRotation);
    return degrees;
}

private void zeroFromPotentiometer(){
    turretMotor.setPosition((potentiometer.get()/360 * 5.33)-degreesToRotations(200.62));
}

public void setTurretPosition(double degrees){
    double min = TurretConstants.turretMinimumRotation;
    double max = TurretConstants.turretMaximumRotation;
    if(degrees < 0){
        degrees += 360;
    }
    if(degrees < min){
        setpoint = min;
    }
    else if(degrees > max){
        setpoint = max;
    }
    else{
        setpoint = degrees;
    }
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    //double clampedSetpoint = clampTurretRotation(degrees);
    turretMotor.setControl(m_request.withPosition(degreesToRotations(setpoint)));
}

public double degreesToRotations(double degrees){
    return degrees/360 * 5.33;
}

public double getTurretDegrees(){
    turretDeg = turretMotor.getPosition().getValueAsDouble()*TurretConstants.gearRatio*TurretConstants.degreesPerRev;
    return turretDeg;
}

public void setTurretSpeed(double speed){
    turretMotor.set(speed);
}

public void setTurretVoltage(double volts){
    turretMotor.setVoltage(volts);
}

public void stopTurretMotor(){
    turretMotor.stopMotor();
}
}
