package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.databind.exc.MismatchedInputException;

import edu.wpi.first.math.MathUtil;
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

AnalogInput m_input = new AnalogInput(1);

AnalogPotentiometer potentiometer = new AnalogPotentiometer(m_input,180,-90);
    
public TurretSubsystem(){

var talonFXConfigs = new TalonFXConfiguration();
talonFXConfigs.CurrentLimits.StatorCurrentLimit = 60;
talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 30;
talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

var slot0Configs = talonFXConfigs.Slot0;
slot0Configs.kS = ShooterConstants.kS;
slot0Configs.kV = ShooterConstants.kV;
slot0Configs.kA = ShooterConstants.kA;
slot0Configs.kP = ShooterConstants.kP;
slot0Configs.kI = ShooterConstants.kI;
slot0Configs.kD = ShooterConstants.kD;

var motionMagicConfigs = talonFXConfigs.MotionMagic;
motionMagicConfigs.MotionMagicAcceleration = 400;
motionMagicConfigs.MotionMagicJerk = 4000;

talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.turretMaximumRotation;
talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.turretMinimumRotation;
turretMotor.getConfigurator().apply(talonFXConfigs);
zeroFromPotentiometer();
boolean potentiometerConnected = potentiometer.get()==-90 ? false : true;
SmartDashboard.putBoolean("Potentiometer Connected", potentiometerConnected);
}

@Override
public void periodic(){
}

private double clampTurretRotation(double targetRotations) {
    MathUtil.clamp(targetRotations, TurretConstants.turretMinimumRotation, TurretConstants.turretMaximumRotation);
    return targetRotations;
}

private void zeroFromPotentiometer(){
    double turretDeg = potentiometer.get();
    turretDeg = MathUtil.clamp(turretDeg, TurretConstants.turretMinimumRotation, TurretConstants.turretMaximumRotation);
    turretMotor.setPosition(Units.degreesToRotations(turretDeg));
}

public void setTurretPosition(double position){
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    double clampedSetpoint = clampTurretRotation(position);
    turretMotor.setControl(m_request.withPosition(Units.degreesToRotations(clampedSetpoint)));
}

public void setTurretSpeed(double speed){
    turretMotor.set(speed);
}
}
