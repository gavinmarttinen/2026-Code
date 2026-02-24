package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX climberMotor = new TalonFX(ClimberConstants.climberMotorID);

public ClimberSubsystem(){
    var talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 60;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 30;
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climberMotor.getConfigurator().apply(talonFXConfigs);
}

public void setClimberSpeed (double speed){
    climberMotor.set(speed);
}

public void climberUp (double speed){
    climberMotor.set(speed);
    if(climberMotor.getPosition().getValueAsDouble()>67){
        climberMotor.set(0);
    }
}
public void climberDown (double speed){
    climberMotor.set(speed);
    if(climberMotor.getPosition().getValueAsDouble()<67){
        climberMotor.set(0);
    }
}
}