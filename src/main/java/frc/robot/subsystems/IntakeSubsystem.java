package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class IntakeSubsystem extends SubsystemBase{
    private final TalonFX intakeMotor = new TalonFX(IntakeConstants.intakeMotorID);

public IntakeSubsystem(){
    SmartDashboard.putNumber("intakeMotorSpeed", 0);

    var talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 60;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 30;
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeMotor.getConfigurator().apply(talonFXConfigs);
}

@Override
public void periodic() {
   
double speed = SmartDashboard.getNumber("intakeMotorSpeed", 0.0);
SmartDashboard.putNumber("Intake Current", getIntakeCurrent());
    intakeMotor.set(speed);
}

public double getIntakeCurrent(){
     return intakeMotor.getSupplyCurrent().getValueAsDouble();
  }

public void setIntakeSpeed (double speed){
    intakeMotor.set(speed);
}

public void setIntakeSpeedWithCurrent(){
    intakeMotor.set(0.02*Math.pow(getIntakeCurrent(), 2) - 0.04*getIntakeCurrent() + 0.7);
}

}