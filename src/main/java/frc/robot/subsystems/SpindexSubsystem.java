package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpindexConstants;

public class SpindexSubsystem extends SubsystemBase {
    private final TalonFX spindexMotor = new TalonFX(SpindexConstants.spindexMotorID);
    private final TalonFX kickerMotor = new TalonFX(SpindexConstants.kickerMotorID);
    private final Timer spindexTimer = new Timer();

    public SpindexSubsystem(){
    var talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 60;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 40;
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    spindexMotor.getConfigurator().apply(talonFXConfigs);
    kickerMotor.getConfigurator().apply(talonFXConfigs);
    spindexTimer.reset();
}

@Override
public void periodic(){
    //SmartDashboard.putNumber("Spindex Current", getSpindexCurrent());
}

public void setSpindexSpeed(double spindexMotorSpeed, double kickerMotorSpeed){
    // if(getSpindexCurrent()>SpindexConstants.spindexCurrentLimit){
    //     spindexTimer.start();
      
        
    // }
    //  if(spindexTimer.get()<SpindexConstants.spindexTimer&&spindexTimer.isRunning()){
    //     stopSpindex();
    //    // reverseSpindex();
    // }

    // else{
     spindexMotor.set(spindexMotorSpeed);
     kickerMotor.set(kickerMotorSpeed);
    //spindexTimer.stop();
    //spindexTimer.reset();
//}
}

public void stopSpindex(){
    spindexMotor.set(0);
    kickerMotor.set(0);
}

// public double getSpindexCurrent(){
//     return spindexMotor.getSupplyCurrent().getValueAsDouble();
// }

public void reverseSpindex(){
   spindexMotor.set(SpindexConstants.spindexReverseSpeed);
}
}
