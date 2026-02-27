// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// 16 25 13/16
package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SpindexConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SpindexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {

    private final SendableChooser<Command> autoChooser;

    private final CommandPS5Controller driverController = new CommandPS5Controller(0);
    private final CommandPS5Controller operatorController = new CommandPS5Controller(1);

    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    private final SpindexSubsystem spindexSubsystem = new SpindexSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final TurretSubsystem turretSubsystem = new TurretSubsystem();


    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final AutoAim autoAim = new AutoAim(drivetrain);
    private final HoodSubsystem hoodSubsystem = new HoodSubsystem(autoAim);

    public RobotContainer() {
        configureBindings();

        NamedCommands.registerCommand("Spin Up Shooter 5 Sec", Commands.run(()-> shooterSubsystem.setShooterVelocity(ShooterConstants.shooterMotorVelocity), shooterSubsystem).withTimeout(5));
        NamedCommands.registerCommand("Run Intake", Commands.run(()->intakeSubsystem.setIntakeSpeed(-IntakeConstants.intakeMotorSpeed), intakeSubsystem));
        NamedCommands.registerCommand("Right Trench Turret Position 1 Sec", new ParallelCommandGroup(Commands.run(()->turretSubsystem.setTurretPosition(85),turretSubsystem),Commands.run(()->hoodSubsystem.setHood(0.2), hoodSubsystem)).withTimeout(1));
        NamedCommands.registerCommand("Right Trench Turret Position 0.5 Sec", new ParallelCommandGroup(Commands.run(()->turretSubsystem.setTurretPosition(85),turretSubsystem),Commands.run(()->hoodSubsystem.setHood(0.2), hoodSubsystem)).withTimeout(0.5));
        NamedCommands.registerCommand("Left Trench Turret Position 1 Sec", new ParallelCommandGroup(Commands.run(()->turretSubsystem.setTurretPosition(6),turretSubsystem),Commands.run(()->hoodSubsystem.setHood(0.2), hoodSubsystem)).withTimeout(1));
        NamedCommands.registerCommand("Left Trench Turret Position 0.5 Sec", new ParallelCommandGroup(Commands.run(()->turretSubsystem.setTurretPosition(6),turretSubsystem),Commands.run(()->hoodSubsystem.setHood(0.2), hoodSubsystem)).withTimeout(0.5));
        NamedCommands.registerCommand("Run Spindex 7 Sec", Commands.run(()->spindexSubsystem.setSpindexSpeed(SpindexConstants.spindexMotorSpeed,SpindexConstants.kickerMotorSpeed), spindexSubsystem).withTimeout(7).andThen(Commands.run(()->spindexSubsystem.reverseKickerStopSpindex(), spindexSubsystem).withTimeout(0.1)));
        NamedCommands.registerCommand("Run Spindex 3.5 Sec", Commands.run(()->spindexSubsystem.setSpindexSpeed(SpindexConstants.spindexMotorSpeed,SpindexConstants.kickerMotorSpeed), spindexSubsystem).withTimeout(3.5).andThen(Commands.run(()->spindexSubsystem.reverseKickerStopSpindex(), spindexSubsystem).withTimeout(0.1)));
        NamedCommands.registerCommand("Run Spindex 2 Sec", Commands.run(()->spindexSubsystem.setSpindexSpeed(SpindexConstants.spindexMotorSpeed,SpindexConstants.kickerMotorSpeed), spindexSubsystem).withTimeout(2).andThen(Commands.run(()->spindexSubsystem.reverseKickerStopSpindex(), spindexSubsystem).withTimeout(0.1)));
        NamedCommands.registerCommand("Stop Spindex", Commands.run(()->spindexSubsystem.reverseKickerStopSpindex(), spindexSubsystem).withTimeout(0.1));
        NamedCommands.registerCommand("Left Deep Turret Position 1 Sec", new ParallelCommandGroup(Commands.run(()->turretSubsystem.setTurretPosition(32),turretSubsystem),Commands.run(()->hoodSubsystem.setHood(0.45), hoodSubsystem)));
        NamedCommands.registerCommand("Climber Up", Commands.run(()-> climberSubsystem.climberUp(ClimberConstants.climberMotorSpeed), climberSubsystem));
        NamedCommands.registerCommand("Climber Down", Commands.run(()-> climberSubsystem.climberDown(-ClimberConstants.climberMotorSpeed), climberSubsystem));
        NamedCommands.registerCommand("Drive To Climb Left", drivetrain.applyRequest(()->
            driveFacingAngle.withVelocityX(drivetrain.driveToClimbLeftX())
            .withVelocityY(drivetrain.driveToClimbLeftY())
            .withTargetDirection(Rotation2d.fromDegrees(0))));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        turretSubsystem.setDefaultCommand(Commands.run(()->turretSubsystem.setTurretPosition(autoAim.getHubRotation() - drivetrain.getState().Pose.getRotation().getDegrees()), turretSubsystem));
        //turretSubsystem.setDefaultCommand(Commands.run(()->turretSubsystem.setTurretSpeed(operatorController.getLeftX()), turretSubsystem));
       // turretSubsystem.setDefaultCommand(Commands.run(()->turretSubsystem.stopTurretMotor(), turretSubsystem));
        climberSubsystem.setDefaultCommand(Commands.run(()->climberSubsystem.setClimberSpeed(0), climberSubsystem));
        spindexSubsystem.setDefaultCommand(Commands.run(()->spindexSubsystem.reverseKickerStopSpindex(),spindexSubsystem));
        intakeSubsystem.setDefaultCommand(Commands.run(()->intakeSubsystem.setIntakeSpeed(0), intakeSubsystem));
        hoodSubsystem.setDefaultCommand(Commands.run(()->hoodSubsystem.setHoodPosition(), hoodSubsystem));

        driverController.button(11).whileTrue(drivetrain.applyRequest(()->brake));
        driverController.button(15).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        driverController.L2().whileTrue(Commands.run(()->turretSubsystem.setTurretPosition(90),turretSubsystem)); //Fix Turret
        driverController.R2().whileTrue(Commands.run(()->spindexSubsystem.setSpindexSpeed(driverController.getRawAxis(4)<0.5 ? SpindexConstants.spindexMotorSpeedSlow : SpindexConstants.spindexMotorSpeed, SpindexConstants.kickerMotorSpeed),spindexSubsystem));

        operatorController.R2().whileTrue(new ParallelCommandGroup(Commands.run(()->turretSubsystem.setTurretPosition(75),turretSubsystem),Commands.run(()->hoodSubsystem.setHood(0.2), hoodSubsystem))); //Right Trench
        operatorController.circle().whileTrue(new ParallelCommandGroup(Commands.run(()->turretSubsystem.setTurretPosition(41),turretSubsystem),Commands.run(()->hoodSubsystem.setHood(0.32), hoodSubsystem))); //Right Deep
        operatorController.L2().whileTrue(new ParallelCommandGroup(Commands.run(()->turretSubsystem.setTurretPosition(99),turretSubsystem),Commands.run(()->hoodSubsystem.setHood(0.2), hoodSubsystem))); //Left Trench
        operatorController.square().whileTrue(new ParallelCommandGroup(Commands.run(()->turretSubsystem.setTurretPosition(32),turretSubsystem),Commands.run(()->hoodSubsystem.setHood(0.45), hoodSubsystem))); //Left Deep
        operatorController.povUp().whileTrue(Commands.run(()->climberSubsystem.setClimberSpeed(ClimberConstants.climberMotorSpeed), climberSubsystem));
        operatorController.povDown().whileTrue(Commands.run(()->climberSubsystem.setClimberSpeed(-ClimberConstants.climberMotorSpeed), climberSubsystem));
        operatorController.L1().whileTrue(Commands.run(()->intakeSubsystem.setIntakeSpeed(IntakeConstants.intakeMotorSpeed), intakeSubsystem));
        operatorController.R1().toggleOnTrue(Commands.run(()->intakeSubsystem.setIntakeSpeed(-IntakeConstants.intakeMotorSpeed), intakeSubsystem));
        operatorController.povLeft().onTrue(Commands.run(()-> shooterSubsystem.setShooterVelocity(ShooterConstants.shooterMotorVelocity)));
        operatorController.povRight().whileTrue(new ParallelCommandGroup(Commands.run(()->shooterSubsystem.setShooterVelocity(ShooterConstants.shooterMotorVelocityMax),shooterSubsystem),Commands.run(()->hoodSubsystem.setHood(1), hoodSubsystem))); //Far Pass
        operatorController.cross().whileTrue(new ParallelCommandGroup(Commands.run(()->turretSubsystem.setTurretPosition(90),turretSubsystem),Commands.run(()->hoodSubsystem.setHood(0.2), hoodSubsystem))); //Tower
        operatorController.triangle().whileTrue(new ParallelCommandGroup(Commands.run(()->turretSubsystem.setTurretPosition(0),turretSubsystem),Commands.run(()->hoodSubsystem.setHood(0.0), hoodSubsystem), Commands.run(()->shooterSubsystem.setShooterVelocity(ShooterConstants.shooterMotorVelocityHub), shooterSubsystem)));
        operatorController.button(10).whileTrue(Commands.run(()->spindexSubsystem.setSpindexSpeed(SpindexConstants.spindexMotorSpeed, SpindexConstants.kickerMotorSpeed),spindexSubsystem));
        //operatorController.circle().whileTrue(Commands.run(()->turretSubsystem.setPosition(), turretSubsystem));
        //operatorController.square().whileTrue(Commands.run(()->hoodSubsystem.setPosition(), hoodSubsystem));
        drivetrain.registerTelemetry(logger::telemeterize);
    
    }
    
    public Command getAutonomousCommand() {
          return autoChooser.getSelected();
    }
}
