package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.cobraConstants;

public class Cobra extends SubsystemBase {
    private TalonFX rotationMotor = new TalonFX(cobraConstants.rotationMotorID);
    private TalonFX squisherMotor = new TalonFX(cobraConstants.squisherMotorID);
    private CANSparkMax indexerMotor = new CANSparkMax(
        cobraConstants.indexerMotorID, 
        MotorType.kBrushless);

    private SparkPIDController indexerController;

    public Cobra() {
        TalonFXConfiguration rotationConfigs = new TalonFXConfiguration();
        TalonFXConfiguration squisherConfigs = new TalonFXConfiguration();

        rotationConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rotationConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rotationConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = cobraConstants.upperRotationSoftLimit;
        rotationConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = cobraConstants.lowerRotationSoftLimit;
        rotationConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        rotationConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        rotationConfigs.CurrentLimits.SupplyCurrentLimit = cobraConstants.rotationMotorCurrentLimit;
        rotationConfigs.Audio.BeepOnConfig = true;

        squisherConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        squisherConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        squisherConfigs.CurrentLimits.SupplyCurrentLimit = cobraConstants.squisherMotorCurrentLimit;
        squisherConfigs.Audio.BeepOnConfig = true;

        rotationMotor.getConfigurator().apply(rotationConfigs);
        squisherMotor.getConfigurator().apply(squisherConfigs);

        indexerMotor.setIdleMode(IdleMode.kBrake);
        indexerMotor.setInverted(false);
        indexerMotor.setSmartCurrentLimit(cobraConstants.indexerMotorCurrentLimit);
        
        indexerController = indexerMotor.getPIDController();
    }

    // rotation motor basic setters

    public void setRotation(double speed) {
        rotationMotor.set(speed);
    }

    public void setRotationPos(double pos) {
        rotationMotor.setControl(new PositionVoltage(pos));
    }

    public void stopRotation() {
        rotationMotor.stopMotor();
    }

    // squisher motor basic setters

    public void setSquisher(double speed) {
        squisherMotor.set(speed);
    }

    public void setSquisherVel(double vel) {
        squisherMotor.setControl(new VelocityVoltage(vel));
    }

    public void stopSquisher() {
        squisherMotor.stopMotor();
    }

    // indexer motor basic setters

    public void setIndexer(double speed) {
        indexerMotor.set(speed);
    }

    public void setIndexerVel(double vel) {
        indexerController.setReference(vel, ControlType.kVelocity);
    }

    public void stopIndexer() {
        indexerMotor.stopMotor();
    }

    // COMMANDS //

    // rotation motor commands

    public Command setRotationCommand(DoubleSupplier speed) {
        return this.run(() -> setRotation(speed.getAsDouble()));
    }

    public Command setRotationPosCommand(DoubleSupplier pos) {
        return this.run(() -> setRotationPos(pos.getAsDouble()));
    }

    public Command stopRotationCommand() {
        return this.runOnce(() -> stopRotation());
    }

    // squisher motor commands

    public Command setSquisherCommand(DoubleSupplier speed) {
        return this.run(() -> setSquisher(speed.getAsDouble()));
    }

    public Command setsquisherVelCommand(DoubleSupplier vel) {
        return this.run(() -> setSquisherVel(vel.getAsDouble()));
    } 

    public Command stopSquisherCommand() {
        return this.runOnce(() -> stopSquisher());
    }

    // indexer motor commands

    public Command setIndexerCommand(DoubleSupplier speed) {
        return this.run(() -> setIndexer(speed.getAsDouble()));
    }

    public Command setIndexerVelCommand(DoubleSupplier vel) {
        return this.run(() -> setIndexerVel(vel.getAsDouble()));
    }

    public Command stopIndexerCommand() {
        return this.runOnce(() -> stopIndexer());
    }
}
