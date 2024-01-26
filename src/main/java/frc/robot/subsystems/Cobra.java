package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.cobraConstants;

import java.util.function.DoubleSupplier;


public class Cobra extends SubsystemBase {
    private final TalonFX pivotMotor = new TalonFX(cobraConstants.pivotMotorID);
    private final TalonFX squisherMotor = new TalonFX(cobraConstants.squisherMotorID);
    private final CANSparkMax indexerMotor = new CANSparkMax(
            cobraConstants.indexerMotorID,
            MotorType.kBrushless);

    private final SparkPIDController indexerController;

    private final DutyCycleEncoder pivotEncoder =
            new DutyCycleEncoder(cobraConstants.pivotEncoderID);

    private final PIDController pivotPID = new PIDController(
            cobraConstants.pivotP,
            cobraConstants.pivotI,
            cobraConstants.pivotD);

    private final LaserCan laserCan1 = new LaserCan(cobraConstants.laserCan1ID);
    private final LaserCan laserCan2 = new LaserCan(cobraConstants.laserCan2ID);

    public Cobra() {
        TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
        TalonFXConfiguration squisherConfigs = new TalonFXConfiguration();

        pivotConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = cobraConstants.upperRotationSoftLimit;
        pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = cobraConstants.lowerRotationSoftLimit;
        pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        pivotConfigs.CurrentLimits.SupplyCurrentLimit = cobraConstants.rotationMotorCurrentLimit;
        pivotConfigs.Audio.BeepOnConfig = true;

        squisherConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        squisherConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        squisherConfigs.CurrentLimits.SupplyCurrentLimit = cobraConstants.squisherMotorCurrentLimit;
        squisherConfigs.Audio.BeepOnConfig = true;

        pivotMotor.getConfigurator().apply(pivotConfigs);
        squisherMotor.getConfigurator().apply(squisherConfigs);

        indexerMotor.setIdleMode(IdleMode.kBrake);
        indexerMotor.setInverted(false);
        indexerMotor.setSmartCurrentLimit(cobraConstants.indexerMotorCurrentLimit);

        indexerController = indexerMotor.getPIDController();
    }

    @Override
    public void periodic() {
        double speed = pivotPID.calculate(pivotEncoder.getAbsolutePosition());
        pivotMotor.setVoltage(speed);
    }

    // rotation motor basic setters

    public void setPivotSpeed(double speed) {
        pivotMotor.set(speed);
    }

    public void setPivotPos(double pos) {
        pivotPID.setSetpoint(pos);
    }

    public void stopPivot() {
        pivotMotor.stopMotor();
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

    public Command setPivotCommand(DoubleSupplier speed) {
        return this.run(() -> setPivotSpeed(speed.getAsDouble()));
    }

    public Command setPivotPosCommand(DoubleSupplier pos) {
        return this.run(() -> setPivotPos(pos.getAsDouble())).until(pivotPID::atSetpoint);
    }

    public Command stopPivotCommand() {
        return this.runOnce(() -> stopPivot());
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

    public Command CobraCollect() {
        return setPivotPosCommand(() -> cobraConstants.pivotCollectAngle).
                andThen(Commands.parallel(setSquisherCommand(() -> -0.5)));
    }
}
