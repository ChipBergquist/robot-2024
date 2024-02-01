package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.climberConstants;

public class Climber extends SubsystemBase{
    private TalonFX climberMotor1 = new TalonFX(climberConstants.climberMotor1ID);
    private TalonFX climberMotor2 = new TalonFX(climberConstants.climberMotor2ID);

    public Climber() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.CurrentLimits.SupplyCurrentLimit = climberConstants.climberMotorCurrentLimit;
        configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = climberConstants.upperSoftLimit;
        configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = climberConstants.lowerSoftLimit;
        configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false; // TODO: find upper limit for soft stop
        configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        configs.Audio.BeepOnConfig = true;
        
        climberMotor1.getConfigurator().apply(configs);
        climberMotor2.getConfigurator().apply(configs);
        climberMotor2.setControl(new Follower(climberMotor1.getDeviceID(), false));
    }

    public void set(double speed) {
        climberMotor1.set(speed);
    }

    public void setPos(double pos) {
        climberMotor1.setControl(new PositionVoltage(pos));
    }

    public void stop() {
        climberMotor1.stopMotor();
    }

    public Command setCommand(DoubleSupplier speed) {
        return this.run(() -> set(speed.getAsDouble()));
    }

    public Command setPosCommand(DoubleSupplier pos) {
        return this.run(() -> setPos(pos.getAsDouble()));
    }

    public Command stopCommand() {
        return this.runOnce(() -> stop());
    }
}
