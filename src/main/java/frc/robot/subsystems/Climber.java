package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.climberConstants;

public class Climber extends SubsystemBase{
    private TalonFX climberMotor = new TalonFX(climberConstants.climberMotorID);

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
        
        climberMotor.getConfigurator().apply(configs);
    }

    public void set(double speed) {
        climberMotor.set(speed);
    }

    // public void setVel(double speed) {
    //     climberMotor.setControl(new VelocityVoltage(speed));
    // }

    public void setPos(double pos) {
        climberMotor.setControl(new PositionVoltage(pos));
    }

    public void stop() {
        climberMotor.stopMotor();
    }

    public Command setCommand(DoubleSupplier speed) {
        return this.run(() -> set(speed.getAsDouble()));
    }

    // public Command setVelCommand(DoubleSupplier speed) {
    //     return this.run(() -> setVel(speed.getAsDouble()));
    // }

    public Command setPosCommand(DoubleSupplier pos) {
        return this.run(() -> setPos(pos.getAsDouble()));
    }

    public Command stopCommand() {
        return this.runOnce(() -> stop());
    }
}
