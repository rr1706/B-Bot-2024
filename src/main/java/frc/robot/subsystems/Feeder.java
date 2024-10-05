package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;

public class Feeder extends SubsystemBase {
    private final CANSparkMax m_motor = new CANSparkMax(9, MotorType.kBrushless);

    public Feeder() {
        m_motor.restoreFactoryDefaults();
        m_motor.setSmartCurrentLimit(CurrentLimit.kFeeder);
        m_motor.enableVoltageCompensation(GlobalConstants.kVoltageCompensation);
        m_motor.setIdleMode(IdleMode.kBrake);

        m_motor.setInverted(true);
        m_motor.burnFlash();
    }

    public void run(double speed) {
        m_motor.set(speed);
    }

    public Command runCommand(double speed) {
        return runOnce(() -> run(speed));
    }

    public Command feed(){
        return runEnd(()->run(0.8), ()->stop());
    }

    public void stop() {
        m_motor.stopMotor();
    }

    public Command stopCommand() {
        return runOnce(() -> stop());
    }
}
