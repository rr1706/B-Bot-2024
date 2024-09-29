package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase{
    private final CANSparkMax m_motor = new CANSparkMax(6, MotorType.kBrushless);
    private final RelativeEncoder m_Encoder = m_motor.getEncoder();
    private final SparkPIDController m_pid = m_motor.getPIDController();
    private double m_angle = 5.0;
    private boolean m_PIDEnabled = true;

    public Pivot() {
        m_motor.setSmartCurrentLimit(CurrentLimit.kPivot);
        m_motor.enableVoltageCompensation(GlobalConstants.kVoltageCompensation);
        m_motor.setIdleMode(IdleMode.kBrake);
        m_pid.setP(PivotConstants.kP);

        m_motor.setInverted(true);
        m_motor.setSoftLimit(SoftLimitDirection.kForward, 37);
        m_motor.setSoftLimit(SoftLimitDirection.kReverse, 0);
        m_motor.burnFlash();
    }

    public void pitchToAngle(double angle) {
        m_PIDEnabled = true; 
        if (angle >= 36.0) {
            angle = 36.0;
        } else if (angle <= 1.0) {
            angle = 1.0;
        }
        m_angle = angle;
    }

    public Command changePitch(double adjust) {
        return runOnce(() -> {
            m_PIDEnabled = true; 
            m_angle += adjust;
            if (m_angle >= 36.0) {
                m_angle = 36.0;
        } else if (m_angle <= 1.0) {
            m_angle = 1.0;
        }
        });
    }

    public Command pitchCommand(double angle){
        return runOnce(()->pitchToAngle(angle));
    }

    public void stop() {
        m_motor.stopMotor();
    }
     public void setZero() {
        m_Encoder.setPosition(-0.4);
     }

     public void zero() {
        m_PIDEnabled = false;
        m_motor.set(-0.1);
     }

     public double getCurrent() {
        return m_motor.getOutputCurrent();
     }

     @Override
     public void periodic() {
        if (m_PIDEnabled) {
            m_pid.setReference(m_angle, ControlType.kPosition);
        }
     }
    
 {
    
 }   
}
