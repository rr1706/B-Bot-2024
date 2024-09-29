package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;
import com.thethriftybot.ThriftyNova.EncoderType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.ModuleConstants.Aziumth;
import frc.robot.Constants.ModuleConstants.Drive;

/**
 * FRC 1706 Class for creating a swerve module with 2 SparkMax motor controllers
 * and an analog azimuth Encoder
 */
public class SparkNovaSwerveModule extends SubsystemBase {
    private final CANSparkMax m_azimuthMotor;
    private final ThriftyNova m_driveMotor;
    private final AbsoluteEncoder m_azimuthEnc;
    private final SparkPIDController m_azimuthPID;

    /**
     * Create a new FRC 1706 NEOKrakenSwerveModule Object
     *
     * @param moduleID module ID also CAN ID for Azimuth and Drive MCs.
     * @param offset   The offset for the analog encoder.
     */
    public SparkNovaSwerveModule(int moduleID, double offset) {
        m_driveMotor = new ThriftyNova(moduleID);
        m_driveMotor.setMaxCurrent(CurrentType.STATOR, (double)CurrentLimit.kDrive)
            .setMaxCurrent(CurrentType.SUPPLY, (double)CurrentLimit.kDrive);
        m_driveMotor.setBrakeMode(true);
        m_driveMotor.setInverted(false);
        m_driveMotor.useEncoderType(EncoderType.INTERNAL);
        m_azimuthMotor = new CANSparkMax(moduleID + 10, MotorType.kBrushless);
        //m_azimuthMotor.restoreFactoryDefaults();
        m_azimuthMotor.setSmartCurrentLimit(CurrentLimit.kAzimuth);
        m_azimuthMotor.enableVoltageCompensation(GlobalConstants.kVoltageCompensation);
        m_azimuthMotor.setInverted(false);
        m_azimuthMotor.setIdleMode(IdleMode.kBrake);

        m_azimuthEnc = m_azimuthMotor.getAbsoluteEncoder(Type.kDutyCycle);
        m_azimuthEnc.setPositionConversionFactor(Aziumth.kPositionFactor);
        m_azimuthEnc.setVelocityConversionFactor(Aziumth.kVelocityFactor);

        //m_azimuthEnc.setZeroOffset(offset);

        m_azimuthEnc.setInverted(true);

        m_azimuthPID = m_azimuthMotor.getPIDController();

        m_azimuthPID.setFeedbackDevice(m_azimuthEnc);

        m_azimuthPID.setPositionPIDWrappingEnabled(true);
        m_azimuthPID.setPositionPIDWrappingMinInput(0.0);
        m_azimuthPID.setPositionPIDWrappingMaxInput(2.0 * Math.PI);

        m_azimuthPID.setP(Aziumth.kp);

        m_azimuthMotor.burnFlash();
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getStateAngle()));
    }

    /**
     * 
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getStateAngle()));
    }

    public double getDriveVelocity() {
        return m_driveMotor.getVelocity()/42.0 * Drive.kToMeters;
    }

    public double getDrivePosition() {
        return m_driveMotor.getPosition()/42.0 * Drive.kToMeters;
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getStateAngle()));
        double metersToRotations = state.speedMetersPerSecond * Drive.kToRots;

        m_driveMotor.setPercent(metersToRotations/96.0);

        m_azimuthPID.setReference(state.angle.getRadians(), ControlType.kPosition);
    }

    public double getStateAngle() {
        return m_azimuthEnc.getPosition();
    }

    public void stop() {
        m_driveMotor.setPercent(0.0);
        m_azimuthMotor.stopMotor();
    }

}
