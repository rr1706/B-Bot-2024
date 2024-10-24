package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.MathUtils;

public class AutoShooterByPose extends Command {
    private final Shooter m_shooter;
    private final Drivetrain m_robotDrive;
    private final Pivot m_pitcher;
    private Supplier<Pose2d> getPose;

    private InterpolatingDoubleTreeMap m_pitchTable = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap m_velocityTable = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap m_timeTable = new InterpolatingDoubleTreeMap();

    private final Timer m_timer = new Timer();

    private final SlewRateLimiter m_pitchFilter = new SlewRateLimiter(60.0);
    private final SlewRateLimiter m_velocityFilter = new SlewRateLimiter(400.0);

    public AutoShooterByPose(Shooter shooter, Drivetrain robotDrive, Pivot pitcher, Supplier<Pose2d> getPose) {
        m_shooter = shooter;
        m_robotDrive = robotDrive;
        m_pitcher = pitcher;

        this.getPose = getPose;

        m_pitchTable = MathUtils.pointsToTreeMap(ShooterConstants.kPitchTable);
        m_velocityTable = MathUtils.pointsToTreeMap(ShooterConstants.kVelocityTable);
        m_timeTable = MathUtils.pointsToTreeMap(ShooterConstants.kTimeTable);
        addRequirements(m_shooter);

    }

    @Override
    public void initialize() {
        m_pitchFilter.reset(m_pitcher.getSetPitch());
        m_velocityFilter.reset(m_shooter.getSetVelocity());
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        var alliance = DriverStation.getAlliance();

        Translation2d goalLocation;

        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            goalLocation = GoalConstants.kRedGoal;

        } else {
            goalLocation = GoalConstants.kBlueGoal;

        }

        goalLocation = compForMovement(goalLocation);

        Translation2d toGoal = goalLocation.minus(getPose.get().getTranslation());

        double goalDistance = toGoal.getDistance(new Translation2d()) * 39.37;

        SmartDashboard.putNumber("Pose Distance", goalDistance);

        m_pitcher.pitchToAngle(m_pitchFilter.calculate(m_pitchTable.get(goalDistance)));
        m_shooter.run(m_velocityFilter.calculate(m_velocityTable.get(goalDistance)), 30.0);

    }

    Translation2d compForMovement(Translation2d goalLocation) {

        Translation2d toGoal = goalLocation.minus(getPose.get().getTranslation());

        double rx = m_robotDrive.getFieldRelativeSpeed().vx + m_robotDrive.getFieldRelativeAccel().ax * 0.030;
        double ry = m_robotDrive.getFieldRelativeSpeed().vy + m_robotDrive.getFieldRelativeAccel().ay * 0.030;

        double shotTime = m_timeTable.get(toGoal.getDistance(new Translation2d()));
        return new Translation2d(goalLocation.getX() - rx * shotTime, goalLocation.getY() - ry * shotTime);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_pitcher.pitchToAngle(10.0);
        // TODO Auto-generated method stub
    }

}
