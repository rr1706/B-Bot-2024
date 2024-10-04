// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Constants.Auto;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveByController;
import frc.robot.commands.SmartShootByPose;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SendableChooser<Command> autoChooser;

  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();
  private final Feeder m_feeder = new Feeder();
  private final Pivot m_pivot = new Pivot();
  private final Drivetrain m_drive = new Drivetrain();
  private final PoseEstimator m_poseEstimator = new PoseEstimator(m_drive);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  private final DriveByController m_driveByController = new DriveByController(m_drive, m_driverController);

  private final SmartShootByPose m_shoot = new SmartShootByPose(m_shooter, m_drive, m_pivot, m_driverController,
      m_poseEstimator::getPose);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_drive.setDefaultCommand(m_driveByController);

    configureAutoBuilder();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.

    m_driverController.pov(0).onTrue(
        new InstantCommand(
            () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                m_poseEstimator
                    .resetOdometry(new Pose2d(new Translation2d(15.17, 5.55), new Rotation2d()));
              } else {
                m_poseEstimator.resetOdometry(
                    new Pose2d(new Translation2d(1.31, 5.55), new Rotation2d(Math.PI)));
              }
            }));
    m_driverController.pov(180).onTrue(new InstantCommand(
        () -> m_drive.resetOdometry(new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(Math.PI)))));

    m_driverController.a().onTrue(new InstantCommand(() -> m_shooter.run(9.7, 20)).alongWith(m_pivot.pitchCommand(36)))
        .onFalse(new InstantCommand(() -> m_shooter.stop()).alongWith(m_pivot.pitchCommand(5)));
    m_driverController.b()
        .onTrue(m_feeder.runCommand(0.5).alongWith(m_intake.runCommand(0.4))
            .alongWith(new WaitCommand(0.090).andThen(m_pivot.pitchCommand(26))))
        .onFalse(m_feeder.stopCommand().alongWith(m_intake.stopCommand()));
    m_driverController.leftTrigger()
        .whileTrue(m_shoot.alongWith(new InstantCommand(() -> m_poseEstimator.setAuto(false))));
    m_driverController.rightTrigger().whileTrue(m_feeder.feed().alongWith(m_intake.feed()))
        .onFalse(m_feeder.runCommand(-0.4).alongWith(m_shooter.runCommand(-10.0, 0.0)).raceWith(new WaitCommand(0.070))
            .andThen(m_intake.stopCommand().alongWith(new InstantCommand(() -> m_shooter.stop()))
                .alongWith(m_feeder.stopCommand())));
    m_driverController.leftBumper()
        .onTrue(new InstantCommand(() -> m_shooter.run(-20.0, 0)).alongWith(m_feeder.runCommand(-0.25))
            .alongWith(m_pivot.pitchCommand(26.0)))
        .onFalse(new InstantCommand(() -> m_shooter.stop()).alongWith(m_feeder.stopCommand())
            .alongWith(m_pivot.pitchCommand(5.0)));
    m_driverController.rightBumper().onTrue(m_intake.runCommand(-0.8)).onFalse(m_intake.stopCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link RPobot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();

  }

  public void configureAutoBuilder() {
    AutoBuilder.configureHolonomic(m_poseEstimator::getPose, m_poseEstimator::resetOdometry,
        m_drive::getChassisSpeed,
        m_drive::drive, Auto.autoConfig,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        }, m_drive);
  }
}
