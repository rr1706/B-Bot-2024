// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveByController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();
  private final Feeder m_feeder = new Feeder();
  private final Pivot m_pivot = new Pivot();
  private final Drivetrain m_drive = new Drivetrain();


  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

          private final DriveByController m_driveByController = new DriveByController(m_drive, m_driverController);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
        m_drive.setDefaultCommand(m_driveByController);


    configureBindings();


  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.leftTrigger().onTrue(new InstantCommand(()->m_shooter.run(40, 25)).alongWith(m_pivot.pitchCommand(32))).onFalse(new InstantCommand(()->m_shooter.stop()).alongWith(m_pivot.pitchCommand(5)));
    m_driverController.leftBumper().onTrue(m_intake.runCommand(0.8).alongWith(m_feeder.runCommand(0.8))).onFalse(m_feeder.runCommand(-0.4).alongWith(new WaitCommand(0.090)).andThen(m_intake.stopCommand().alongWith(m_feeder.stopCommand())));
    m_driverController.rightBumper().onTrue(m_intake.runCommand(-0.8)).onFalse(m_intake.stopCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link RPobot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous[\]
    return new WaitCommand(15.0);
  }
}
