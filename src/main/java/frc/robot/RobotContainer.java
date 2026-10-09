package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Constants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {

  // Subsystems
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();

  private final IndexerSubsystem m_IndexerSubsystem = new IndexerSubsystem();

  private final HoodSubsystem m_HoodSubsystem = new HoodSubsystem();

  // Xbox controller
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();

    // Hood manual and automatic control
    m_HoodSubsystem.setDefaultCommand(
        Commands.runEnd(
            () -> m_HoodSubsystem.control(m_driverController.getHID().getPOV()),
            m_HoodSubsystem::stop,
            m_HoodSubsystem));
  }

  private void configureBindings() {

    // LEFT TRIGGER - INTAKE AND INDEXER FORWARD
    m_driverController
        .leftTrigger()
        .and(m_driverController.rightBumper().negate())
        .whileTrue(
            Commands.startEnd(
                () -> {
                  m_IntakeSubsystem.intakeForward();
                  m_IndexerSubsystem.runForward();
                },
                () -> {
                  m_IntakeSubsystem.intakeStop();
                  m_IndexerSubsystem.stop();
                },
                m_IntakeSubsystem,
                m_IndexerSubsystem));

    // LEFT BUMPER - INTAKE REVERSE
    m_driverController
        .leftBumper()
        .whileTrue(
            Commands.startEnd(
                m_IntakeSubsystem::intakeBackward,
                m_IntakeSubsystem::intakeStop,
                m_IntakeSubsystem));

    // RIGHT BUMPER - INDEXER REVERSE
    m_driverController
        .rightBumper()
        .whileTrue(
            Commands.startEnd(
                m_IndexerSubsystem::runReverse, m_IndexerSubsystem::stop, m_IndexerSubsystem));

    // B BUTTON - INTAKE UP
    m_driverController
        .b()
        .onTrue(
            new InstantCommand(
                () -> m_IntakeSubsystem.setPoint(Constants.Intake_Up), m_IntakeSubsystem));

    // X BUTTON - INTAKE DOWN
    m_driverController
        .x()
        .onTrue(
            new InstantCommand(
                () -> m_IntakeSubsystem.setPoint(Constants.Intake_Down), m_IntakeSubsystem));

    // A BUTTON - TOGGLE AUTOMATIC HOOD AIMING
    m_driverController.a().onTrue(Commands.runOnce(m_HoodSubsystem::toggleAutomatic));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
