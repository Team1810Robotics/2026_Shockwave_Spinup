
package frc.robot;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.Constants.OperatorConstants;

//import frc.robot.commands.Autos;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  // Subsystems
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final IndexerSubsystem m_IndexerSubsystem = new IndexerSubsystem();

  // Xbox controller
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

   
m_driverController.leftTrigger()
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
            m_IndexerSubsystem
        )
    );


    // LEFT BUMPER - INTAKE REVERSE
    m_driverController.leftBumper().whileTrue(
        Commands.startEnd(
            m_IntakeSubsystem::intakeBackward,
            m_IntakeSubsystem::intakeStop,
            m_IntakeSubsystem
        )
    );

    // RIGHT BUMPER - INDEXER REVERSE
    m_driverController.rightBumper().whileTrue(
        Commands.startEnd(
            m_IndexerSubsystem::runReverse,
            m_IndexerSubsystem::stop,
            m_IndexerSubsystem
        )
    );

    // B BUTTON - INTAKE UP
    m_driverController.b().onTrue(
        Commands.startEnd(
            m_IntakeSubsystem::intakeDown,
            m_IntakeSubsystem::intakeStop,
            m_IntakeSubsystem
        )
    );

    // X BUTTON - INTAKE DOWN
    m_driverController.x().onTrue(
        Commands.startEnd(
            m_IntakeSubsystem::intakeUp,
            m_IntakeSubsystem::intakeStop,
            m_IntakeSubsystem
        )
    );
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
