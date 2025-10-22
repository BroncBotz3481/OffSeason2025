package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.OutakeArmSubsystem;
import frc.robot.subsystems.OutakeRollerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;



public class ScoringSystem
{

  private IntakeArmSubsystem    m_intake;
  private OutakeArmSubsystem m_outake;
  private ElevatorSubsystem    m_elevator;
  private SwerveSubsystem      m_swerve;
  private IntakeRollerSubsystem m_intakeRoller;
  private OutakeRollerSubsystem m_outakeRoller;
  private SwerveInputStream m_swerveInputStream;
  private LoadingSystem        m_loadingSystem;
  private TargetingSystem      m_targetSystem;



  public ScoringSystem(IntakeArmSubsystem intake, ElevatorSubsystem elevator, SwerveSubsystem swerve,
                      OutakeArmSubsystem outake, IntakeRollerSubsystem intakeRoller, OutakeRollerSubsystem outakeRoller, 
                      LoadingSystem loading, TargetingSystem targeting, SwerveInputStream driveStream)
  {
    m_intake = intake;
    m_elevator = elevator;
    m_swerve = swerve;
    m_outake = outake;
    m_loadingSystem = loading;
    m_intakeRoller = intakeRoller;
    m_targetSystem = targeting;
    m_outakeRoller = outakeRoller;
    m_swerveInputStream = driveStream;
  }

  ///  Score Coral Command for PathPlanner that does not move the swerve drive at all keeping the pathplanner auto
  /// intact.
  public Command scoreCoralAuto()
  {
    return null;
  }

  public Command restArmsSafe()
  {
    return null;
  }

  public Command scoreCoral()
  {
    // Arm down, elevator down, drive backwards x in
    return m_swerve.stopDrivingCommand()
      .andThen(m_elevator.getCoralCommand(m_targetSystem).repeatedly().alongWith(m_outake.getCoralCommand(m_targetSystem).repeatedly())) 
      .until(m_elevator.atCoralHeight(m_targetSystem).and(m_outake.atCoralAngle(m_targetSystem)))
      .andThen(m_outake.hold().alongWith(m_elevator.hold(), m_targetSystem.driveToCoralTarget(m_swerve)))//.alongWith() ended when the last cmd ends
      .andThen(m_outake.hold().alongWith(m_elevator.hold(), m_outakeRoller.out().withTimeout(1.5)))
      .andThen(m_swerve.driveBackwards().alongWith(m_outake.pass(), m_elevator.toMin()));
  }



}

//                     WALDO
///                     ( ) /-----\
///                  |||||||||
//                   ((o)-(o))
//                    |  W  |
//                    --| |--
