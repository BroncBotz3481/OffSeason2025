// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.GroundConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OutakeConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.OutakeArmSubsystem;
import frc.robot.subsystems.OutakeRollerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.systems.LoadingSystem;
import frc.robot.systems.ScoringSystem;
import frc.robot.systems.TargetingSystem;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final SwerveSubsystem drivebase = new SwerveSubsystem();//Remember to add new swerve.json

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =  
      new CommandXboxController(OperatorConstants.kOperatorControllerPort) ;

  private final SwerveSubsystem drivebase = new SwerveSubsystem();

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY()*-1,
                                                                () -> m_driverController.getLeftX()*-1)
                                                            .withControllerRotationAxis(() ->
                                                                                            m_driverController.getRightX() *
                                                                                            -1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.3)
                                                            .scaleRotation(0.6)
                                                            .allianceRelativeControl(false);
  SwerveInputStream driveDirectAngle     = driveAngularVelocity.copy()
                                                               .withControllerHeadingAxis(() -> m_driverController.getRightX(),
                                                                                          () -> m_driverController.getRightY())
                                                               .headingWhile(true);


  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> -m_driverController.getLeftY(),
                                                                   () -> -m_driverController.getLeftX())
                                                               .withControllerRotationAxis(() -> m_driverController.getRawAxis(
                                                                   2))
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);

  Command driveRobotOrientedAngularVelocity = drivebase.drive(Robot.isSimulation() ? driveAngularVelocitySim : driveAngularVelocity);


  private final IntakeArmSubsystem intakeArmSubsystem = new IntakeArmSubsystem();
  private final OutakeArmSubsystem outakeArmSubsystem = new OutakeArmSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final IntakeRollerSubsystem groundRollers = new IntakeRollerSubsystem();
  private final OutakeRollerSubsystem outtakeRollers = new OutakeRollerSubsystem();
  private final TargetingSystem targetingSystem = new TargetingSystem();
  private final LoadingSystem loadingSystem = new LoadingSystem(intakeArmSubsystem, elevatorSubsystem, drivebase, outakeArmSubsystem, groundRollers, outtakeRollers, targetingSystem, driveAngularVelocity);
  private final ScoringSystem scoringSystem = new ScoringSystem(intakeArmSubsystem, elevatorSubsystem, drivebase, outakeArmSubsystem, groundRollers, outtakeRollers, loadingSystem, targetingSystem, driveAngularVelocity); 

                                                               
 /** The container for the robot. Contains subsystems, OI devices, and commands. */
 public RobotContainer() {
  // Configure the trigger bindings
  defaultCommands();
  configureBindings();
  DriverStation.silenceJoystickConnectionWarning(true);
 
 


}
public void defaultCommands(){
  // intakeArmSubsystem.setDefaultCommand(intakeArmSubsystem.holdDefer());
  // outakeArmSubsystem.setDefaultCommand(outakeArmSubsystem.holdDefer());
  // elevatorSubsystem.setDefaultCommand(elevatorSubsystem.holdDefer());

  intakeArmSubsystem.setDefaultCommand(intakeArmSubsystem.set(0));
  outakeArmSubsystem.setDefaultCommand(outakeArmSubsystem.set(0));
  elevatorSubsystem.setDefaultCommand(elevatorSubsystem.set(0));

  drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity);
  
}
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
    boolean testingEl = false;
    boolean testingGR = false;
    boolean testingOu = false;
    boolean loadingtest = false;

    if (testingEl){
      m_driverController.button(1).onTrue(elevatorSubsystem.CoralL1());
      m_driverController.button(2).onTrue(elevatorSubsystem.CoralL2());
      m_driverController.button(3).onTrue(elevatorSubsystem.CoralL3());
      m_driverController.button(4).onTrue(elevatorSubsystem.CoralL4());
    }
    if (testingGR){
      m_driverController.button(1).onTrue(intakeArmSubsystem.setPass());
      m_driverController.button(2).onTrue(intakeArmSubsystem.setGround());
      m_driverController.button(3).onTrue(intakeArmSubsystem.setAngle(130));

    }
    if (testingOu){
      m_driverController.button(1).onTrue(outakeArmSubsystem.L1());
      m_driverController.button(2).onTrue(outakeArmSubsystem.L2());
      m_driverController.button(3).onTrue(outakeArmSubsystem.L3());
      m_driverController.button(4).onTrue(outakeArmSubsystem.L4());
      m_driverController.button(5).onTrue(outakeArmSubsystem.pass());
    }
    if (loadingtest) {
      m_driverController.button(1).onTrue(loadingSystem.coralLoad());
      m_driverController.button(2).onTrue(loadingSystem.coralTransfer());
      m_driverController.button(3).onTrue(loadingSystem.coralLoad().andThen(loadingSystem.coralTransfer()));
    }
    m_driverController.button(7).whileTrue(intakeArmSubsystem.sysId());
    m_driverController.button(1).whileTrue(loadingSystem.coralLoad());
    m_driverController.button(2).whileTrue(loadingSystem.coralLoadAuto());
    m_driverController.button(3).whileTrue(loadingSystem.coralLoad()).whileFalse(loadingSystem.coralTransfer());

    /////////////////// ---------- BUTTONS ---------- \\\\\\\\\\\\\\\\\\\\\\\

    
    // m_operatorController.button(1).whileTrue(intakeArmSubsystem.sysId());
    // m_operatorController.button(2).whileTrue(intakeArmSubsystem.set(0));
    // // Schedule `set` when the Xbox controller's B button is pressed,
    // // cancelling on release.
    // m_operatorController.button(3).whileTrue(intakeArmSubsystem.set(0.3));
    // m_operatorController.button(4).whileTrue(intakeArmSubsystem.set(-0.3));
    // m_operatorController.button(5).onTrue(intakeArmSubsystem.setAngle(50));

    if (Robot.isSimulation()){

      m_operatorController.button(1).whileTrue(loadingSystem.coralLoad());
      m_operatorController.button(2).onTrue(loadingSystem.coralLoadAuto());
  
      m_operatorController.button(3).onTrue(elevatorSubsystem.CoralL1().alongWith(outakeArmSubsystem.L1()));
      m_operatorController.button(4).onTrue(elevatorSubsystem.CoralL2().alongWith(outakeArmSubsystem.L2()));
      m_operatorController.button(5).onTrue(elevatorSubsystem.CoralL3().alongWith(outakeArmSubsystem.L3()));
      m_operatorController.button(6).onTrue(elevatorSubsystem.CoralL4().alongWith(outakeArmSubsystem.L4()));
  
      m_operatorController.button(7).whileTrue(outtakeRollers.out());
      m_operatorController.button(8).onTrue(intakeArmSubsystem.setGround());

    } else {

      m_operatorController.rightBumper().whileTrue(loadingSystem.coralLoad());
      m_operatorController.leftBumper().onTrue(loadingSystem.coralLoadAuto());
  
      m_operatorController.a().onTrue(elevatorSubsystem.CoralL1().alongWith(outakeArmSubsystem.L1()));
      m_operatorController.b().onTrue(elevatorSubsystem.CoralL2().alongWith(outakeArmSubsystem.L2()));
      m_operatorController.x().onTrue(elevatorSubsystem.CoralL3().alongWith(outakeArmSubsystem.L3()));
      m_operatorController.y().onTrue(elevatorSubsystem.CoralL4().alongWith(outakeArmSubsystem.L4()));
  
      m_operatorController.povUp().whileTrue(outtakeRollers.out());
      m_operatorController.povDown().onTrue(intakeArmSubsystem.setGround());

    }


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
  //testing stuff
  // public Command coralTransfer(){
  //   return elevatorSubsystem.setElevatorHeight(Meters.convertFrom(5, Inches))
  //   .alongWith(intakeArmSubsystem.setPass(), outakeArmSubsystem.pass());
  // }

  public Command setStartAngles(){
    return Commands.run(()-> {
      intakeArmSubsystem.setAngle(GroundConstants.kStartingPose.in(Degree));
      outakeArmSubsystem.setAngle(OutakeConstants.kStartingPose.in(Degree));
    });
  }
}
