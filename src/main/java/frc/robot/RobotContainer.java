// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Setpoints.Arm.GroundIntake;
import frc.robot.Setpoints.Arm.OuttakeArm;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.OutakeArmSubsystem;
import frc.robot.subsystems.OutakeRollerSubsystem;
import swervelib.SwerveInputStream;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.lang.reflect.Method;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.systems.LoadingSystem;
import frc.robot.systems.ScoringSystem;
import frc.robot.systems.TargetingSystem;
import frc.robot.systems.TargetingSystem.ReefBranchLevel;

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

  Command driveRobotOrientedAngularVelocity = drivebase.drive(driveAngularVelocity);


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
 
 


}
public void defaultCommands(){
  // intakeArmSubsystem.setDefaultCommand(intakeArmSubsystem.holdDefer());
  // outakeArmSubsystem.setDefaultCommand(outakeArmSubsystem.holdDefer());
  // elevatorSubsystem.setDefaultCommand(elevatorSubsystem.holdDefer());
  // drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity);
  
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
    
    boolean testingEl = true;
    boolean testingGR = true;
    boolean testingOu = true;
    boolean loadingtest = false;

    // if (testingEl){
    //   m_driverController.button(1).onTrue(elevatorSubsystem.CoralL1());
    //   m_driverController.button(2).onTrue(elevatorSubsystem.CoralL2());
    //   m_driverController.button(3).onTrue(elevatorSubsystem.CoralL3());
    //   m_driverController.button(4).onTrue(elevatorSubsystem.CoralL4());
    // }
    // if (testingGR){
    //   m_driverController.button(1).onTrue(intakeArmSubsystem.setPass());
    //   m_driverController.button(2).onTrue(intakeArmSubsystem.setGround());
    //   m_driverController.button(3).onTrue(intakeArmSubsystem.setAngle(130));

    // }
    // if (testingOu){
    //   m_driverController.button(1).onTrue(outakeArmSubsystem.L1());
    //   m_driverController.button(2).onTrue(outakeArmSubsystem.L2());
    //   m_driverController.button(3).onTrue(outakeArmSubsystem.L3());
    //   m_driverController.button(4).onTrue(outakeArmSubsystem.L4());
    //   m_driverController.button(5).onTrue(outakeArmSubsystem.pass());
    // }
    // if (loadingtest) {
    //   m_driverController.button(1).onTrue(loadingSystem.coralLoad());
    //   m_driverController.button(2).onTrue(loadingSystem.coralTransfer());
    //   m_driverController.button(3).onTrue(loadingSystem.coralLoad().andThen(loadingSystem.coralTransfer()));
    // }


    // //m_driverController.button(1).onTrue(scoringSystem.scoreCoral());
    

    // m_driverController.rightBumper().whileTrue(groundRollers.in());
    // m_driverController.leftBumper().whileTrue(elevatorSubsystem.setElevatorHeight(Meters.convertFrom(5, Inches)));

    // m_driverController.button(1).whileTrue(loadingSystem.coralLoad());
    // m_driverController.button(2).whileTrue(loadingSystem.coralTransfer());
    // m_driverController.button(3).whileTrue(scoringSystem.scoreCoral());
    // m_driverController.button(4).whileTrue(loadingSystem.coralLoad().andThen(loadingSystem.coralTransfer()).andThen(scoringSystem.scoreCoral()));

    // m_driverController.button(5).whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
    //                                                      .andThen(Commands.runOnce(() ->
    //                                                                                    drivebase.getSwerveDrive().field.getObject(
    //                                                                                        "target").setPose(
    //                                                                                        targetingSystem.getCoralTargetPose())))
    //                                                      .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L4))
    //                                      );
    // m_driverController.button(6).onTrue(scoringSystem.scoreCoral());
    //   //L2 Score Coral
    //   m_driverController.button(6).whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
    //                                                      .andThen(Commands.runOnce(() ->
    //                                                                                    drivebase.getSwerveDrive().field.getObject(
    //                                                                                        "target").setPose(
    //                                                                                        targetingSystem.getCoralTargetPose())))

    //                                                      .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L2))
    //                                      );
    //   //L3 Score Coral
    //   m_driverController.button(7).whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
    //                                                      .andThen(Commands.runOnce(() ->
    //                                                                                    drivebase.getSwerveDrive().field.getObject(
    //                                                                                        "target").setPose(
    //                                                                                        targetingSystem.getCoralTargetPose())))

    //                                                      .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L3))
    //                                      );
    //   //L4 Score Coral
    //   m_driverController.button(8).whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
    //                                                      .andThen(Commands.runOnce(() ->
    //                                                                                    drivebase.getSwerveDrive().field.getObject(
    //                                                                                        "target").setPose(
    //                                                                                        targetingSystem.getCoralTargetPose())))
    //                                                      .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L4))
    //                                      );
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
}
