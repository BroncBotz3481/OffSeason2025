// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.OutakeArmSubsystem;
import swervelib.SwerveInputStream;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final SwerveSubsystem drivebase = new SwerveSubsystem();//Remember to add new swerve.json
  private final IntakeArmSubsystem intakeArmSubsystem = new IntakeArmSubsystem();
  private final OutakeArmSubsystem outakeArmSubsystem = new OutakeArmSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
  //                                                               () -> m_driverController.getLeftY()*-1,
  //                                                               () -> m_driverController.getLeftX()*-1)
  //                                                           .withControllerRotationAxis(() ->
  //                                                                                           m_driverController.getRightX() *
  //                                                                                           -1)
  //                                                           .deadband(OperatorConstants.DEADBAND)
  //                                                           .scaleTranslation(0.3)
  //                                                           .scaleRotation(0.6)
  //                                                           .allianceRelativeControl(false);
  // SwerveInputStream driveDirectAngle     = driveAngularVelocity.copy()
  //                                                              .withControllerHeadingAxis(() -> m_driverController.getRightX(),
  //                                                                                         () -> m_driverController.getRightY())
  //                                                              .headingWhile(true);


  // SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
  //                                                                  () -> -m_driverController.getLeftY(),
  //                                                                  () -> -m_driverController.getLeftX())
  //                                                              .withControllerRotationAxis(() -> m_driverController.getRawAxis(
  //                                                                  2))
  //                                                              .deadband(OperatorConstants.DEADBAND)
  //                                                              .scaleTranslation(0.8)
  //                                                              .allianceRelativeControl(true);

                                                               
 /** The container for the robot. Contains subsystems, OI devices, and commands. */
 public RobotContainer() {
  // Configure the trigger bindings
  configureBindings();

  intakeArmSubsystem.setDefaultCommand(intakeArmSubsystem.setAngle(Degrees.of(150)));
  outakeArmSubsystem.setDefaultCommand(outakeArmSubsystem.setAngle(Degrees.of(-15)));
  elevatorSubsystem.setDefaultCommand(elevatorSubsystem.setHeight(Inches.of(0)));

  


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
    
    m_driverController.button(1).whileTrue(intakeArmSubsystem.setAngle(Degrees.of(45)));
    m_driverController.button(2).whileTrue(intakeArmSubsystem.setAngle(Degrees.of(20)));
    m_driverController.button(3).whileTrue(outakeArmSubsystem.setAngle(Degrees.of(25)));
    m_driverController.button(4).whileTrue(outakeArmSubsystem.setAngle(Degrees.of(-25)));
    m_driverController.button(5).whileTrue(elevatorSubsystem.setHeight(Inches.of(70)));

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
}
