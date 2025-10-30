// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Map;
import java.util.Set;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.systems.TargetingSystem;
import frc.robot.systems.TargetingSystem.ReefBranchLevel;
import frc.robot.Setpoints;
import frc.robot.Setpoints.Elevator.Coral;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ElevatorSubsystem extends SubsystemBase {


    private SparkMax m_motor = new SparkMax(CanIDs.ElevatorMain, MotorType.kBrushless);

    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withMechanismCircumference(ElevatorConstants.mechanismCircumference)

            .withClosedLoopController(ElevatorConstants.kP,
                    ElevatorConstants.kI,
                    ElevatorConstants.kD,
                    MetersPerSecond.of(0.5),
                    MetersPerSecondPerSecond.of(0.5))

            .withSimClosedLoopController(ElevatorConstants.ksimP,
                    ElevatorConstants.ksimI,
                    ElevatorConstants.ksimD,
                    MetersPerSecond.of(0.5),
                    MetersPerSecondPerSecond.of(0.5))
            // Feedforward Constants
            .withFeedforward(new ElevatorFeedforward(ElevatorConstants.kS,
                    ElevatorConstants.kG,
                    ElevatorConstants.kV))

            .withSimFeedforward(new ElevatorFeedforward(ElevatorConstants.ksimS,
                    ElevatorConstants.ksimG,
                    ElevatorConstants.ksimV))
            // Telemetry name and verbosity level
            .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
            // Gearing from the motor rotor to final shaft.
            // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which corresponds to the gearbox attached to your motor.
            .withGearing(new MechanismGearing(new GearBox(ElevatorConstants.gearbox)))
            // Motor properties to prevent over currenting.
            .withMotorInverted(false)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(ElevatorConstants.statorCurrentLimit)
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25))
            .withFollowers(Pair.of(new SparkMax(CanIDs.ElevatorFollower, MotorType.kBrushless), false));


    // Vendor motor controller object

    // Create our SmartMotorController from our Spark and config with the NEO.
    private SmartMotorController sparkSmartMotorController = new SparkWrapper(m_motor, DCMotor.getNEO(2), smcConfig);

    private ElevatorConfig elevconfig = new ElevatorConfig(sparkSmartMotorController)
            .withStartingHeight(ElevatorConstants.startingHeight)
            .withHardLimits(ElevatorConstants.hardLimitMin, ElevatorConstants.hardLimitMax)
            .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
            .withMass(ElevatorConstants.mass);

    //Elevator Mechanism
    private Elevator m_elevator = new Elevator(elevconfig);


    /**
     * Creates a new ExampleSubsystem.
     */
    public ElevatorSubsystem() {
    }


    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        m_elevator.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        m_elevator.simIterate();
    }


    private Trigger atHeight(double height, double tolerance) {
        return new Trigger(() -> MathUtil.isNear(height,
                getHeightMeters(),
                tolerance));
    }

    /**
     * Gets the height of the elevator and compares it to the given height with the given tolerance.
     *
     * @param height         Height in meters
     * @param allowableError Tolerance in meters.
     * @return Within that tolerance.
     */
    public boolean aroundHeight(double height, double allowableError) {
        return MathUtil.isNear(height, getHeightMeters(), allowableError);
    }

    /**
     * Gets the height of the elevator and compares it to the given height with the given tolerance.
     *
     * @param height Height in meters
     * @return Within that tolerance.
     */
    public boolean aroundHeight(double height) {
        return aroundHeight(height, ElevatorConstants.kElevatorAllowableError);
    }


    private double getHeightMeters() {
        return m_elevator.getHeight().in(Meters);
    }

    /**
     * Set the height of the elevator.
     *
     * @param angle Distance to go to.
     */
    public Command setElevatorHeight(double heightInMeters) {
        return m_elevator.setHeight(Units.Meters.of(heightInMeters));
//                .until(atHeight(getHeightMeters(), ElevatorConstants.kElevatorAllowableError));
    }

    // public void hold(){
    //   return m_elevator.setHeight(m_elevator.getHeight());
    // }

    /**
     * Move the elevator up and down.
     *
     * @param dutycycle [-1, 1] speed to set the elevator too.
     */
    public Command set(double dutycycle) {
        return m_elevator.set(dutycycle);
    }

    /**
     * Run sysId on the {@link Elevator}
     */
    public Command sysId() {
        return m_elevator.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
    }

    public Command setPower(double d) {
        return run(() -> m_motor.set(d));
    }

    public Command toMin() {
        return setPower(-0.1).until(m_elevator.min());
    }

    public Command CoralL1() {
        return setElevatorHeight(Setpoints.Elevator.Coral.L1);
    }

    public Command CoralL2() {
        return setElevatorHeight(Setpoints.Elevator.Coral.L2);
    }

    public Command CoralL3() {
        return setElevatorHeight(Setpoints.Elevator.Coral.L3);
    }

    public Command CoralL4() {
        return setElevatorHeight(Setpoints.Elevator.Coral.L4);
    }

    public Command pass(){
        return setElevatorHeight(Meters.convertFrom(5, Inches));
    }

    public Command hold() {
        return m_elevator.setHeight(m_elevator.getHeight());
    }

    public Command holdDefer(){
        return Commands.defer(()->hold(),Set.of(this));
    }

    public Command getCoralCommand(TargetingSystem targetingSystem) {
        return Commands.select(Map.of(ReefBranchLevel.L1, CoralL1(),
                        ReefBranchLevel.L2, CoralL2(),
                        ReefBranchLevel.L3, CoralL3(),
                        ReefBranchLevel.L4, CoralL4()),
                targetingSystem::getTargetBranchLevel);
    }

    public Trigger atCoralHeight(TargetingSystem targetingSystem) {
        return new Trigger(() -> {
            switch (targetingSystem.getTargetBranchLevel()) {
                case L2 -> {
                    return aroundHeight(Coral.L2);
                }
                case L3 -> {
                    return aroundHeight(Coral.L3);
                }
                case L1 -> {
                    return aroundHeight(Coral.L1);
                }
                case L4 -> {
                    return aroundHeight(Coral.L4);
                }
            }
            return false;
        });


    }
}

