// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
//import frc.robot.subsystems.CommandAlgaeMech;
import frc.robot.subsystems.CommandCoralElevator;
import frc.robot.subsystems.CommandCoralPivot;

public class RobotContainer {

    private final SendableChooser<Command> autoChooser;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * .9; // kSpeedAt12Volts desired top
                                                                                       // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    /* Telemetry logger */
    /* Telemetry logger */
    private final Telemetry logger = new Telemetry(MaxSpeed);

    /* Drive controllers */
    private final CommandPS4Controller driver = new CommandPS4Controller(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // /* Subsystems */
    private final CommandCoralPivot coralMech = new CommandCoralPivot();
    private final CommandCoralElevator coralElevator = new CommandCoralElevator();
    //private final CommandAlgaeMech algaeMech = new CommandAlgaeMech();

    // /* Auto functions */
    // NamedCommands.registerCommand("Coral T4 Prime", coralMech.exampleCommand());

    /* Drivetrain */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings(); // set controller bindings

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser("Simple auto");

        NamedCommands.registerCommand("Coral T4 Prime", coralMech.PivotTo170());

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                 // negative Y (forward)
                        .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                  // negative X (left)
                ));
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                 // negative Y (forward)
                        .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                  // negative X (left)
                );

        driver.cross().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.circle().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));
        driver.cross().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.circle().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

        // Run SysId routines when holding share/options and triangle/square.
        // Run SysId routines when holding share/options and triangle/square.
        // Note that each routine should be run exactly once in a single log.
        driver.share().and(driver.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.share().and(driver.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.options().and(driver.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.options().and(driver.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        driver.share().and(driver.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.share().and(driver.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.options().and(driver.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.options().and(driver.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on L1 press
        driver.triangle().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        /* Coral pivot */
        operator.povDown().whileTrue(coralMech.runOnce(() -> coralMech.PivotTo0()));
        operator.povRight().whileTrue(coralMech.runOnce(() -> coralMech.PivotTo90()));
        operator.povUp().whileTrue(coralMech.runOnce(() -> coralMech.PivotTo170()));

        /* Elevator movement */
        operator.a().whileTrue(coralElevator.Lower());
        // operator.b().whileTrue(coralElevator.runOnce(() -> coralElevator.RaiseToT3()));
        operator.y().whileTrue(coralElevator.Raise());
        operator.a().onFalse(coralElevator.Stop());
        operator.y().onFalse(coralElevator.Stop());

        /* Algae mech */
        //operator.b().whileTrue(algaeMech.runOnce(() -> algaeMech.PivotToAngle(0)));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
