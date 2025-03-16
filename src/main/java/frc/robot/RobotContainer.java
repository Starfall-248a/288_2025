// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PS4Controller.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CoralPivotConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandCoralPivot;
import frc.robot.subsystems.CommandElevator;
import frc.robot.subsystems.CommandHang;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CommandAlgae;

public class RobotContainer {

    private final SendableChooser<Command> autoChooser;

    // Flag to track the current speed mode
    private boolean isSlowSpeed = false;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double SlowSpeed = MaxSpeed * 0.2; // 20% of top speed
    private double speedMultiplier = isSlowSpeed ? slowSpeed : 1.0;
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // Define the slow speed multiplier
    private static double slowSpeed = 1; // Adjust this value as needed

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Define the button to toggle speed mode
    private final CommandPS4Controller Driver = new CommandPS4Controller(0);
    private final CommandPS4Controller operator = new CommandPS4Controller(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final CommandCoralPivot coralPivot = new CommandCoralPivot();
    public final CommandAlgae algae = new CommandAlgae();
    public final CommandElevator elevator = new CommandElevator();
    public final CommandHang hang = new CommandHang();

    public RobotContainer() {
        configureBindings();
        
        // For convenience a programmer could change this when going to competition.
        boolean isCompetition = true;

        // Build an auto chooser. This will use Commands.none() as the default option.
        // As an example, this will only show autos that start with "comp" while at
        // competition as defined by the programmer
        autoChooser = null;
        /*autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        (stream) -> isCompetition
            ? stream.filter(auto -> auto.getName().startsWith("comp"))
            : stream
        );

        SmartDashboard.putData("Auto Chooser", autoChooser);*/
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> 
                drive.withVelocityX(-Driver.getLeftY() * MaxSpeed * slowSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-Driver.getLeftX() * MaxSpeed * slowSpeed * 0.4) // Drive left with negative X (left)
                    .withRotationalRate(-Driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        elevator.setDefaultCommand(elevator.setGravity());
        coralPivot.setDefaultCommand(coralPivot.setBrake());

        Driver.R2().onTrue(new InstantCommand(() -> slowSpeed = .25));
        Driver.R2().onFalse(new InstantCommand(() -> slowSpeed = 1));

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        Driver.circle().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-Driver.getLeftY(), -Driver.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // Driver.triangle().whileTrue(newSequence(
        //     drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-1, 0)),
        //     );

        
        // Driver.share().and(Driver.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // Driver.options().and(Driver.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // Driver.options().and(Driver.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        Driver.R1().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        operator.R1().whileTrue(hang.Up());
        operator.L1().whileTrue(hang.Down());
        operator.R1().onFalse(hang.Stop());
        operator.L1().onFalse(hang.Stop());

        /* Scoring macros */
        // operator.cross().onTrue(Commands.parallel(coralPivot.setPosition(CoralPivotConstants.kStow), elevator.setPosition(ElevatorConstants.kStowPosition)));
        operator.square().onTrue(Commands.sequence(elevator.setPosition(ElevatorConstants.kStabPosition), elevator.setPosition(ElevatorConstants.kLoadingPosition)));

        operator.povUp().onTrue(Commands.parallel(elevator.setPosition(ElevatorConstants.kT4Position)));
        operator.povDown().onTrue(Commands.parallel(elevator.setPosition(ElevatorConstants.kStowPosition)));
        operator.povLeft().onTrue(Commands.parallel(elevator.setPosition(ElevatorConstants.kT3Position)));
        operator.povRight().onTrue(Commands.sequence(elevator.setPosition(ElevatorConstants.kLoadingPosition)));

        operator.triangle().whileTrue(elevator.setPosition(elevator.getElevatorPosition() + .25));
        operator.cross().whileTrue(elevator.setPosition(elevator.getElevatorPosition() - .25));
        operator.circle().whileTrue(Commands.run(() -> elevator.resetElevatorEncoder()));

        operator.L2().whileTrue(Commands.run(() -> coralPivot.PivotMotor.set(0.45)));
        operator.L2().onFalse(Commands.run(() -> coralPivot.PivotMotor.set(0.0)));
        operator.R2().whileTrue(Commands.run(() -> coralPivot.PivotMotor.set(-0.45)));
        operator.R2().onFalse(Commands.run(() -> coralPivot.PivotMotor.set(0.0)));

        Driver.L2().whileTrue(algae.down());
        Driver.R2().whileTrue(algae.up());
        Driver.L2().onFalse(algae.stopWrist());
        Driver.R2().onFalse(algae.stopWrist());

        Driver.square().whileTrue(algae.intake());
        Driver.square().onFalse(algae.stopIntake());
        Driver.cross().whileTrue(algae.outtake());
        Driver.cross().onFalse(algae.stopIntake());


        // operator.circle().whileTrue(Commands.run(() -> elevator.))

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // Red = -.25
    // Blue = .25
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            drivetrain.applyRequest(() ->
            drive.withVelocityX(.25*MaxSpeed)
            .withVelocityY(0)
            .withRotationalRate(0)).withTimeout(1.2),
            drivetrain.applyRequest(() ->
            drive.withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0)).withTimeout(10));
    }

    private Command moveManipulator(double elevatorPosition, double armPosition) {
        return Commands.either(
            Commands.sequence(
                elevator.setPosition(elevatorPosition),
                coralPivot.setPosition(armPosition)
            ),
            Commands.parallel(
                elevator.setPosition(elevatorPosition),
                coralPivot.setPosition(armPosition)
            ),
            () -> elevator.isSafe()
        );
    }
}
