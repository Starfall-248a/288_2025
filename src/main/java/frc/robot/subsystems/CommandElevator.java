package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.ElevatorConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CommandElevator implements Subsystem {
    public static SparkMax elevatorMotor = new SparkMax(14, MotorType.kBrushless);
    private static final Encoder elevatorEncoder = new Encoder(0, 1, true, Encoder.EncodingType.k2X); // Encoder for the elevator

    private static final PIDController elevatorPID = new PIDController(
        ElevatorConstants.kElevatorP, //kp
        ElevatorConstants.kElevatorI, //ki
        ElevatorConstants.kElevatorD //kd
    ); // PID controller for the elevator motor

    /* Subsystem init */
    public CommandElevator() {
        elevatorPID.setTolerance(ElevatorConstants.kElevatorTolerance); // Set the tolerance for the PID controller
        elevatorEncoder.reset();
        elevatorPID.reset();

    }

    public boolean isSafe() { //detect weather or not the elevator is above the safe position
        boolean isAboveSafePosition = elevatorEncoder.getDistance() > ElevatorConstants.kSafePosition;
        SmartDashboard.putBoolean("Elevator Above Safe Position", isAboveSafePosition);
        return isAboveSafePosition;
    }

    public double getPosition() {
        return elevatorEncoder.getDistance();
    }
    
    public Double getElevatorPosition() {
        return elevatorEncoder.getDistance();
    }

    public void resetElevatorEncoder() {
        elevatorEncoder.reset();
    }

    public Command setGravity() {
        return run(() -> {
            elevatorMotor.set(.025);
        });
    }

    // /* Set point commands */

    public Command setPosition(double position) {
        return run(() -> {
            elevatorMotor.set(MathUtil.clamp(elevatorPID.calculate(elevatorEncoder.getDistance(), position), -0.9, 0.9));
        }).until(() -> elevatorPID.atSetpoint());
    }

    public BooleanSupplier isAtPosition() {
        double setPoint = elevatorPID.getSetpoint();
        return () -> Math.abs(getPosition() - setPoint) < .1;
    }
}
