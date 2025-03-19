package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.controller.PIDController;

public class CommandCoralPivot implements Subsystem{
    public static final TalonFX PivotMotor = new TalonFX(13); // Coral arm pivot motor

    public PIDController pivotPID = new PIDController(8, 0, 0.1); // PID controller for the pivot motor

    /* Subsystem init */
    public CommandCoralPivot() {
        pivotPID.setTolerance(.01); // Set the tolerance for the PID controller
        PivotMotor.setPosition(0);
        pivotPID.reset();
    }

    public Command setBrake(){
        return run(
            () -> {
                PivotMotor.set(0); // Set the pivot motor to the calculated PID value
            });
    }

    public double getPosition() {
        return PivotMotor.getPosition().getValueAsDouble();
    }

    /* Set point commands */

    public BooleanSupplier isAtPosition() {
        double setPoint = pivotPID.getSetpoint();
        return () -> Math.abs(getPosition() - setPoint) < .1;
    }

    public void moveToSetPosition(double position) {
        pivotPID.setSetpoint(position);
        PivotMotor.set(pivotPID.calculate(getPosition(), position));
    }

    public Command moveToSetPositionCommand(double position) {
        return run(() -> moveToSetPosition(position)).until(isAtPosition());
    }
}