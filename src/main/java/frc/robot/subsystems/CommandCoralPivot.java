package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.controller.PIDController;

public class CommandCoralPivot implements Subsystem{
    public final TalonFX PivotMotor = new TalonFX(13); // Coral arm pivot motor

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

    public Double getPosition() {
        return PivotMotor.getPosition().getValueAsDouble();
    }

    /* Set point commands */

    public Command setPosition(double Position){
        return run(
            () -> {
                PivotMotor.set(MathUtil.clamp(pivotPID.calculate(PivotMotor.getPosition().getValueAsDouble(), Position), -.9, .9)); // Set the pivot motor to the calculated PID value
            }).until(() -> pivotPID.atSetpoint());
    }
}