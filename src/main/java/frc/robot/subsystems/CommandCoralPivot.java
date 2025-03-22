package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;

public class CommandCoralPivot implements Subsystem{
    public static final TalonFX PivotMotor = new TalonFX(13); // Coral arm pivot motor
    private final Encoder coralEncoder; 

    public PIDController pivotPID = new PIDController(8, 0, 0.1); // PID controller for the pivot motor

    /* Subsystem init */
    public CommandCoralPivot() {
        pivotPID.setTolerance(.01); // Set the tolerance for the PID controller
        coralEncoder = new Encoder(4, 5,false, Encoder.EncodingType.k2X);

    }


    /* Set point commands */

    public void moveToSetPosition(double position) {
        pivotPID.setSetpoint(position);
        PivotMotor.set(pivotPID.calculate(getPosition(), position));
    }

    public Command moveToSetPositionCommand(double position) {
        return run(() -> moveToSetPosition(position)).until(isAtPosition());
    }

    public double getPosition() {
        return coralEncoder.getDistance();
    }
    

    public void resetCoralEncoder() {
        coralEncoder.reset();
    }

    public Command setCoralPosition(double position) {
        return run(() -> {
            PivotMotor.set(MathUtil.clamp(pivotPID.calculate(coralEncoder.getDistance(), position), -0.9, 0.9));
        }).until(() -> pivotPID.atSetpoint());
    }

    public BooleanSupplier isAtPosition() {
        double setPoint = pivotPID.getSetpoint();
        return () -> Math.abs(getPosition() - setPoint) < .1;
    }
}