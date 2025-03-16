package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.controller.PIDController;

public class CommandAlgae implements Subsystem {
    private final SparkMax algaeWrist = new SparkMax(19, MotorType.kBrushless);
    private final TalonFX algaeIntake = new TalonFX(17);

    public CommandAlgae() {
        
    }

    public Command up(){
        return run(
            () -> {
                algaeWrist.set(.25);
            });
    }

    public Command down(){
        return run(
            () -> {
                algaeWrist.set(-.25);
            });
    }

    public Command intake(){
        return run(
            () -> {
                algaeIntake.set(.35);
            });
    }

    public Command outtake(){
        return run(
            () -> {
                algaeIntake.set(-1);
            });
    }

    public Command stopIntake(){
        return run(
            () -> {
                algaeIntake.set(0);
            });
    }

    public Command stopWrist(){
        return run(
            () -> {
                algaeWrist.set(0);
            });
    }
}