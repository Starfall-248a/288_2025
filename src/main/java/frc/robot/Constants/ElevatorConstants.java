package frc.robot.Constants;

public class ElevatorConstants {
    /* PID values */
    public static final double kElevatorP = 8;
    public static final double kElevatorI = 1.5;
    public static final double kElevatorD = 4;

    public static final double kElevatorTolerance = 75;

    public static final double kFeedForwardS = (0.95 - 0.2)/2*0.8;   /* kG too high - kG too low / 2  0.95, 0.2 */
    public static final double kFeedForwardG = (0.95 + 0.2)/2;  /* kG too high + kG too low / 2 */    // calculated value 0.6
    public static final double kFeedForwardV = 3.5;   // calculated value 0.12
    public static final double kFeedForwardA = 2.0;   // calculated value 0.1

    /* Positions */
    public static final double kStowPosition = 0;
    public static final double kSafePosition = 8000;
    public static final double kLoadingPosition = 12000;
    public static final double kStabPosition = 1000;
    public static final double kT2Position = 11000;
    public static final double kT3Position = 10000;
    public static final double kT4Position = 21500;
}
