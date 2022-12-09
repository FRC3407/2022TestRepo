package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final double kMaxSpeedMetersPerSecond = 1.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double drivebase_kS = 1.1185;	// "kS"(volts) -> base voltage required to overcome static friction -> from SysID characterization
    public static final double drivebase_kV = 2.1132;	// "kV"(volts * seconds / meters) -> voltage required for each additional meter/second of velocity -> from SysID characterization
    public static final double drivebase_kA = 1.0668;	// "kA"(volts * seconds^2 / meters) -> voltage required for each additional meter/second^2 of acceleration -> from SysID characterization
    public static final double drivebase_kP = 3.5176;	// "kP"(volts * seconds / meters) -> voltage added to correct for error

    public static final double trackwidth = Units.inchesToMeters(21.819);


    
}
