package frc.robot;


import edu.wpi.first.math.util.Units;


public final class constants {
    public static final double kMaxSpeedMetersPerSecond = 1.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double drivebase_kS = 1.1185;
    public static final double drivebase_kV = 2.1132;
    public static final double drivebase_kA = 1.0668;
    public static final double drivebase_kP = 3.5176;

    public static final double trackwidth = Units.inchesToMeters(21.819);
    public static final double srx_mag_units_per_revolution = 4096.0;
    public static final double drive_wheel_diameter_meters = Units.inchesToMeters(6.0);
    public static final double drivebase_max_voltage = 10.0;

}
