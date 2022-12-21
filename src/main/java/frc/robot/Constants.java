package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final double max_velocity_meters_per_sec = 1.5;
    public static final double max_acceleration_meters_per_sec_sqrd = 1.5;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double drivebase_kS = 1.1185;	// "kS"(volts) -> base voltage required to overcome static friction -> from SysID characterization
    public static final double drivebase_kV = 2.1132;	// "kV"(volts * seconds / meters) -> voltage required for each additional meter/second of velocity -> from SysID characterization
    public static final double drivebase_kA = 1.0668;	// "kA"(volts * seconds^2 / meters) -> voltage required for each additional meter/second^2 of acceleration -> from SysID characterization
    public static final double drivebase_kP = 3.5176;	// "kP"(volts * seconds / meters) -> voltage added to correct for error

    public static final double trackwidth = Units.inchesToMeters(21.819);
    public static final double srx_mag_units_per_revolution = 4096.0;

    public static final double drivewheel_diameter_meters = Units.inchesToMeters(6.0);
    public static final double drivebase_max_voltage = 10.0; 

    public static final double ramsete_B = 2.0;
    public static final double ramsete_Zeta = 0.7;



    
}
