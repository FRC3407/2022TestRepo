package frc.robot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.ADIS16470;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

public class CLDriveBase extends SubsystemBase {

    private final WPI_TalonSRX 
        leftf = new WPI_TalonSRX(0),
        leftb = new WPI_TalonSRX(1),
        rightf = new WPI_TalonSRX(3),
        rightb = new WPI_TalonSRX(2);

    private final ADIS16470 gyro = new ADIS16470();

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
        constants.drivebase_kS, 
        constants.drivebase_kV, 
        constants.drivebase_kA
    );

    private final DifferentialDriveOdometry odometry;
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(constants.trackwidth);;

    public CLDriveBase(){
        this.odometry = new DifferentialDriveOdometry(gyro.getRotation2d()); // gets angle so it knows where to start from

        leftf.configFactoryDefault(); // sets all to factory default value
        leftb.configFactoryDefault(); 
        rightf.configFactoryDefault(); 
        rightb.configFactoryDefault(); 

        leftf.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative); // telling the motor controller to get values from encoder
        rightf.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative); 

        leftf.setSelectedSensorPosition(0.0);
        rightf.setSelectedSensorPosition(0.0);

        leftf.setSensorPhase(false); 
        rightf.setSensorPhase(true); // inverts right side

        leftb.follow(leftf); // sets motors to follow the front motors cause they're no motor controller groups
        rightb.follow(rightf);
        leftb.setInverted(InvertType.FollowMaster); // inverts these too
        rightb.setInverted(InvertType.FollowMaster);

        








    }

public Pose2d getCurrentPose() { // in meters
    return this.odometry.getPoseMeters();

}

public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(this.getLeftVelocity(), this.getRightVelocity());

}
 
public double getRawLeftPosition() {
		return this.leftf.getSelectedSensorPosition();
	}
	public double getRawRightPosition() {
		return this.rightf.getSelectedSensorPosition();
	}
	public double getRawLeftVelocity() {
		return this.leftf.getSelectedSensorVelocity();
	}
	public double getRawRightVelocity() {
		return this.rightf.getSelectedSensorVelocity();
	}


public double getLeftPositionMeters() {
	return this.getRawLeftPosition()
        / constants.srx_mag_units_per_revolution
        * constants.drive_wheel_diameter_meters * Math.PI;
}
public double getRightPositionMeters() {
    return this.getRawRightPosition()
        / constants.srx_mag_units_per_revolution
        * constants.drive_wheel_diameter_meters * Math.PI;

}
public double getRightVelocity() {
    return this.getRawRightVelocity()
    *10
    / constants.srx_mag_units_per_revolution
    * constants.drive_wheel_diameter_meters * Math.PI;

}
public double getLeftVelocity() {
    return this.getRawLeftVelocity()
    *10
    / constants.srx_mag_units_per_revolution
    * constants.drive_wheel_diameter_meters * Math.PI;

}
public double getcontinuousAngle() { // in degrees
    return this.gyro.getAngle();

}
public double getHeading() { // from 180 to -180 (left or right)
    return this.gyro.getRotation2d().getDegrees();

}

public double getTurnRate() { // in degrees per sec
    return -this.gyro.getRate();

}
public Rotation2d getRotation2d() { 
    return this.gyro.getRotation2d();

}
public DifferentialDriveVoltageConstraint getVoltageConstraint() {
    return new DifferentialDriveVoltageConstraint(
        this.feedforward, this.kinematics, constants.drivebase_max_voltage
        );

}
public TrajectoryConfig getTrajectoryConfig(){
    return new TrajectoryConfig(
        constants.kMaxSpeedMetersPerSecond, 
        constants.kMaxAccelerationMetersPerSecondSquared
        ).setKinematics(
            this.kinematics
        ).addConstraint(this.getVoltageConstraint()
        );
}
/* "Setters" -> require command key
	public void resetOdometry(Pose2d p, CLDriveCommand c) {
		this.resetEncoders(c);
		this.position_offset = this.getTotalPose();
		this.odometry.resetPosition(p, this.getRotation());
	}
	public void setDriveVoltage(double lv, double rv, CLDriveCommand c) {
		this.left.setVoltage(lv);
		this.right.setVoltage(rv);
		super.getDrive().feed();
	}
	public void resetEncoders(CLDriveCommand c) {
		this.left.setSelectedSensorPosition(0.0);
		this.right.setSelectedSensorPosition(0.0);
	}
	public void zeroHeading(CLDriveCommand c) {
		this.gyro.reset();
	}

	public void setInitial(Pose2d init) {	// set the initial position based on where the robot is located on the field - used primarily for accurate dashboard view
		if(this.position_offset.equals(new Pose2d())) {
			this.position_offset = init;
		}
	}*/


public static class FollowTrajectory extends CLDriveCommands {
    private final Trajectory trajectory;




}





}


