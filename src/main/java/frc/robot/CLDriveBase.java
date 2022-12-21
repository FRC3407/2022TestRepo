package frc.robot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.ADIS16470;
import java.nio.file.Path;
import java.util.Arrays;
import edu.wpi.first.math.trajectory.TrajectoryUtil;

public class CLDriveBase extends SubsystemBase { //CL Stands for Closed Loop

    private final WPI_TalonSRX
        leftf = new WPI_TalonSRX(0),
        leftb = new WPI_TalonSRX(1),
        rightf = new WPI_TalonSRX(3),
        rightb = new WPI_TalonSRX(2);

    private final ADIS16470 gyro = new ADIS16470();

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
        Constants.drivebase_kS, 
        Constants.drivebase_kV, 
        Constants.drivebase_kA
    );

    private final DifferentialDriveOdometry odometry;
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.trackwidth);;

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

    public Pose2d getCurrentPose() {  // in meters // gets where it is on a 2d plane and direction, uses odometry to help
    return this.odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
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
    
    public double getLeftPositionMeters() { // convert number of ticks to meters
        return this.getRawLeftPosition()
            / Constants.srx_mag_units_per_revolution
            * Constants.drivewheel_diameter_meters * Math.PI;
		
	}
    public double getRightPositionMeters() { // convert number of ticks to meters
        return this.getRawRightPosition()
            / Constants.srx_mag_units_per_revolution
            * Constants.drivewheel_diameter_meters * Math.PI;
		
	}

    public double getLeftVelocity() {
		return this.getRawLeftVelocity()
            * 10
            / Constants.srx_mag_units_per_revolution
            * Constants.drivewheel_diameter_meters * Math.PI;
	}
	public double getRightVelocity() {
		return this.getRawRightVelocity()
            * 10
            / Constants.srx_mag_units_per_revolution
            * Constants.drivewheel_diameter_meters * Math.PI;
	}
	
    public double getContinuousAngle() { // in degrees
        return this.gyro.getAngle();
    }

    public double getHeading() { // from 180 to -180 // helps to figure out if youre facing left or right
        return this.gyro.getRotation2d().getDegrees();
    }

    public double getTurnRate() { // in degrees per second
        return -this.gyro.getRate(); // positive is counterclockwise 
    }

    public Rotation2d getRotation2d() {
        return this.gyro.getRotation2d();
    }

    public DifferentialDriveVoltageConstraint getVoltageConstraint(){
        return new DifferentialDriveVoltageConstraint(
            this.feedforward, this.kinematics, Constants.drivebase_max_voltage
            );
    }
    public TrajectoryConfig getTrajectoryConfig() {
        return new TrajectoryConfig(
            Constants.max_velocity_meters_per_sec,
            Constants.max_acceleration_meters_per_sec_sqrd
        ).setKinematics(
            this.kinematics
        ).addConstraint(
            this.getVoltageConstraint()
        );
    }

    // "Setters" -> require command key
	public void resetOdometry(Pose2d p, CLDriveCommand c) { 
		this.resetEncoders(c);
		// this.position_offset = this.getTotalPose(); keeps track of total 
		this.odometry.resetPosition(p, this.getRotation2d());
	}
	public void setDriveVoltage(double lv, double rv, CLDriveCommand c) {
		this.leftf.setVoltage(lv);
		this.rightf.setVoltage(rv);
		// super.getDrive().feed();  helps feed updates the motors
	}
	public void resetEncoders(CLDriveCommand c) {
		this.leftf.setSelectedSensorPosition(0.0);
		this.rightf.setSelectedSensorPosition(0.0);
	}
	public void zeroHeading(CLDriveCommand c) {  // resets the angle of the gyro
		this.gyro.reset();
	}

	// public void setInitial(Pose2d init) {	// set the initial position based on where the robot is located on the field - used primarily for accurate dashboard view
	// 	if(this.position_offset.equals(new Pose2d())) {
	// 		this.position_offset = init;
	// 	}
	// }

    public static abstract class CLDriveCommand extends CommandBase { // only classes that extend from this can access it 
        protected final CLDriveBase drivebase_cl;  

        protected CLDriveCommand(CLDriveBase db){
            this.drivebase_cl = db;
        }

        protected void resetOdometry(Pose2d p){
            this.drivebase_cl.resetOdometry(p, this);
        }
        protected void setDriveVoltage(double lv, double rv){
            this.drivebase_cl.setDriveVoltage(lv, rv, this);
        }
        protected void resetEncoders(){
            this.drivebase_cl.resetEncoders(this);
        }

        protected void zeroHeading() {
            this.drivebase_cl.zeroHeading(this);
        }

    }

    
    public static class FollowTrajectory extends CLDriveCommand {

        private final Trajectory trajectory;
        private final RamseteCommand controller;
        private final boolean stop;
        
        public FollowTrajectory(CLDriveBase db, Trajectory t) { this(db, t, false); }
        public FollowTrajectory(CLDriveBase db, Path json_path) { this(db, json_path, false); }

        public FollowTrajectory(CLDriveBase db, Trajectory t, boolean s){
            super(db);
            this.trajectory = t;
            this.stop = s;
            this.controller = new RamseteCommand (
                this.trajectory,
                super.drivebase_cl::getCurrentPose,
                new RamseteController(Constants.ramsete_B, Constants.ramsete_Zeta),
                super.drivebase_cl.feedforward,
                super.drivebase_cl.kinematics,
                super.drivebase_cl::getWheelSpeeds,
                new PIDController(Constants.drivebase_kP, 0, 0),
                new PIDController(Constants.drivebase_kP, 0, 0),
                super::setDriveVoltage

            );
            

        }

        public FollowTrajectory(CLDriveBase db, Path json_path, boolean s) {
            super(db);
			this.stop = s;
			Trajectory temp;
			try {
				temp = TrajectoryUtil.fromPathweaverJson(json_path);
			} catch(Exception e) {
				System.err.println("FAILED TO READ TRAJECTORY: " + json_path.toString() + " -> " + e.getMessage());
				temp = new Trajectory(Arrays.asList(new Trajectory.State()));	// do-nothing trajectory as placeholder
			}
			this.trajectory = temp;
			this.controller = new RamseteCommand (
				this.trajectory,
				super.drivebase_cl::getCurrentPose,
				new RamseteController(Constants.ramsete_B, Constants.ramsete_Zeta),
				super.drivebase_cl.feedforward,
				super.drivebase_cl.kinematics,
				super.drivebase_cl::getWheelSpeeds,
				new PIDController(Constants.drivebase_kP, 0, 0),
				new PIDController(Constants.drivebase_kP, 0, 0),
				super::setDriveVoltage
			);


        }



    }



}
