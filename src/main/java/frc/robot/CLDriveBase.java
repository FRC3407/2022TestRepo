package frc.robot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.ADIS16470;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

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



 

}