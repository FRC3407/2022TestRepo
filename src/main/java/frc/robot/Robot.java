// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


/**this is a code, a drive code
 * made by patricasplitlump(seila)...sorta.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick = new Joystick(1);
  private Joystick m_rightStick  = new Joystick(2);

  private final MotorController m_leftMotor = new WPI_TalonSRX(0);
  private final MotorController m_rightMotor = new WPI_TalonSRX(2);

  private final MotorController m_leftMotor_2 = new WPI_TalonSRX(1);
  private final MotorController m_rightMotor_2 = new WPI_TalonSRX(3);
  //Motor values might work, they might not, who knows, because i dont

  private final MotorControllerGroup left = new MotorControllerGroup(m_leftMotor, m_leftMotor_2);
  private final MotorControllerGroup right = new MotorControllerGroup(m_rightMotor, m_rightMotor_2);

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    m_myRobot = new DifferentialDrive(left, right);
    m_leftStick = new Joystick(1);
    m_rightStick = new Joystick(2);
  }

  @Override
  public void teleopPeriodic() {
    m_myRobot.tankDrive(m_leftStick.getY(), m_rightStick.getY());
  }
}
