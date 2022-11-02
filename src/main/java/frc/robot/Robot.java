// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  //Remember to change the imports to the correct ones for the robot that you are using
  private final PWMVictorSPX m_leftfrontDrive = new PWMVictorSPX(0);
  private final PWMVictorSPX m_rightfrontDrive = new PWMVictorSPX(3);
  private final PWMVictorSPX m_leftbackDrive = new PWMVictorSPX(1);
  private final PWMVictorSPX m_rightbackDrive = new PWMVictorSPX(2);
  private final MotorControllerGroup left_side = new MotorControllerGroup(m_leftfrontDrive, m_leftbackDrive);
  private final MotorControllerGroup right_side = new MotorControllerGroup(m_rightfrontDrive, m_rightbackDrive);
  private final Joystick m_leftStick = new Joystick(0);
  private final Joystick m_rightStick = new Joystick(1);
  private DifferentialDrive m_myRobot;

  
   @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    right_side.setInverted(true);

    m_myRobot = new DifferentialDrive(left_side, right_side);
  }

  @Override
  public void teleopPeriodic() {
    m_myRobot.tankDrive(m_leftStick.getY(), m_rightStick.getY());
  }
}