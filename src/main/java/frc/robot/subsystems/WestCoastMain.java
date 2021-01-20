// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.Driver;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WestCoastMain extends SubsystemBase {
  /** Creates a new WestCoastMain. */
    WPI_TalonSRX backleft;
    WPI_TalonSRX backright;
    WPI_TalonSRX frontleft;
    WPI_TalonSRX frontright;
    public DifferentialDrive driver;
    public AHRS gyro;
    static private int PIDIDX= 0;
    private double angular_velocity;
    public DifferentialDriveOdometry m_odometry; 
    
  public WestCoastMain() {
   //Motors
   backleft = new WPI_TalonSRX(Constants.backleftid);
   backright = new WPI_TalonSRX(Constants.backrightid);
   frontleft = new WPI_TalonSRX(Constants.frontleftid);
   frontright = new WPI_TalonSRX(Constants.fronrightid);
   backleft.follow(frontleft);
   backright.follow(frontright);
   // Encoder
   frontleft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor.CTRE_MagEncoder_Relative, PIDIDX, 10);
   frontright.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor.CTRE_MagEncoder_Relative, PIDIDX, 10);
   //Gyro
   gyro = new AHRS(SPI.Port.kMXP);
   //Groups
   SpeedControllerGroup rightmotors = new SpeedControllerGroup(backright, frontright);
   SpeedControllerGroup leftmotors = new SpeedControllerGroup(backleft, frontleft);
   
   DifferentialDrive driver = new DifferentialDrive(rightmotors, leftmotors);
   //Resetting
   resetEncoders();
   resetGyro();
   
  }
 
 public void arcadeDrive(double speed, double rotation){
   driver.arcadeDrive(speed, rotation);
 }
 //Reseting Functions
 public void resetGyro(){
   gyro.reset();
 }
 
 public void resetEncoders(){
   frontright.setSelectedSensorPosition(0);
   frontleft.setSelectedSensorPosition(0);
 }

 public Pose2d getPose2d(){
   return m_odometry.getPoseMeters();
 }
 public DifferentialDriveWheelSpeeds getWheelSpeeds(){
   return new DifferentialDriveWheelSpeeds(frontleft.getSelectedSensorVelocity()*
   (1.0/Constants.kEncoderSPR)*Constants.wheelDiameter, 
   (frontright.getSelectedSensorVelocity()*
   (1.0/Constants.kEncoderSPR)*Constants.wheelDiameter));
 }

 public double getrightencoderdistance(){
   return frontright.getSelectedSensorPosition() * 
   (1.0/Constants.kEncoderSPR) * Constants.wheelDiameter;
 }
 public double getleftencoderdistance(){
  return frontleft.getSelectedSensorPosition() * 
  (1/Constants.kEncoderSPR) * Constants.wheelDiameter;
}
public double getAverageEncoderDistance(){
  return (getrightencoderdistance() + getleftencoderdistance())/2;
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   angular_velocity=gyro.getRate();
   
   
  }

}
