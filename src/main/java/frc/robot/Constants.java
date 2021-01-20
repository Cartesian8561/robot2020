// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int joystickport = 0;

    

	public static final int fronrightid = 0;
	public static final int backleftid = 0;
	public static final int backrightid = 0;
	public static final int frontleftid = 0;

    
    
    //Joysticks Values
    public static final int yaxis = 2;
    public static int xaxis = 3;
    // Joystik Inputs 
    public static final double ySpeed = RobotContainer.cartesJoystick.getRawAxis(Constants.yaxis);
    public static final double xRotation = RobotContainer.cartesJoystick.getRawAxis(Constants.xaxis);



	public static final Port gyroid = null;
    public static final double kEncoderSPR = 0;
	public static final double wheelDiameter = 0;
	public static final double kMaxSpeedMetersPerSecond = 2;
	public static final double kMaxAccelerationMetersPerSecondSquared = 1;
	public static final DifferentialDriveKinematics kDriveKinematics = null;


    //Robot Characteization Tool
	public static final double kaVoltSecondsSquaredPerMeter = 0;
    public static final double ksVolts = 0;
    public static final double kvVoltSecondsPerMeter = 0;



	

}
