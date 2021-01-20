// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;


import java.util.List;



import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.geometry.Pose2d;

/** Add your docs here. */
public class CartesyTrajectories {
    public Trajectory[] galacticsearch = new Trajectory[3];
    Constants m_cConstants;
    
    
    
    public CartesyTrajectories(){
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                                new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter,
                                                Constants.kaVoltSecondsSquaredPerMeter),
                                Constants.kDriveKinematics, 11); // 8
        TrajectoryConfig configForward = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
         m_cConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);
                        // Burayı anlat zencimn

        
       galacticsearch[0] = TrajectoryGenerator.generateTrajectory(List.of
        (new Pose2d(10.2, 16.6, new Rotation2d(0)),
        new Pose2d(24, 12, new Rotation2d(-Math.toRadians(0.8)))
        ,new Pose2d(36,17, new Rotation2d(0))), configForward);
        
        
        
        
        galacticsearch[1] = TrajectoryGenerator.generateTrajectory(
         List.of(new Pose2d(36,17, new Rotation2d(0)),
         new Pose2d(36, 12, new Rotation2d(Math.toRadians(175))), 
         new Pose2d(24.5, 18.2, new Rotation2d(Math.toRadians(178.9)))), 
        configForward); 
         
         
         
         
         galacticsearch[2] = TrajectoryGenerator.generateTrajectory(List.of( 
             new Pose2d(24.5, 18.2, new Rotation2d(Math.toRadians(178.9))),
             new Pose2d(10, 13, new Rotation2d(Math.toRadians(175))), 
             new Pose2d(10, 13, new Rotation2d(Math.toRadians(175)))), 
             configForward);
         
         
    }
}


