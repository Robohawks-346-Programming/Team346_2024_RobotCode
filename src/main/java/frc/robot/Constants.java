// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants {
    public static final double DRIVETRAIN_TRACKWIDTH_METERS             = 0.8128; //32 in
    public static final double DRIVETRAIN_WHEELBASE_METERS              = 0.7112; //28 in
    public static final double DRIVETRAIN_GEAR_RATIO                    = 5.12; //For L4 Gear Ratio
    public static final double WHEEL_DIAMETER                           = 0.09398; // 3.7 in
    
    public static final double DRIVE_CONVERSION                         = (WHEEL_DIAMETER * Math.PI) / DRIVETRAIN_GEAR_RATIO;
    public static final double TURN_CONVERSION                          = 12.8;

    public static final SwerveDriveKinematics DRIVE_KINEMATICS          = 
        new SwerveDriveKinematics(
            new Translation2d(DRIVETRAIN_WHEELBASE_METERS/ 2.0, DRIVETRAIN_TRACKWIDTH_METERS / 2.0), // front left
            new Translation2d(DRIVETRAIN_WHEELBASE_METERS/ 2.0, -DRIVETRAIN_TRACKWIDTH_METERS / 2.0), // front right
            new Translation2d(-DRIVETRAIN_WHEELBASE_METERS/ 2.0, DRIVETRAIN_TRACKWIDTH_METERS / 2.0), // back left
            new Translation2d(-DRIVETRAIN_WHEELBASE_METERS/ 2.0, -DRIVETRAIN_TRACKWIDTH_METERS / 2.0) // back right
        );

    public static final int MAX_VOLTAGE                                 = 12;
    public static final int DRIVE_CURRENT_LIMIT                         = 60;
    public static final int TURN_CURRENT_LIMIT                          = 25;

    public static final boolean IS_FIELD_RELATIVE                       = true;

    public static final double MAX_MOVE_VELOCITY                        = 2; // for testing
    public static final double MAX_TURN_VELOCITY                        = 2; // for testing
    public static final double MAX_MOVE_VELOCITY_FAST                   = 5.5;
    public static final double MAX_TURN_VELOCITY_FAST                   = 5.5;
    
    public static final int FRONT_LEFT_DRIVE_ID                         = 9;
    public static final int FRONT_LEFT_TURN_ID                          = 11;
    public static final int FRONT_LEFT_ENCODER_ID                       = 1;
    public static final double FRONT_LEFT_TURN_OFFSET                   = 0;

    public static final int FRONT_RIGHT_DRIVE_ID                        = 7;
    public static final int FRONT_RIGHT_TURN_ID                         = 17;
    public static final int FRONT_RIGHT_ENCODER_ID                      = 2;
    public static final double FRONT_RIGHT_TURN_OFFSET                  = 0;

    public static final int BACK_LEFT_DRIVE_ID                          = 10;
    public static final int BACK_LEFT_TURN_ID                           = 41;
    public static final int BACK_LEFT_ENCODER_ID                        = 4;
    public static final double BACK_LEFT_TURN_OFFSET                    = 0;

    public static final int BACK_RIGHT_DRIVE_ID                         = 12;
    public static final int BACK_RIGHT_TURN_ID                          = 24;
    public static final int BACK_RIGHT_ENCODER_ID                       = 3;
    public static final double BACK_RIGHT_TURN_OFFSET                   = 0;

    public static final double DRIVE_P                                  = 0.1;
    public static final double DRIVE_I                                  = 0;
    public static final double DRIVE_D                                  = 0;
    public static final double DRIVE_FF                                 = 2.96;

    public static final double TURN_P                                   = 0.01;
    public static final double TURN_I                                   = 0;
    public static final double TURN_D                                   = 0.005;
    public static final double TURN_FF                                  = 0;

    public static final int DRIVER_CONTROLLER_PORT                      = 0;
 }

 public static final class VisionConstants {

        public static final String[] cameraNames = {
            "FrontLeft", 
            "FrontRight", 
            "BackLeft", 
            "BackRight"
        };

        public static final Transform3d[] vehicleToCameras = {//10 deg yaw, 5 deg pitch
            new Transform3d(new Translation3d(0.03, 0.146, Units.inchesToMeters(31.5)), new Rotation3d(0, Units.degreesToRadians(-5), Units.degreesToRadians(-10))),
            new Transform3d(new Translation3d(0.03, -0.146, Units.inchesToMeters(31.5)), new Rotation3d(0, Units.degreesToRadians(-5), Units.degreesToRadians(10))),
            new Transform3d(new Translation3d(-0.03, 0.146, Units.inchesToMeters(31.5)), new Rotation3d(0, Units.degreesToRadians(-5), Units.degreesToRadians(-175))),
            new Transform3d(new Translation3d(-0.03, -0.146, Units.inchesToMeters(31.5)), new Rotation3d(0, Units.degreesToRadians(-5), Units.degreesToRadians(175)))
        };

        public static final double fieldLength = 16.542;
        public static final double fieldWidth = 8.014;

        public static final double singleTagAmbiguityCutoff = 0.05;
        public static final double minimumStdDev = 0.3;
        public static final double eulerMultiplier = 0.25;
        public static final double distanceMultiplier = 0.4;

    }
}