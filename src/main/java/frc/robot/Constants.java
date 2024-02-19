// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
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
    public static final double DRIVETRAIN_TRACKWIDTH_METERS             = Units.inchesToMeters(32);
    public static final double DRIVETRAIN_WHEELBASE_METERS              = Units.inchesToMeters(28);
    public static final double DRIVETRAIN_GEAR_RATIO                    = 5.12; //For L4 Gear Ratio
    public static final double WHEEL_DIAMETER                           = Units.inchesToMeters(4);

    public static final double MAX_DRIVE_BASE_RADIUS = Math.sqrt(Math.pow((DRIVETRAIN_TRACKWIDTH_METERS/2), 2) + Math.pow((DRIVETRAIN_WHEELBASE_METERS/2), 2));
    
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

    public static final double MAX_MOVE_VELOCITY                        = 4; // for testing
    public static final double MAX_TURN_VELOCITY                        = 4; // for testing
    public static final double MAX_MOVE_VELOCITY_FAST                   = 5.5;
    public static final double MAX_TURN_VELOCITY_FAST                   = 5.5;
    
  // Front left Swerve Module
  public static final int FRONT_LEFT_DRIVE_ID                         = 21;
  public static final int FRONT_LEFT_TURN_ID                          = 22;
  public static final int FRONT_LEFT_ENCODER_ID                       = 23;
  public static final boolean FRONT_LEFT_DRIVE_MOTOR_INVERT           = true;
  public static final Rotation2d FRONT_LEFT_TURN_OFFSET               = Rotation2d.fromRotations(0.874755859375); // 359 is good

  // Back left Swerve Module
  public static final int BACK_LEFT_DRIVE_ID                          = 31;  // 31 is og
  public static final int BACK_LEFT_TURN_ID                           = 32;  // 32 is og
  public static final int BACK_LEFT_ENCODER_ID                        = 33;  // 33 is og
  public static final boolean BACK_LEFT_DRIVE_MOTOR_INVERT            = true;
  public static final Rotation2d BACK_LEFT_TURN_OFFSET                = Rotation2d.fromRotations(0.990234375); // 358.5 is good

  // Front Right Swerve Module
  public static final int FRONT_RIGHT_DRIVE_ID                        = 24;
  public static final int FRONT_RIGHT_TURN_ID                         = 25;
  public static final int FRONT_RIGHT_ENCODER_ID                      = 26;
  public static final boolean FRONT_RIGHT_DRIVE_MOTOR_INVERT          = true;
  public static final Rotation2d FRONT_RIGHT_TURN_OFFSET              = Rotation2d.fromRotations(-0.65673828125); // 1 is good

  // Back Right Swerve Module
  public static final int BACK_RIGHT_DRIVE_ID                         = 34;  // 34 is og
  public static final int BACK_RIGHT_TURN_ID                          = 35;  // 35 is og
  public static final int BACK_RIGHT_ENCODER_ID                       = 36;  // 36 is og
  public static final boolean BACK_RIGHT_DRIVE_MOTOR_INVERT           = true;
  public static final Rotation2d BACK_RIGHT_TURN_OFFSET               = Rotation2d.fromRotations(0.6787109375); // 0 is good

    public static final double DRIVE_P                                  = 0.1;
    public static final double DRIVE_I                                  = 0;
    public static final double DRIVE_D                                  = 0;
    public static final double DRIVE_FF                                 = 2.96;

    public static final double TURN_P                                   = 10;
    public static final double TURN_I                                   = 0;
    public static final double TURN_D                                   = 0;
    public static final double TURN_FF                                  = 0;

    public static final int DRIVER_CONTROLLER_PORT                      = 0;
 }

 public static final class VisionConstants {

        public static final String[] cameraNames = {
            "FL", 
            "FR", 
            "BL", 
            "BR",
            "Camera_Module_v1"
        };

        public static final Transform3d[] vehicleToCameras = {//10 deg yaw, 5 deg pitch
            new Transform3d(new Translation3d(0.03, 0.146, Units.inchesToMeters(31.5)), new Rotation3d(0, Units.degreesToRadians(-5), Units.degreesToRadians(-10))),
            new Transform3d(new Translation3d(0.03, -0.146, Units.inchesToMeters(31.5)), new Rotation3d(0, Units.degreesToRadians(-5), Units.degreesToRadians(10))),
            new Transform3d(new Translation3d(-0.03, 0.146, Units.inchesToMeters(31.5)), new Rotation3d(0, Units.degreesToRadians(-5), Units.degreesToRadians(-175))),
            new Transform3d(new Translation3d(-0.03, -0.146, Units.inchesToMeters(31.5)), new Rotation3d(0, Units.degreesToRadians(-5), Units.degreesToRadians(175)))
        };

        public static final double SINGLE_TAG_AMBIGUITY_CUTOFF                  = 0.05;
        public static final double MINIMUM_STANDARD_DEVIATION                   = 0.3;
        public static final double EULER_MULTIPLIER                             = 0.25;
        public static final double DISTANCE_MULTIPLIER                          = 0.4;

    }

    public static final class AutoConstants {
        public static final double AUTO_DRIVE_P                                  = 5;
        public static final double AUTO_DRIVE_I                                  = 0;
        public static final double AUTO_DRIVE_D                                  = 0;

        public static final double AUTO_TURN_P                                   = 9;
        public static final double AUTO_TURN_I                                   = 0;
        public static final double AUTO_TURN_D                                   = 0;

        public static final HolonomicPathFollowerConfig HOLONOMIC_PATH_FOLLOWER_CONFIG = 
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(AUTO_DRIVE_P, AUTO_DRIVE_I, AUTO_DRIVE_D), // Translation PID constants
            new PIDConstants(AUTO_TURN_P, AUTO_TURN_I, AUTO_TURN_D), // Rotation PID constants
            Constants.DriveConstants.MAX_MOVE_VELOCITY, // Max module speed, in m/s
            DriveConstants.MAX_DRIVE_BASE_RADIUS, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        );

    }

    public static final class PivotConstants {
        public static final int PIVOT_MOTOR_ID                                = 8;

        public static final double PIVOT_GEAR_RATIO                           = 48; // 48 motor rev : 1 arm rev

        public static final double HOME_PIVOT_ANGLE                           = 35; //off vertical
        public static final double AMP_PIVOT_ANGLE                            = 73; //off vertical
        public static final double TRAP_PIVOT_ANGLE                           = 92; //off vertical
        public static final double SOURCE_PIVOT_ANGLE                         = 81; //off vertical
        public static final double PIVOT_ANGLE_THRESHOLD                      = 0.75; // in degrees

        public static final double SPEAKER_CLOSE_PIVOT_ANGLE                  = 28; //off vertical

        public static final double PIVOT_P                                    = 0.5;
        public static final double PIVOT_I                                    = 0;
        public static final double PIVOT_D                                    = 0;

        public static final int PIVOT_BRAKE_FORWARD_CHANNEL                   = 4;
        public static final int PIVOT_BRAKE_REVERSE_CHANNEL                   = 5;
    }

    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR_ID                               = 15;
        public static final double INTAKE_MOTOR_SPEED                         = 0.5;
    }

    public static final class ShooterConstants {
        public static final int TOP_SPEAKER_ROLLER_MOTOR_ID                   = 10;
        public static final int BOTTOM_SPEAKER_ROLLER_MOTOR_ID                = 11;

        public static final double SPEAKER_SHOOTER_P                      = 0.5;
        public static final double SPEAKER_SHOOTER_I                      = 0;
        public static final double SPEAKER_SHOOTER_D                      = 0;
    }

    public static final class IndexerConstants {
        public static final int FEEDER_ROLLER_MOTOR_ID                        = 6;
        public static final int AMP_ROLLER_MOTOR_ID                           = 5;

        public static final double FEEDER_ROLLER_SPEED                        = 0.5;
        public static final double AMP_ROLLERS_ROLLER_SPEED                   = 0.5;
    }

    public static final class ClimberConstants {
        public static final int LEFT_HOOK_MOTOR_ID                            = 57;
        public static final int RIGHT_HOOK_MOTOR_ID                           = 58;

        public static final double HOOK_P                                     = 0;
        public static final double HOOK_I                                     = 0;
        public static final double HOOK_D                                     = 0;

        public static final double CLIMBER_REV_THRESHOLD                      = 5;

        public static final double CLIMB_REV                              = 1000;

    }

    public static final class LEDConstants {
        public static final int LED_PORT                                      = 1;
        public static final int NUMBER_OF_LEDS                                = 50;
    }
}