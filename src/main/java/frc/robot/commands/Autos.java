// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.PathPlannerAuto;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.Drivetrain.Drivetrain;

// public final class Autos {
//   Drivetrain drivetrain;
//   public Autos() {
//     drivetrain = RobotContainer.drivetrain;
//     AutoBuilder.configureHolonomic(
//       drivetrain::getPose, 
//       drivetrain::resetPose, 
//       drivetrain::getSpeeds, 
//       drivetrain::drive, 
//       Constants.AutoConstants.HOLONOMIC_PATH_FOLLOWER_CONFIG, 
//         () -> {
//                 var alliance = DriverStation.getAlliance();
//                 if (alliance.isPresent()) {
//                     return alliance.get() == DriverStation.Alliance.Red;
//                 }
//                 return false;
//               },
//         drivetrain);
//     }

//     public Command returnAutos() {
//       return new PathPlannerAuto("Test Auto");
//     }

       
// }