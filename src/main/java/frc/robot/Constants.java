// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.simulation.SimConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class CanConstants {

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 4;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 5;
    public static final int FRONT_LEFT_MODULE_STEER_CANCODER = 6;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 232.56;// 231.5;// -Math.toRadians(0.0);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 7;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 8;
    public static final int FRONT_RIGHT_MODULE_STEER_CANCODER = 9;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 317.37;// -Math.toRadians(-42);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 10;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 11;
    public static final int BACK_LEFT_MODULE_STEER_CANCODER = 12;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = 181.9;// -Math.toRadians(0.0);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 13;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 14;
    public static final int BACK_RIGHT_MODULE_STEER_CANCODER = 15;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 253.83;// -Math.toRadians(-105);

    public static final int LIFT_ARM_MOTOR = 18;
    public static final int EXTEND_ARM_MOTOR = 19;
    public static final int WRIST_MOTOR = 20;
    public static final int INTAKE_MOTOR = 21;
    public static final int LIFT_CANCODER = 22;
    public static final int CUBE_SENSOR = 23;
    public static final int CONE_SENSOR = 24;

  }

  public class IDConstants {

    public static final int FRONT_LEFT_LOCATION = 0;
    public static final int FRONT_RIGHT_LOCATION = 1;
    public static final int REAR_LEFT_LOCATION = 2;
    public static final int REAR_RIGHT_LOCATION = 3;

  }

  public class PDPConstants {

    public static final int FRONT_LEFT_DRIVE_CHANNEL = 1;
    public static final int FRONT_RIGHT_DRIVE_CHANNEL = 1;
    public static final int BACK_LEFT_DRIVE_CHANNEL = 1;
    public static final int BACK_RIGHT_DRIVE_CHANNEL = 1;

    public static final int FRONT_LEFT_TURN_CHANNEL = 1;
    public static final int FRONT_RIGHT_TURN_CHANNEL = 1;
    public static final int BACK_LEFT_TURN_CHANNEL = 1;
    public static final int BACK_RIGHT_TURN_CHANNEL = 1;

  }

  public static final class DriveConstants {

    public static final boolean kFrontLeftTurningMotorReversed = true;
    public static final boolean kBackLeftTurningMotorReversed = true;
    public static final boolean kFrontRightTurningMotorReversed = true;
    public static final boolean kBackRightTurningMotorReversed = true;

    public static final boolean kFrontLeftDriveMotorReversed = false;
    public static final boolean kBackLeftDriveMotorReversed = false;
    public static final boolean kFrontRightDriveMotorReversed = true;
    public static final boolean kBackRightDriveMotorReversed = true;

    public static final double kTrackWidth = Units.inchesToMeters(22);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(27);

    private final static Translation2d m_frontLeftLocation = new Translation2d(kWheelBase / 2, kTrackWidth / 2);
    private final static Translation2d m_frontRightLocation = new Translation2d(kWheelBase / 2, -kTrackWidth / 2);
    private final static Translation2d m_backLeftLocation = new Translation2d(-kWheelBase / 2, kTrackWidth / 2);
    private final static Translation2d m_backRightLocation = new Translation2d(-kWheelBase / 2, -kTrackWidth / 2);

    public static final Translation2d[] kModuleTranslations = {

        m_frontLeftLocation,
        m_frontRightLocation,
        m_backLeftLocation,
        m_backRightLocation };

    public final static SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    public static final boolean kGyroReversed = true;

    public static final double kMaxSpeedMetersPerSecond = 3.25;

    public static final double kMaxRotationRadiansPerSecond = Math.PI / 2;

    public static final double kMaxRotationRadiansPerSecondSquared = Math.PI;

    public static double kPhysicalMaxSpeedMetersPerSecond = 3.25;

    public static int kPhysicalMaxAngularSpeedRadiansPerSecond = 3;

  }

  public static final class DriverConstants {

    public static double kTranslationSlew = 50.0;
    public static double kRotationSlew = 1.0;
    public static double kControllerDeadband = .025;
    public static double kControllerRotDeadband = .1;

  }

  public static final class ModuleConstants {

    // ModuleConfiguration MK4I_L1

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);

    public static double mk4iL1DriveGearRatio = 1 / ((14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0));// 8.14 .122807

    public static double mk4iL1TurnGearRatio = 1 / ((14.0 / 50.0) * (10.0 / 60.0));// 21.43 1/.046667

    public static final double kDriveMetersPerEncRev =

        (kWheelDiameterMeters * Math.PI) / mk4iL1DriveGearRatio;// 0.039198257811106

    public static double kEncoderRevsPerMeter = 1 / kDriveMetersPerEncRev;// 25.511337897182322

    public static final double kTurningDegreesPerEncRev =

        360 / mk4iL1TurnGearRatio;

    // public static final double kTurningRadiansPerEncoderRev =
    // Units.degreesToRadians(kTurningDegreesPerEncRev);

    // max turn speed = (5400/ 21.43) revs per min 240 revs per min 4250 deg per
    // min

    public static double kVoltCompensation = 12.6;

    public static double kSMmaxAccel = 90;// deg per sec per sec

    public static double maxVel = 90; // deg per sec

    public static double allowedErr = .05;// deg

    public static double kMaxModuleAngularSpeedDegPerSec = 90;

    public static final double kMaxModuleAngularAccelerationDegreesPerSecondSquared = 90;

  }

  public static final class LiftArmConstants {

    public static double GEAR_RATIO = 21;

    public static double PULLEY_TEETH = 24;

    public static double TOOTH_BELT_PITCH = Units.metersToInches(.005);// .197"

    public static final double MIN_ANGLE = 31;

    public static final double MAX_ANGLE = 105;

    public static final double MIN_INCHES = 0;

    public static final double MAX_INCHES = 16;

    public static final double MAX_CANCODER = 98;

    public static final double MIN_CANCODER = 31;

    private static final double ANGLE_RANGE = MAX_ANGLE - MIN_ANGLE;// 56

    private static final double INCH_RANGE = MAX_INCHES - MIN_INCHES;// 15



    public static final double INCHES_PER_ENCODER_REV = (TOOTH_BELT_PITCH * PULLEY_TEETH) / GEAR_RATIO;// .225


    public static final double MAX_RATE_INCHES_PER_SEC = (INCHES_PER_ENCODER_REV * 5700) / 60;// 21.4


    public static final double LIFT_CANCODER_OFFSET = -204;


    // if arm driven direct through 21:1 reduction arm speed = 5700/(60*21)= 4.5
    // revs per sec = 1628 deg per sec
    // compare to 80 degrees per sec gives an additional 20:1 ratio

    // arm feedforward
    public static final double ksVolts = .6;//

    public static final double kGVolts = .52;

    public static final TrapezoidProfile.Constraints liftArmInchConstraints

        = new Constraints(5, 5);

    public static final double JOG_SLEW_RATE = 10;

    public static final double kControllerDeadband = 0.05;

    public static final double kvVoltSecondsPerInch = .5;

    public static final double kAVoltSecondSquaredPerInch = 0;

  }

  //
  public static final class ExtendArmConstants {

    public static double GEAR_RATIO = 49;
    public static double PULLEY_TEETH = 24;
    public static double TOOTH_BELT_PITCH = Units.metersToInches(.005);// .2

    public static final double MIN_POSITION = -1;

    public static final double MAX_POSITION = 26;

    public static final double INCHES_PER_ENCODER_REV = TOOTH_BELT_PITCH * PULLEY_TEETH / GEAR_RATIO;// .1

    public static final double MAX_RATE_INCHES_PER_SEC = (INCHES_PER_ENCODER_REV * 11000) / 60;// 18

    public static double ksExtArmVolts = .1;

    public static double kvExtArmVoltSecondsPerInch = .6;// .68 max .34 gave just over 50% try .5

    public static double kaExtArmVoltSecondsSquaredPerInch = 0;

    public static double kControllerDeadband = 0.05;

    public static final TrapezoidProfile.Constraints extendArmConstraints

        = new Constraints(15, 30);

    public static final TrapezoidProfile.Constraints intakeConstraints

        = new Constraints(7.5, 10);

    public static final TrapezoidProfile.Constraints deliverConstraints

        = new Constraints(7.5, 10);

    public static final double JOG_SLEW_RATE = 10;

    public static final double kgVolts = 0.25;

  }

  // max speed of 20 deg per sec
  // so 900 revs off motor = 20 degree

  public static final class WristConstants {

    public static final double GEAR_RATIO = 700;

    public static final double MOTOR_PULLEY_TEETH = 24;

    public static final double SHAFT_PULLEY_TEETH = 38;

    public static final double NET_GEAR_RATIO = GEAR_RATIO * SHAFT_PULLEY_TEETH / MOTOR_PULLEY_TEETH;// 38 *700 /24
                                                                                                     // =1108

    public static final double DEGREES_PER_ENCODER_REV = 360 / NET_GEAR_RATIO;// 360/1108 .324

    public static final double RADIANS_PER_ENCODER_REV = Units.degreesToRadians(DEGREES_PER_ENCODER_REV);

    public static final double MAX_DEGREES_PER_SEC = DEGREES_PER_ENCODER_REV * 11000 / 60;// 60

    public static final double MAX_RADS_PER_SEC = Units.degreesToRadians(MAX_DEGREES_PER_SEC);

    public static final double MIN_ANGLE = 50;

    public static final double MAX_ANGLE = 250;

    public static final TrapezoidProfile.Constraints wristConstraints =

        new Constraints(Units.degreesToRadians(20), Units.degreesToRadians(40));

    public static final double JOG_SLEW_RATE = 10;

    public static final double kControllerDeadband = 0.05;

    public static double ksVolts = .06;

    public static double kgVolts = .16;

    public static double kvWristVoltSecondsPerRadian = 10;// Math.PI * 4;

    public static double kaWristVoltSecondsSquaredPerRadian;

  }

  public static final class CurrentLimitConstants {

    public static final int turnMotorSmartLimit = 20;

    public static final int driveMotorSmartLimit = 20;

  }

  public final static class ModuleTuneConstants {

    public static final double kPModuleDriveController = .002;
    public static final double kIModuleDriveController = 0;
    public static final double kDModuleDriveController = 0;

    public static final double kPModuleTurningController = .004;
    public static final double kIModuleTurningController = 0;
    public static final double kDModuleTurningController = 0;

  }

  public static final class SYSIDConstants {
    // from Beta test
    public static final double ksDriveVoltSecondsPerMeter = .0927;
    public static final double kvDriveVoltSecondsSquaredPerMeter = 3.13;
    public static final double kaDriveVoltSecondsSquaredPerMeter = 0.82;
    // sysid on module?
    public static final double kvTurnVoltSecondsPerRadian = 1.47; // originally 1.5
    public static final double kaTurnVoltSecondsSquaredPerRadian = 0.348; // originally 0.3

  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 1;
    public static final int kArmControllerPort = 3;
    public static final int kTestControllerPort = 4;

  }

  public static final class PPConstants {
    public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond
        / 10;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    public static final double kPXController = 2;
    public static final double kDXController = 0;
    public static final double kIXController = 0;

    public static final double kPYController = 2;
    public static final double kDYController = 0;
    public static final double kIYController = 0;

    public static final double kPThetaController = 0.1;
    public static final double kDThetaController = 0;
    public static final double kIThetaController = 0;

    public static final double kPRotateController = 0.025;
    public static final double kDRotateController = .0;
    public static final double kIRotateController = 0;

    public static final double kPStrafeController = 0.8;
    public static final double kDStrafeController = .0;
    public static final double kIStrafeController = 0;

  }

  public static final class drToTgtConstants {

    public static final double strkP = 1;
    public static final double strkD = 0;
    public static final double strkI = 0;

    public static final double rotkP = 1;
    public static final double rotkD = 0;
    public static final double rotkI = 0;

  }

  public static class VisionConstants {

    public static final double tapeTyMidLevel = 5;

    public static final double tapeTyUpperLevel = 7;

  }

  public static final class LEDConstants {

    public static final int LED_CONTROLLER_PORT = 1;
  }

  public static double redBlueYShift = Units.inchesToMeters(100);

  public static final Transform2d redToBlueTransform = new Transform2d(new Translation2d(0, redBlueYShift),
      new Rotation2d());

  public static class LoadStationPickupConstants {

    static Pose2d aprilTag5 = SimConstants.Tags.aprilTags[3].toPose2d();

    static Transform2d rightPickupT2d = new Transform2d(new Translation2d(0, .8), new Rotation2d());

    static Transform2d leftPickupT2d = new Transform2d(new Translation2d(0, -.8), new Rotation2d());

    public static Pose2d blueLeftTarget = aprilTag5.plus(rightPickupT2d);

    public static Pose2d blueRightTarget = aprilTag5.plus(leftPickupT2d);

    static Pose2d aprilTag6 = SimConstants.Tags.aprilTags[4].toPose2d();

    public static Pose2d redLeftTarget = aprilTag6.plus(rightPickupT2d);

    public static Pose2d redRightTarget = aprilTag6.plus(leftPickupT2d);

  }
}