// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oi;

import java.util.Map;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftArmSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem.GridDrop;
import frc.robot.subsystems.GameHandlerSubsystem.gamePiece;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.utils.AutoFactory;

/** Add your docs here. */
public class ShuffleboardCompetition {

        private LimelightVision m_llv;

        private DriveSubsystem m_drive;

        private GameHandlerSubsystem m_gps;

        private AutoFactory m_af;

        private IntakeSubsystem m_intake;

        public ShuffleboardCompetition(LimelightVision llv, DriveSubsystem drive, GameHandlerSubsystem gps,
                        AutoFactory af, LiftArmSubsystem lift, ExtendArmSubsystem ext, WristSubsystem wrist,
                        IntakeSubsystem intake) {
                String name = "Competition";
                m_llv = llv;
                m_gps = gps;
                m_af = af;
                m_drive = drive;
                m_intake = intake;

                ShuffleboardTab area1 = Shuffleboard.getTab(name);

                area1.add("AutoChooser", m_af.m_autoChooser)
                                .withSize(2, 1)
                                .withPosition(0, 0);

                // area1.addString("Current Pipeline", () -> m_llv.getCurrentPipelineName())
                // .withSize(1, 1)
                // .withPosition(2, 0);

                area1.add("LevelChooser", m_af.m_pieceLevelChooser)
                                .withSize(1, 1)
                                .withPosition(2, 0);

                area1.addString(" PipelineType", () -> m_llv.getCurrentPipelineTypeName())
                                .withSize(1, 1).withPosition(3, 0);

                // area1.addBoolean("HasTag", () -> m_drive.hasTag)
                // .withPosition(0, 1);

                area1.add("DropChooser", m_af.m_startLocationChooser)
                                .withSize(2, 1)
                                .withPosition(0, 1);

                area1.addNumber("FidID", () -> m_drive.fiducialID)
                                .withPosition(0, 2);

                area1.addBoolean("InhibitVision", () -> m_drive.inhibitVision)
                                .withPosition(0, 3);

                area1.addBoolean("LimelightOK", () -> m_drive.limelightExists)
                                .withPosition(0, 4);

                area1.addBoolean("Alliance", () -> m_drive.getAllianceBlue())
                                .withWidget(BuiltInWidgets.kBooleanBox).withSize(1, 1)
                                .withPosition(9, 4)
                                .withProperties(Map.of("colorwhenfalse", "red", "colorwhentrue", "blue"));

                // area1.addBoolean("HasTarget", () -> m_drive.hasTarget)
                // .withPosition(1, 1);

                area1.addNumber("TX Degrees", () -> round2dp(m_drive.tx))
                                .withPosition(1, 2);

                area1.addNumber("TY Degrees", () -> round2dp(m_drive.ty))
                                .withPosition(1, 3);
                area1.addBoolean("CAN OK", () -> m_gps.CANOK)
                                .withPosition(1, 4);

                area1.addNumber("ConeDistance", () -> m_intake.getConeSensorDistance())
                                .withSize(1, 1)
                                .withPosition(2, 2);

                area1.addNumber("CubeDistance", () -> m_intake.getCubeSensorDistance())
                                .withSize(1, 1)
                                .withPosition(2, 3);

                area1.addBoolean(GridDrop.LEFT_CUBE.name(), () -> m_gps.drops[1])

                                .withPosition(3, 1).withSize(1, 1)

                                .withWidget(BuiltInWidgets.kBooleanBox)

                                .withProperties(Map.of("colorwhentrue", "green"));

                area1.addBoolean(GridDrop.LEFT_PIPE.name(), () -> m_gps.drops[2])

                                .withPosition(4, 1).withSize(1, 1)

                                .withWidget(BuiltInWidgets.kBooleanBox)

                                .withProperties(Map.of("colorwhentrue", "green"));

                area1.addBoolean(GridDrop.COOP_LEFT_PIPE.name(), () -> m_gps.drops[3])

                                .withPosition(5, 1).withSize(1, 1)

                                .withWidget(BuiltInWidgets.kBooleanBox)

                                .withProperties(Map.of("colorwhentrue", "green"));

                area1.addBoolean(GridDrop.COOP_CUBE.name(), () -> m_gps.drops[4])

                                .withPosition(6, 1).withSize(1, 1)

                                .withWidget(BuiltInWidgets.kBooleanBox)

                                .withProperties(Map.of("colorwhentrue", "green"));

                area1.addBoolean(GridDrop.COOP_RIGHT_PIPE.name(), () -> m_gps.drops[5])

                                .withPosition(7, 1).withSize(1, 1)

                                .withWidget(BuiltInWidgets.kBooleanBox)

                                .withProperties(Map.of("colorwhentrue", "green"));

                area1.addBoolean(GridDrop.RIGHT_PIPE.name(), () -> m_gps.drops[6])

                                .withPosition(8, 1).withSize(1, 1)

                                .withWidget(BuiltInWidgets.kBooleanBox)

                                .withProperties(Map.of("colorwhentrue", "green"));

                area1.addBoolean(GridDrop.RIGHT_CUBE.name(), () -> m_gps.drops[7])

                                .withPosition(9, 1).withSize(1, 1)

                                .withWidget(BuiltInWidgets.kBooleanBox)

                                .withProperties(Map.of("colorwhentrue", "green"));

                area1.addBoolean("Next Gamepiece", () -> m_gps.gamePieceType.equals(gamePiece.CONE))
                                .withWidget(BuiltInWidgets.kBooleanBox).withSize(2, 1)
                                .withPosition(8, 0)
                                .withProperties(Map.of("colorwhenfalse", "purple", "colorwhentrue", "yellow"));

                area1.addString("NextLevel", () -> m_gps.getDropOffLevel().name())
                                .withPosition(7, 0).withSize(1, 1)
                                .withWidget(BuiltInWidgets.kTextView);

                area1.addBoolean("RobotHasCone", () -> m_intake.conePresent)
                                .withWidget(BuiltInWidgets.kBooleanBox).withSize(2, 1)
                                .withPosition(4, 0).withSize(1, 1)
                                .withProperties(Map.of("colorwhenfalse", "gray", "colorwhentrue", "yellow"));

                area1.addBoolean("RobotHasCube", () -> m_intake.cubePresent)
                                .withWidget(BuiltInWidgets.kBooleanBox).withSize(2, 1)
                                .withPosition(5, 0).withSize(1, 1)
                                .withProperties(Map.of("colorwhenfalse", "gray", "colorwhentrue", "purple"));

                area1.addNumber("RobotX", () -> round2dp(m_drive.getX()))
                                .withPosition(6, 2).withSize(1, 1);

                area1.addNumber("RobotY", () -> round2dp(m_drive.getY()))
                                .withPosition(7, 2).withSize(1, 1);

                area1.addNumber("Heading", () -> round2dp(m_drive.getEstimatedPosition().getRotation().getDegrees()))
                                .withPosition(8, 2).withSize(1, 1);

                area1.addNumber("ActiveY", () -> m_gps.getActiveDropPose().getY())
                                .withPosition(9, 2);
                area1.addNumber("LiftDlvrAngle", () -> round2dp(Units.radiansToDegrees(lift.deliverAngleRads)))
                                .withPosition(6, 3);
                area1.addNumber("ExtDlvrDist", () -> ext.deliverDistance)
                                .withPosition(7, 3);
                area1.addNumber("WristDlvrAngle", () -> round2dp(Units.radiansToDegrees(wrist.deliverAngleRads)))
                                .withPosition(8, 3);
                area1.addNumber("Intake Speed", () -> round2dp(intake.deliverSpeed))
                                .withPosition(9, 3).withSize(1, 1);
        }

        public double round2dp(double number) {
                number = Math.round(number * 100);
                number /= 100;
                return number;
        }
}
