// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oi;

import java.util.Map;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftArmSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.utils.TrajectoryFactory;

/** Add your docs here. */
public class ShuffleboardArms {

        private LimelightVision m_llv;

        private LiftArmSubsystem m_lift;

        private ExtendArmSubsystem m_ext;

        private WristSubsystem m_wrist;

        private TrajectoryFactory m_tf;

        public ShuffleboardArms(LiftArmSubsystem lift,

                        ExtendArmSubsystem ext, WristSubsystem wrist,
                        IntakeSubsystem intake, TrajectoryFactory tf) {

                m_lift = lift;
                m_ext = ext;
                m_wrist = wrist;
                m_tf = tf;

                ShuffleboardLayout liftLayout = Shuffleboard.getTab("Arms")
                                .getLayout("LiftLayout", BuiltInLayouts.kList)
                                .withPosition(0, 0)
                                .withSize(2, 4).withProperties(Map.of("Label position", "LEFT"));

                liftLayout.addNumber("LiftPosition", () -> round2dp(m_lift.positioninches));
                liftLayout.addNumber("GoalInches", () -> round2dp(m_lift.goalInches));
                liftLayout.addNumber("CanCoderDeg", () -> round2dp(m_lift.cancoderPosition));
                liftLayout.addNumber("MotorOut", () -> round2dp(m_lift.appliedOutput));
                liftLayout.addNumber("Amps", () -> round2dp(m_lift.amps));

                liftLayout.addBoolean("Stopped", () -> m_lift.stopped)
                                .withWidget(BuiltInWidgets.kTextView);

                liftLayout.addBoolean("AtGoal", () -> m_lift.atGoal)
                                .withWidget(BuiltInWidgets.kTextView);

                liftLayout.addBoolean("LiftCANOK", () -> m_lift.liftArmMotorConnected)
                                .withWidget(BuiltInWidgets.kTextView);

                // liftLayout.add("LIPfPID", m_lift.m_liftController).withWidget("Profiled PID
                // Controller");

                ShuffleboardLayout extLayout = Shuffleboard.getTab("Arms")
                                .getLayout("ExtLayout", BuiltInLayouts.kList)
                                .withPosition(2, 0)
                                .withSize(2, 4).withProperties(Map.of("Label position", "LEFT"));

                extLayout.addNumber("ExtPositionInch", () -> round2dp(m_ext.positionInches));
                extLayout.addNumber("ExtAmps", () -> round2dp(m_ext.amps));
                extLayout.addNumber("GoalInches", () -> round2dp(m_ext.goalInches));
                extLayout.addNumber("MotorOut", () -> round2dp(m_ext.appliedOutput));
                extLayout.addNumber("VELIPS", () -> round2dp(m_ext.inchespersec));
                extLayout.addBoolean("Stopped", () -> m_ext.isStopped())
                                .withWidget(BuiltInWidgets.kTextView);
                extLayout.addBoolean("AtGoal", () -> m_ext.atGoal)
                                .withWidget(BuiltInWidgets.kTextView);

                extLayout.addBoolean("EXTCANOK", () -> m_ext.extendMotorConnected)
                                .withWidget(BuiltInWidgets.kTextView);

                // extLayout.add("EXPfPID", m_ext.m_extController).withWidget("Profiled PID
                // Controller");

                ShuffleboardLayout wristLayout = Shuffleboard.getTab("Arms")
                                .getLayout("WristLayout", BuiltInLayouts.kList)
                                .withPosition(4, 0)
                                .withSize(2, 4)
                                .withProperties(Map.of("Label position", "LEFT"));

                wristLayout.addNumber("WristPosRads", () -> round2dp(m_wrist.angleRadians));
                wristLayout.addNumber("WristPosDeg", () -> round2dp(m_wrist.angleDegrees));
                wristLayout.addNumber("WristGoal", () -> round2dp(m_wrist.goalAngleRadians));

                // wristLayout.addNumber("CommandRadPerSec", () -> m_wrist.commandRadPerSec);
                wristLayout.addNumber("WristAmps", () -> round2dp(m_wrist.amps));
                wristLayout.addNumber("WristVelRadPS", () -> round2dp(m_wrist.radspersec));

                wristLayout.addBoolean("WristCANOK", () -> m_wrist.wristMotorConnected)
                                .withWidget(BuiltInWidgets.kTextView);
                wristLayout.addBoolean("Stopped", () -> m_wrist.isStopped())
                                .withWidget(BuiltInWidgets.kTextView);
                wristLayout.addBoolean("AtGoal", () -> m_wrist.atGoal)
                                .withWidget(BuiltInWidgets.kTextView);
                // wristLayout.add("WRPfPID", m_wrist.m_wristController).withWidget("Profiled
                // PID Controller");

                ShuffleboardLayout intakeLayout = Shuffleboard.getTab("Arms")
                                .getLayout("IntakeLayout", BuiltInLayouts.kList)
                                .withPosition(6, 0)
                                .withSize(2, 2)
                                .withProperties(Map.of("Label position", "LEFT"));

                intakeLayout.add("StopIntake", new StopIntake(intake));

                intakeLayout.addNumber("Actual RPM", () -> round2dp(intake.rpm));

                intakeLayout.addNumber("Intake Amps", () -> round2dp(intake.amps));

                intakeLayout.addBoolean("IntakeCANOK", () -> intake.intakeMotorConnected)
                                .withWidget(BuiltInWidgets.kTextView);

                ShuffleboardTab deliverLayout = Shuffleboard.getTab("Arms");

                deliverLayout.addNumber("WristDelDeg", () -> round2dp(Units.radiansToDegrees(wrist.deliverAngleRads)))
                                .withPosition(6, 3).withSize(1, 1);

                deliverLayout.addNumber("ExtDelInch", () -> round2dp(ext.deliverDistance))
                                .withPosition(7, 2).withSize(1, 1);

                deliverLayout.addNumber("Intake Speed", () -> round2dp(intake.deliverSpeed))
                                .withPosition(7, 3).withSize(1, 1);

                ShuffleboardLayout trajLayout = Shuffleboard.getTab("Arms")
                                .getLayout("TrajLayout", BuiltInLayouts.kList)
                                .withPosition(8, 0)
                                .withSize(2, 1)
                                .withProperties(Map.of("Label position", "TOP"));

                trajLayout.add("TrajectoryChooser", tf.ppTrajChooser);

        }

        public double round2dp(double number) {
                number = Math.round(number * 100);
                number /= 100;
                return number;
        }
}
