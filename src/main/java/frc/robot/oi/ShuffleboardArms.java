// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oi;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

                liftLayout.addNumber("LiftPosition", () -> round2dp(m_lift.positionInches));
                liftLayout.addNumber("SliderInches", () -> round2dp(m_lift.targetInchesFromSlider));
                liftLayout.addNumber("LiftVel IPS", () -> round2dp(m_lift.inchespersec));
                liftLayout.addNumber("CanCoderPosn", () -> round2dp(m_lift.cancoderPosition));
                liftLayout.addNumber("CommandIPS", () -> round2dp(m_lift.commandIPS));

                liftLayout.addBoolean("LiftCANOK", () -> m_lift.checkCANOK())
                                .withWidget(BuiltInWidgets.kTextView);

                liftLayout.add("LIPfPID", m_lift.m_liftController).withWidget("Profiled PID Controller");

                ShuffleboardLayout extLayout = Shuffleboard.getTab("Arms")
                                .getLayout("ExtLayout", BuiltInLayouts.kList)
                                .withPosition(2, 0)
                                .withSize(2, 4).withProperties(Map.of("Label position", "LEFT"));

                // extLayout.addNumber("ExtPositionInch", () -> round2dp(m_ext.positionInches));
                // extLayout.addNumber("ECommandIPS", () -> round2dp(m_ext.commandIPS));
                // extLayout.addNumber("ExtAmps", () -> round2dp(m_ext.amps));
                extLayout.addBoolean("EXTCANOK", () -> m_ext.extendMotorConnected)
                                .withWidget(BuiltInWidgets.kTextView);

                extLayout.add("EXPfPID", m_ext.m_extController).withWidget("Profiled PID Controller");

                ShuffleboardLayout wristLayout = Shuffleboard.getTab("Arms")
                                .getLayout("WristLayout", BuiltInLayouts.kList)
                                .withPosition(4, 0)
                                .withSize(2, 4)
                                .withProperties(Map.of("Label position", "LEFT"));

                wristLayout.addNumber("WristPosition", () -> round2dp(m_wrist.positionDegrees));
                wristLayout.addNumber("CommandDPS", () -> round2dp(m_wrist.commandDPS));
                // wristLayout.addNumber("WristAmps", () -> round2dp(m_wrist.amps));
                wristLayout.addBoolean("WristCANOK", () -> m_wrist.wristMotorConnected)
                                .withWidget(BuiltInWidgets.kTextView);

                wristLayout.add("WRPfPID", m_wrist.m_wristController).withWidget("Profiled PID Controller");

                ShuffleboardLayout intakeLayout = Shuffleboard.getTab("Arms")
                                .getLayout("IntakeLayout", BuiltInLayouts.kList)
                                .withPosition(6, 0)
                                .withSize(2, 2)
                                .withProperties(Map.of("Label position", "LEFT"));

                intakeLayout.add("RunIntake", new InstantCommand(() -> intake.runIntake()));

                intakeLayout.add("StopIntake", new StopIntake(intake));

                intakeLayout.addNumber("Actual RPM", () -> round2dp(intake.getRPM()));
                intakeLayout.addNumber("Intake Amps", () -> round2dp(intake.getAmps()));

                intakeLayout.addBoolean("IntakeCANOK", () -> intake.checkCANOK())
                                .withWidget(BuiltInWidgets.kTextView);

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
