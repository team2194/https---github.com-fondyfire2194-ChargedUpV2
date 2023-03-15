// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.simulation.SimConstants;

/** Add your docs here. */
public class GameHandlerSubsystem extends SubsystemBase {

        public static int gridHeight = 0;// 0 floor, 1 mid, 2 high

        /**
         * Grid values as viewed by Blue Drivers starting on their right at the field
         * RIGHT
         * Red drivers have the field RIGHT on their left
         * The X distance is the center of the robot as it strafes past the grids
         * For Blue it will be the distance used.
         * For Red it will be the field length minus that distance
         * Similarly Y will be used direct or by subtracting from the field width
         * Grids are 66" wide
         */

        public enum GridDrop {

                LEFT_CUBE(SimConstants.Tags.aprilTagsRed[0], false,
                                SimConstants.Tags.aprilTagsBlue[5],
                                false),

                LEFT_PIPE(SimConstants.leftPipeRed, true,
                                SimConstants.leftPipeBlue,
                                true),

                COOP_LEFT_PIPE(SimConstants.coopleftPipeRed, true,
                                SimConstants.coopleftPipeBlue, true),

                COOP_CUBE(SimConstants.Tags.aprilTagsRed[1], false,
                                SimConstants.Tags.aprilTagsBlue[6], false),

                COOP_RIGHT_PIPE(SimConstants.cooprightPipeRed, true,
                                SimConstants.cooprightPipeBlue,
                                true),

                RIGHT_PIPE(SimConstants.rightPipeRed, true,
                                SimConstants.rightPipeBlue,
                                true),

                RIGHT_CUBE(SimConstants.Tags.aprilTagsRed[2], false,
                                SimConstants.Tags.aprilTagsBlue[7],
                                false);

                private final Pose2d bluePose;

                private final Pose2d redPose;

                private final boolean isPipeRed;

                private final boolean isPipeBlue;

                private final static int length = GridDrop.values().length;

                private GridDrop(Pose2d redPose, boolean isPipeRed, Pose2d bluePose,
                                boolean isPipeBlue) {
                        this.redPose = redPose;
                        this.bluePose = bluePose;
                        this.isPipeRed = isPipeRed;

                        this.isPipeBlue = isPipeBlue;

                }

                public Pose2d getBluePose() {
                        return bluePose;
                }

                public Pose2d getRedPose() {
                        return redPose;
                }

                public boolean getIsPipeRed() {
                        return isPipeRed;
                }

                public boolean getIsPipeBlue() {
                        return isPipeBlue;
                }
        }

        public enum dropOffLevel {

                GROUND_LEVEL(0),
                MID_LEVEL(1),
                TOP_LEVEL(2);

                private int level;

                private dropOffLevel(int level) {
                        this.level = level;
                }
        }

        public enum gamePiece {

                CUBE(0),

                CONE(1);

                private int type;

                private gamePiece(int type) {
                        this.type = type;
                }
        }

        public enum robotPiece {

                CUBE(0),
                CONE(1),
                NO_PIECE(2);

                private int type;

                private robotPiece(int type) {
                        this.type = type;
                }
        }

        public enum fieldTagsBlue {

                RED_LEFT(1, SimConstants.Tags.aprilTagsBlue[0]),
                RED_CENTER(2, SimConstants.Tags.aprilTagsBlue[1]),
                RED_RIGHT(3, SimConstants.Tags.aprilTagsBlue[2]),
                BLUE_LOAD(4, SimConstants.Tags.aprilTagsBlue[3]),
                RED_LOAD(5, SimConstants.Tags.aprilTagsBlue[5]),
                BLUE_LEFT(6, SimConstants.Tags.aprilTagsBlue[5]),
                BLUE_CENTER(7, SimConstants.Tags.aprilTagsBlue[6]),
                BLUE_RIGHT(8, SimConstants.Tags.aprilTagsBlue[7]);

                private int id;

                private Pose2d pose;

                private fieldTagsBlue(int id, Pose2d pose) {
                        this.id = id;
                        this.pose = pose;
                }

                public Pose2d getPose() {
                        return pose;
                }

                public int getID() {
                        return id;
                }

        }

        public enum fieldTagsRed {

                RED_LEFT(1, SimConstants.Tags.aprilTagsRed[0]),
                RED_CENTER(2, SimConstants.Tags.aprilTagsRed[1]),
                RED_RIGHT(3, SimConstants.Tags.aprilTagsRed[2]),
                BLUE_LOAD(4, SimConstants.Tags.aprilTagsRed[3]),
                RED_LOAD(5, SimConstants.Tags.aprilTagsRed[4]),
                BLUE_LEFT(6, SimConstants.Tags.aprilTagsRed[5]),
                BLUE_CENTER(7, SimConstants.Tags.aprilTagsRed[6]),
                BLUE_RIGHT(8, SimConstants.Tags.aprilTagsRed[7]);

                private int id;

                private Pose2d pose;

                private fieldTagsRed(int id, Pose2d pose) {
                        this.id = id;
                        this.pose = pose;
                }

                public Pose2d getPose() {
                        return pose;
                }

        }

        public dropOffLevel chosenLevel = dropOffLevel.MID_LEVEL;

        public gamePiece gamePieceType = gamePiece.CONE;

        public double strafeDistance = 1.5;// meters

        public fieldTagsRed activeTagRed;

        public fieldTagsBlue activeTagBlue;

        private static GridDrop activeDrop = GridDrop.COOP_CUBE;

        public int dropNumberSet;

        public boolean[] drops = new boolean[8];

        public Pose2d blueRightHybridNode = new Pose2d(
                        new Translation2d(Units.inchesToMeters(56.35), Units.inchesToMeters(29)), new Rotation2d());
        public Pose2d blueLeftHybridNode = new Pose2d(
                        new Translation2d(Units.inchesToMeters(56.25), Units.inchesToMeters(196)), new Rotation2d());
        public Pose2d redLeftHybridNode = new Pose2d(
                        new Translation2d(Units.inchesToMeters(56.35), Units.inchesToMeters(250)), new Rotation2d());
        public Pose2d redRightHybridNode = new Pose2d(
                        new Translation2d(Units.inchesToMeters(56.35), Units.inchesToMeters(29)), new Rotation2d());

        private Pose2d gpBluePose;

        private Pose2d gpRedPose;

        private Alliance originalAlliance;

        public boolean CANOK;

        private int loopCtr;

        public GameHandlerSubsystem() {

                dropNumberSet = 4;
                drops[dropNumberSet] = true;
                setActiveDropByNumber(dropNumberSet);

                // SmartDashboard.putNumber("GVL", GridDrop.length);

                originalAlliance = DriverStation.getAlliance();

        }

        /**
         * drop number comes in from the driver station as a value from 0 to 6
         * In the Blue Alliance case, the number cam go straight though and be used as
         * is
         * It is used to set the active drop and this picks out the x, y and pipe
         * entries
         * It also picks the name out of the String array for display purposes
         * In the RedAlliance case, the number needs to be treated differently
         * The String selection remains the same but the x, y and pipe come from the
         * opposite way
         * in the enum
         * For example, the RightHybridPipe is index (ordinal)0. However for Red,
         * the values returned have to be from the LeftHybridPipe - index 8
         * So for Red internally setting the active drop will look wrong but the values
         * will be correct
         * The original index gets used for the String name
         * 
         * 
         * @param drop
         */
        public void setActiveDrop(GridDrop drop) {
                activeDrop = drop;

        }

        public void setActiveDropByNumber(int num) {

                SmartDashboard.putNumber("SELDROP", num);

                dropNumberSet = num;
                
                clearDrops();

                drops[num]=true;

                switch (num) {
                        case 0:
                                break;
                        case 1:
                                setActiveDrop(GridDrop.LEFT_CUBE);
                                gpBluePose = (GridDrop.LEFT_CUBE.bluePose);
                                gpRedPose = (GridDrop.LEFT_CUBE.redPose);

                                break;
                        case 2:
                                setActiveDrop(GridDrop.LEFT_PIPE);
                                gpBluePose = (GridDrop.LEFT_PIPE.bluePose);
                                gpRedPose = (GridDrop.LEFT_PIPE.redPose);
                                break;
                        case 3:
                                setActiveDrop(GridDrop.COOP_LEFT_PIPE);
                                gpBluePose = (GridDrop.COOP_LEFT_PIPE.bluePose);
                                gpRedPose = (GridDrop.COOP_LEFT_PIPE.redPose);
                                break;
                        case 4:
                                setActiveDrop(GridDrop.COOP_CUBE);
                                gpBluePose = (GridDrop.COOP_CUBE.bluePose);
                                gpRedPose = (GridDrop.COOP_CUBE.redPose);
                                break;
                        case 5:
                                setActiveDrop(GridDrop.COOP_RIGHT_PIPE);
                                gpBluePose = (GridDrop.COOP_RIGHT_PIPE.bluePose);
                                gpRedPose = (GridDrop.COOP_RIGHT_PIPE.redPose);
                                break;
                        case 6:
                                setActiveDrop(GridDrop.RIGHT_PIPE);
                                gpBluePose = (GridDrop.RIGHT_PIPE.bluePose);
                                gpRedPose = (GridDrop.RIGHT_PIPE.redPose);
                                break;
                        case 7:
                                setActiveDrop(GridDrop.RIGHT_CUBE);
                                gpBluePose = (GridDrop.RIGHT_CUBE.bluePose);
                                gpRedPose = (GridDrop.RIGHT_CUBE.redPose);
                                break;
                        default:
                                break;

                }

        }

        public GridDrop getActiveDrop() {

                return activeDrop;
        }

        public Pose2d getActiveDropPose() {

                if (DriverStation.getAlliance() == Alliance.Blue) {

                        return gpBluePose;

                } else {
                        return gpRedPose;

                }
        }

        public double getXDistanceRed() {

                return activeDrop.getRedPose().getX();

        }

        public double getXDistanceBlue() {
                return activeDrop.getBluePose().getX();

        }

        public void setDropOffLevel(dropOffLevel level) {
                chosenLevel = level;
        }

        public dropOffLevel getDropOffLevel() {
                return chosenLevel;
        }

        public void stepDropOffLevel() {
                boolean levelChanged = false;
                // if (getDropOffLevel() == dropOffLevel.GROUND_LEVEL) {
                // setDropOffLevel(dropOffLevel.MID_LEVEL);
                // levelChanged = true;
                // }
                if (!levelChanged && getDropOffLevel() == dropOffLevel.MID_LEVEL) {
                        setDropOffLevel(dropOffLevel.TOP_LEVEL);
                        levelChanged = true;
                }
                if (!levelChanged && getDropOffLevel() == dropOffLevel.TOP_LEVEL)
                        setDropOffLevel(dropOffLevel.MID_LEVEL);
        }

        public void setDropOffLevelByNumber(int level) {
                if (level == 0)
                        setDropOffLevel(dropOffLevel.TOP_LEVEL);
                else
                        setDropOffLevel(dropOffLevel.MID_LEVEL);

        }

        public boolean getAllianceBlue() {
                return (DriverStation.getAlliance() == Alliance.Blue);
        }

        public void setGamePieceType(gamePiece type) {
                gamePieceType = type;
        }

        public void toggleGamePieceType() {

                if (gamePieceType == gamePiece.CONE)

                        gamePieceType = gamePiece.CUBE;
                else
                        gamePieceType = gamePiece.CONE;
        }

        public gamePiece getGamePiecetype() {
                return gamePieceType;
        }

        public int getDropNumber() {
                return dropNumberSet;
        }

        public void incDropNumber() {
                dropNumberSet++;
                if (dropNumberSet > GridDrop.length)
                        dropNumberSet = 1;
                clearDrops();
                drops[dropNumberSet] = true;

                setActiveDropByNumber(dropNumberSet);
        }

        public void decDropNumber() {
                dropNumberSet--;
                if (dropNumberSet < 1)
                        dropNumberSet = GridDrop.length;
                clearDrops();
                drops[dropNumberSet] = true;
                setActiveDropByNumber(dropNumberSet);
        }

        public void clearDrops() {
                for (int n = 1; n < drops.length; n++) {
                        drops[n] = false;
                }
        }

        @Override

        public void periodic() {

                if (originalAlliance != DriverStation.getAlliance()) {
                        setActiveDropByNumber(dropNumberSet);
                }

        }

}
