//package com.example.meepmeeptesting;
//
//import com.acmerobotics.roadrunner.MinVelConstraint;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.ProfileAccelConstraint;
//import com.acmerobotics.roadrunner.TurnConstraints;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.noahbres.meepmeep.MeepMeep;
//import com.noahbres.meepmeep.core.colorscheme.ColorManager;
//import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
//import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
//import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
//import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
//import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
//
//import org.jetbrains.annotations.NotNull;
//
//import java.awt.Color;
//import java.util.Arrays;
//
//public class MeepMeepTesting {
//    public static void main(String[] args) {
//        MeepMeep meepMeep = new MeepMeep(800);
//
//        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                .setColorScheme(new ColorScheme() {
//                    @Override
//                    public boolean isDark() {
//                        return true;
//                    }
//
//                    @NotNull
//                    @Override
//                    public Color getBOT_BODY_COLOR() {
//                        return ColorManager.COLOR_PALETTE.getGREEN_400();
//                    }
//
//                    @NotNull
//                    @Override
//                    public Color getBOT_WHEEL_COLOR() {
//                        return ColorManager.COLOR_PALETTE.getBLACK();
//                    }
//
//                    @NotNull
//                    @Override
//                    public Color getBOT_DIRECTION_COLOR() {
//                        return ColorManager.COLOR_PALETTE.getBLUE_800();
//                    }
//
//                    @NotNull
//                    @Override
//                    public Color getAXIS_X_COLOR() {
//                        return ColorManager.COLOR_PALETTE.getGRAY_900();
//                    }
//
//                    @NotNull
//                    @Override
//                    public Color getAXIS_Y_COLOR() {
//                        return ColorManager.COLOR_PALETTE.getGRAY_900();
//                    }
//
//                    @Override
//                    public double getAXIS_NORMAL_OPACITY() {
//                        return .8;
//                    }
//
//                    @Override
//                    public double getAXIS_HOVER_OPACITY() {
//                        return 0.8;
//                    }
//
//                    @NotNull
//                    @Override
//                    public Color getTRAJECTORY_PATH_COLOR() {
//                        return ColorManager.COLOR_PALETTE.getBLUE_300();
//                    }
//
//                    @NotNull
//                    @Override
//                    public Color getTRAJECTORY_TURN_COLOR() {
//                        return ColorManager.COLOR_PALETTE.getYELLOW_400();
//                    }
//
//                    @NotNull
//                    @Override
//                    public Color getTRAJECTORY_MARKER_COLOR() {
//                        return ColorManager.COLOR_PALETTE.getGREEN_600();
//                    }
//
//                    @NotNull
//                    @Override
//                    public Color getTRAJECTORY_SLIDER_BG() {
//                        return ColorManager.COLOR_PALETTE.getGRAY_200();
//                    }
//
//                    @NotNull
//                    @Override
//                    public Color getTRAJECTORY_SLIDER_FG() {
//                        return ColorManager.COLOR_PALETTE.getRED_400();
//                    }
//
//                    @NotNull
//                    @Override
//                    public Color getTRAJECTORY_TEXT_COLOR() {
//                        return ColorManager.COLOR_PALETTE.getGRAY_900();
//                    }
//
//                    @NotNull
//                    @Override
//                    public Color getUI_MAIN_BG() {
//                        return ColorManager.COLOR_PALETTE.getGRAY_200();
//                    }
//                })
//                .build();
//
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-5.5, 62.1, Math.toRadians(270)))
//                .setTangent(Math.toRadians(270))
//                .afterTime(.3, () -> {
////                    intake.toIntakeState(NewIntake.ToIntakeState.RAISE_INTAKE);
//                })
//                .splineToConstantHeading(new Vector2d(-6, 29), Math.toRadians(270))//, new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-40, PARAMS.maxProfileAccel))
//                .setTangent(Math.toRadians(90))
//                .afterDisp(20, () -> {
////                    intake.setTargetSlidePos(18.5);
////                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
//                })
//                .splineToConstantHeading(new Vector2d(-10, 37), Math.toRadians(180))
//                .splineToSplineHeading(new Pose2d(-25.5, 40.5, Math.toRadians(235)), Math.toRadians(180))
//                .turn(Math.toRadians(-85), new TurnConstraints(5 *Math.PI, -2 *Math.PI, 4 *Math.PI))
//                .afterTime(.1, () -> {
////                    intake.toIntakeState(NewIntake.ToIntakeState.RAISE_INTAKE);
////                    intake.setTargetSlidePos(14);
//                })
//
//                .setTangent(new com.reefsharklibrary.data.Vector2d(-35.5, 43).minus(new com.reefsharklibrary.data.Vector2d(-25.5, 40.5)).getDirection())
//                .lineToXSplineHeading(-35.5, Math.toRadians(230))
//                .afterTime(0, () -> {
////                    intake.setTargetSlidePos(18.5);
//                })
//                .waitSeconds(.3)
//                .turn(Math.toRadians(-76), new TurnConstraints(5 *Math.PI, -2 *Math.PI, 4 *Math.PI))
//                .afterTime(.1, () -> {
////                    intake.toIntakeState(NewIntake.ToIntakeState.RETRACT);
//                })
//                .setTangent(Math.toRadians(270))
//                .splineToLinearHeading(new Pose2d(-60, 14, Math.toRadians(270)), Math.toRadians(180))//, new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-50, PARAMS.maxProfileAccel))
//                .splineToConstantHeading(new Vector2d(-63, 17), Math.toRadians(90))//, new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-50, PARAMS.maxProfileAccel))
//                .lineToY(54)//, new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-50, PARAMS.maxProfileAccel))
//                .splineToConstantHeading(new Vector2d(-58,61.7), Math.toRadians(80))//, new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-50, PARAMS.maxProfileAccel))
//
//                .waitSeconds(.5)
//                .setTangent(Math.toRadians(305))
//                .splineToConstantHeading(new Vector2d(-3.5, 31), Math.toRadians(305))
//
//                .afterTime(.75, () -> {
////                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.WAIT_DROP_BEHIND);
//                })
//                .setTangent(Math.toRadians(115))
////                .lineToY(61.7, new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-35, PARAMS.maxProfileAccel))
//                .splineToConstantHeading(new Vector2d(-37, 59), Math.toRadians(90))
//                .lineToY(60.7)//, new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-35, PARAMS.maxProfileAccel))
////                .splineToConstantHeading(new Vector2d(-37, 60), Math.toRadians(90))
////                .lineToY(61.7)
//                .waitSeconds(.1)
//
//                .waitSeconds(.5)
//                .setTangent(new com.reefsharklibrary.data.Vector2d(-4, 29).minus(new com.reefsharklibrary.data.Vector2d(-37, 61.7)).getDirection())
//                .lineToY(29)
//
//                .setTangent(Math.toRadians(115))
//                .afterTime(.75, () -> {
////                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.WAIT_DROP_BEHIND);
//                })
//                .splineToConstantHeading(new Vector2d(-37, 59), Math.toRadians(90))
//                .lineToY(60.7) //, new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-35, PARAMS.maxProfileAccel))
//                .waitSeconds(.1)
//
//                .waitSeconds(.5)
//                .setTangent(new com.reefsharklibrary.data.Vector2d(-4, 29).minus(new com.reefsharklibrary.data.Vector2d(-37, 60.7)).getDirection())
//                .lineToY(29)
//
//
//                .setTangent(Math.toRadians(115))
//                .afterTime(.75, () -> {
////                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.WAIT_DROP_BEHIND);
//                })
//                .splineToConstantHeading(new Vector2d(-37, 59), Math.toRadians(90))
//                .lineToY(60.7)//, new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-35, PARAMS.maxProfileAccel))
//                .waitSeconds(.1)
//
//                .waitSeconds(.5)
//                        .setTangent(new com.reefsharklibrary.data.Vector2d(-4, 29).minus(new com.reefsharklibrary.data.Vector2d(-37, 60.7)).getDirection())
//                        .lineToY(29)
//
//                .afterTime(.3, () -> {
////                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.RETRACT_FROM_PLACE_BEHIND);
//                })
//                .setTangent(Math.toRadians(90))
//                .afterTime(.6, () -> {
////                    intake.setTargetSlidePos(18.5);
////                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
//                })
//                .splineToSplineHeading(new Pose2d(-22, 47, Math.toRadians(155)), Math.toRadians(165))
//
//                .build());
//
//
//        RoadRunnerBotEntity secondBot = new DefaultBotBuilder(meepMeep)
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//
//                .setColorScheme(new ColorScheme() {
//                    @Override
//                    public boolean isDark() {
//                        return true;
//                    }
//
//                    @NotNull
//                    @Override
//                    public Color getBOT_BODY_COLOR() {
//                        return ColorManager.COLOR_PALETTE.getGREEN_400();
//                    }
//
//                    @NotNull
//                    @Override
//                    public Color getBOT_WHEEL_COLOR() {
//                        return ColorManager.COLOR_PALETTE.getBLACK();
//                    }
//
//                    @NotNull
//                    @Override
//                    public Color getBOT_DIRECTION_COLOR() {
//                        return ColorManager.COLOR_PALETTE.getBLUE_800();
//                    }
//
//                    @NotNull
//                    @Override
//                    public Color getAXIS_X_COLOR() {
//                        return ColorManager.COLOR_PALETTE.getGRAY_900();
//                    }
//
//                    @NotNull
//                    @Override
//                    public Color getAXIS_Y_COLOR() {
//                        return ColorManager.COLOR_PALETTE.getGRAY_900();
//                    }
//
//                    @Override
//                    public double getAXIS_NORMAL_OPACITY() {
//                        return .8;
//                    }
//
//                    @Override
//                    public double getAXIS_HOVER_OPACITY() {
//                        return 0.8;
//                    }
//
//                    @NotNull
//                    @Override
//                    public Color getTRAJECTORY_PATH_COLOR() {
//                        return ColorManager.COLOR_PALETTE.getBLUE_300();
//                    }
//
//                    @NotNull
//                    @Override
//                    public Color getTRAJECTORY_TURN_COLOR() {
//                        return ColorManager.COLOR_PALETTE.getYELLOW_400();
//                    }
//
//                    @NotNull
//                    @Override
//                    public Color getTRAJECTORY_MARKER_COLOR() {
//                        return ColorManager.COLOR_PALETTE.getGREEN_600();
//                    }
//
//                    @NotNull
//                    @Override
//                    public Color getTRAJECTORY_SLIDER_BG() {
//                        return ColorManager.COLOR_PALETTE.getGRAY_200();
//                    }
//
//                    @NotNull
//                    @Override
//                    public Color getTRAJECTORY_SLIDER_FG() {
//                        return ColorManager.COLOR_PALETTE.getRED_400();
//                    }
//
//                    @NotNull
//                    @Override
//                    public Color getTRAJECTORY_TEXT_COLOR() {
//                        return ColorManager.COLOR_PALETTE.getGRAY_900();
//                    }
//
//                    @NotNull
//                    @Override
//                    public Color getUI_MAIN_BG() {
//                        return ColorManager.COLOR_PALETTE.getGRAY_200();
//                    }
//                })
//                .build();
//
//        secondBot.runAction(secondBot.getDrive().actionBuilder(new Pose2d(-38.93, -60.23, Math.toRadians(0)))
//                .setTangent(Math.toRadians(300 + 180))
//                .afterTime(.3, () -> {
////                    intake.toIntakeState(NewIntake.ToIntakeState.RAISE_INTAKE);
//                })
//                .afterTime(.9, () -> {
////                    intake.setTargetSlidePos(16);
////                    extensionDistance = 16;
//                })
//                .splineToLinearHeading(new Pose2d(-62, -54, Math.toRadians(250+ 180)), Math.toRadians(45+ 180))
//
//                .setTangent(Math.toRadians(250+ 180))
//                .afterTime(0, () -> {
////                    intake.setTargetSlidePos(9);
////                    extensionDistance = 9;
////                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
////                    intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
////                    autoTimer.reset();
//                })
//                .splineToConstantHeading(new Vector2d(-56, -51), Math.toRadians(250+ 180))
//
//                .setTangent(Math.toRadians(84+ 180))
//                .splineToLinearHeading(new Pose2d(-61.5, -51, Math.toRadians(265+ 180)), Math.toRadians(85+ 180))
//
//                .afterTime(0, () -> {
////                    intake.setTargetSlidePos(11);
////                    extensionDistance = 11;
////                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
////                    intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
////                    autoTimer.reset();
//                })
//                .waitSeconds(.1)
//                .setTangent(265+ 180)
//                .splineToConstantHeading(new Vector2d(-61, -51), Math.toRadians(265+ 180))
//
//                .setTangent(Math.toRadians(90+ 180))
//                .splineToLinearHeading(new Pose2d(-62.5, -52, Math.toRadians(270+ 180)), Math.toRadians(90+ 180))
//
//                        .afterTime(0, () -> {
////                    intake.setTargetSlidePos(13);
////                    extensionDistance = 13;
////                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
////                    intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
////                    autoTimer.reset();
//                })
//                .waitSeconds(.1)
//                .setTangent(270+ 180)
//                .splineToLinearHeading(new Pose2d(-62.5, -51.5, Math.toRadians(278+ 180)), Math.toRadians(270+ 180))
//
//                .setTangent(Math.toRadians(90+ 180))
//                .splineToLinearHeading(new Pose2d(-62.5, -53, Math.toRadians(270+ 180)), Math.toRadians(90+ 180))
//
//                .afterTime(.5, () -> {
////                    intake.toIntakeState(NewIntake.ToIntakeState.RAISE_INTAKE);
//                })
//                        .setTangent(Math.toRadians(240+ 180))
//                        .splineToLinearHeading(new Pose2d(-35, -7, Math.toRadians(180+ 180)), Math.toRadians(180+ 180))
//                        .afterTime(.8, () -> {
////                            intake.setTargetSlidePos(7);
////                            extensionDistance = 7;
//                        })
//                        .afterTime(1.3, () -> {
////                            intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
////                            intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
//                        })
//                        .splineToLinearHeading(new Pose2d(-23, -7, Math.toRadians(185+ 180)), Math.toRadians(180+ 180))
//
//                        .turn(Math.toRadians(-10))
//                        .turn(Math.toRadians(10))
//
//                        .afterTime(0, () -> {
////                            intake.blueAlliance = true;
//                        })
//                        .setTangent(Math.toRadians(180))
//                        .lineToX(-40)
//                        .splineToLinearHeading(new Pose2d(-58.5, -53.5, Math.toRadians(225+ 180)), Math.toRadians(45+ 180))
//
//                        .afterTime(.5, () -> {
////                            intake.toIntakeState(NewIntake.ToIntakeState.RAISE_INTAKE);
//                        })
//                        .afterTime(1, () -> {
////                            outtake.toOuttakeState(NewOuttake.ToOuttakeState.TOUCH_BAR);
//                        })
//                        .setTangent(Math.toRadians(240+ 180))
//                        .splineToLinearHeading(new Pose2d(-35, -6, Math.toRadians(180+ 180)), Math.toRadians(180+ 180))
//                        .splineToConstantHeading(new Vector2d(-20, -6), Math.toRadians(180+ 180))
//
//                .build());
//
//        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
//                .setDarkMode(true)
//                .setBackgroundAlpha(0.95f)
//                .addEntity(myBot)
//
//                .addEntity(secondBot)
//
//                .start();
//    }
//}