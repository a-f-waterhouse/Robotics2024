package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.DriveTrainType;
import org.rowlandhall.meepmeep.roadrunner.SampleTankDrive;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(260), Math.toRadians(360), 15)
                .setDriveTrainType(DriveTrainType.TANK)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(90)))
                        .forward(24)
                        .waitSeconds(3)
                        .back(10)
                        .splineTo(new Vector2d(35, -24), Math.toRadians(90))
                        .splineTo(new Vector2d(41, -12), Math.toRadians(-90))
                        .back(48)
                        .forward(24)
                        .turn(Math.toRadians(180))
                        .forward(10)
                        .waitSeconds(2)
                        .splineTo(new Vector2d(12,-39),Math.toRadians(90))
                        .waitSeconds(3)
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_LIGHT)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}