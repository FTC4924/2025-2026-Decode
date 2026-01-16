package com.example.meepmeeptesting

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity

object MeepMeepTesting {
    @JvmStatic
    fun main(args: Array<String>) {
        val meepMeep: MeepMeep = MeepMeep(800)
        val beginPose = Pose2d(61.0, -15.0, Math.PI)

        val myBot: RoadRunnerBotEntity =
            DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0)
                .build()

        val drive = myBot.drive;

        myBot.runAction(
            SequentialAction(
                drive.actionBuilder(beginPose)
                    .splineToLinearHeading(Pose2d(Vector2d(-12.0, -12.0), -3*(Math.PI / 4)), Math.PI)
                    .build()
            )
        )

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_LIGHT)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start()
    }
}