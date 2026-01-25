package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setDimensions(17, 17)
                .setConstraints(
                        60, 60,
                        Math.toRadians(180),
                        Math.toRadians(180),
                        13.5
                )
                .build();

        bot.runAction(bot.getDrive().actionBuilder(new Pose2d(-56, -34, Math.toRadians(90)))
                // ---- CURVED PATH LIKE YOUR IMAGE ----
                .splineTo(new Vector2d(-54, -20), Math.toRadians(90))
                .splineTo(new Vector2d(-52, -5), Math.toRadians(90))
                .splineTo(new Vector2d(-50, 10), Math.toRadians(90))
                .splineTo(new Vector2d(-48, 25), Math.toRadians(90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .addEntity(bot)
                .start();
    }
}
