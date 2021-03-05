package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.rr_quickstart_examples.drive.SampleMecanumDrive;

@Autonomous(name = "UltimateGoalAuto", group = "road-runner")
public class UltimateGoalAuto extends LinearOpMode {
    private Trajectory squareA;
    private Trajectory squareB;
    private Trajectory squareC;
    private Trajectory parkA;
    private Trajectory parkB;
    private Trajectory parkC;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        final int y = -1;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory powerShotLocation = drive.trajectoryBuilder(new Pose2d(-58, 17 * y))
                .lineTo(new Vector2d(-13,-30  * y))
                .build();

        squareA = drive.trajectoryBuilder(powerShotLocation.end())
                .lineTo(new Vector2d(4, 55 * y))
                .build();
        squareB = drive.trajectoryBuilder(powerShotLocation.end())
                .lineTo(new Vector2d(24, 35 * y))
                .build();
        squareC = drive.trajectoryBuilder(powerShotLocation.end())
                .lineTo(new Vector2d(50, 55 * y))
                .build();

        parkA = drive.trajectoryBuilder(squareA.end())
                .lineTo(new Vector2d(squareA.end().getX(), 0 * y))
                .splineToConstantHeading(new Vector2d(15, 35 * y), 0)
                .build();
        parkB = drive.trajectoryBuilder(squareB.end())
                .lineTo(new Vector2d(15, 35 * y))
                .build();
        parkC = drive.trajectoryBuilder(squareC.end())
                .lineTo(new Vector2d(15, 35 * y))
                .build();

        drive.setPoseEstimate(new Pose2d(-58, 17));
        drive.followTrajectory(powerShotLocation);

        // Shoot fist power shot
        for(int i = 0; i < 2; i++){
            drive.turn(Math.toRadians(-30)); // angle will be tuned later on
            // Shoot second and third powershot
        }
        final int ringAmount = 4;
        if(ringAmount == 0) {
            drive.followTrajectory(squareA);
            drive.followTrajectory(parkA);
        }
        else if(ringAmount == 1) {
            drive.followTrajectory(squareB);
            drive.followTrajectory(parkB);
        }
        else {
            drive.followTrajectory(squareC);
            drive.followTrajectory(parkC);
        }




    }
}