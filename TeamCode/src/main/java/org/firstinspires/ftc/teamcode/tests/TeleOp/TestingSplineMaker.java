package org.firstinspires.ftc.teamcode.tests.TeleOp;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.classes.TapeMeasure;

@Config
@Autonomous(name = "SPLINESTESTS", group = "BLUE")
public class TestingSplineMaker extends LinearOpMode {

    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();
        drive.setPoseEstimate(new Pose2d(-36,60, Math.toRadians(0)));
        Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate(), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(30,60, Math.toRadians(300)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(36,36, Math.toRadians(270)), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(36,-42, Math.toRadians(300)), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(36,-60, Math.toRadians(45)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(60,-60, Math.toRadians(135)), Math.toRadians(86))
                .splineToSplineHeading(new Pose2d(48,-12, Math.toRadians(234)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-48,-12, Math.toRadians(270)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-61.24,-23.04, Math.toRadians(0)), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-48,-36, Math.toRadians(90)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-24,-36, Math.toRadians(45)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(60,-36, Math.toRadians(0)), Math.toRadians(0))
                .build();
        drive.followTrajectory(traj);


    }
}
