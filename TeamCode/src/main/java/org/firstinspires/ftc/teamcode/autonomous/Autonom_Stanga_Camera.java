
/*
package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.Utilities.DegToRad;
import static org.firstinspires.ftc.teamcode.Utilities.RunInParallel;
import static org.firstinspires.ftc.teamcode.Utilities.RunSequentially;
import static org.firstinspires.ftc.teamcode.Utilities.WaitFor;
import static org.firstinspires.ftc.teamcode.Utilities.centimetersToInches;

import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.systems.ExtendoSystem;
import org.firstinspires.ftc.teamcode.systems.LiftSystem;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.tuning.vision.AlignToSample;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Autonom_Stanga_Camera", group = "Auto")
public class Autonom_Stanga_Camera extends BaseOpMode
{
	private RobotHardware robot;
	private MecanumDrive drivetrain;
	@Override
	protected void OnInitialize()
	{
		robot = new RobotHardware(hardwareMap);
		robot.init();

		robot.lift.setLiftPower(1);
		if (!robot.distanceSystem.StatusOk())
		{
			telemetry.addLine("[ERROR] Distance sensor timeout!");
			telemetry.update();
		}
		ColorBlobLocatorProcessor processor;
		VisionPortal portal;
		processor = new AlignToSample.GreyToBlackPipeline();

		portal = new VisionPortal.Builder()
				.addProcessor(processor)
				.setCameraResolution(new Size(640, 480))
				.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
				.setLiveViewContainerId(hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()))
				.enableLiveView(true) // Enable live view
				.build();

		while (portal.getCameraState() != VisionPortal.CameraState.STREAMING)
			sleep(10);

		portal.getCameraControl(GainControl.class).setGain(255);
		portal.getCameraControl(ExposureControl.class).setExposure(20, TimeUnit.MILLISECONDS);
	}

	@Override
	protected void jOnInitialize()
	{

	}

	@Override
	protected void OnRun()
	{
		// Deliver preload specimen
		Actions.runBlocking(RunSequentially(
				robot.scoreRotator.closeAction(),
				robot.scoreStumbler.setDestinationAction(TumblerSystem.TumblerDestination.BUSY),
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(0)
								.splineToLinearHeading(new Pose2d(15, 20, Math.toRadians(-27)), Math.toRadians(-27),(pose2dDual, posePath, v) -> 150, (pose2dDual, posePath, v) -> new MinMax(-150, 150))
								.build(),
						robot.lift.moveLiftTo(LiftSystem.LiftLevel.BASKET_HIGH)
				),
				robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOVER),
				robot.scoreStumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOVER),
				WaitFor(0.6),
				robot.scoreClaw.openAction(),
				WaitFor(0.4),
				robot.scoreStumbler.setDestinationAction(TumblerSystem.TumblerDestination.TRANSFER),
				RunInParallel(
						robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.IDLE),
						robot.lift.moveLiftTo(LiftSystem.LiftLevel.IDLE),
						robot.extendo.extendAction(ExtendoSystem.ExtendoLevel.SAMPLE)
				),
				robot.tumbler.setDestinationAction(TumblerSystem.TumblerDestination.BUSY),
				WaitFor(0.2),
				//robot.rotator.setPositionAction(0.4f),
				robot.smallTumbler.setDestinationAction(TumblerSystem.TumblerDestination.BUSY),
				WaitFor(0.5),
				robot.claw.closeAction(),
				WaitFor(0.3),
				robot.rotator.setPositionAction(0.476f),
				robot.tumbler.setDestinationAction(TumblerSystem.TumblerDestination.TRANSFER),
				robot.smallTumbler.setDestinationAction(TumblerSystem.TumblerDestination.TRANSFER),
				robot.extendo.extendAction(ExtendoSystem.ExtendoLevel.RETRACTED),
				WaitFor(0.4),
				robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.TRANSFER),
				WaitFor(0.4),
				robot.scoreClaw.closeAction(),
				robot.claw.openAction(),
				WaitFor(0.3),
				robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOVER),
				robot.lift.moveLiftTo(LiftSystem.LiftLevel.BASKET_HIGH),
				WaitFor(0.1),
				robot.scoreStumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOVER),
				WaitFor(0.3),
				robot.scoreClaw.openAction(),
				WaitFor(0.4),
				robot.tumbler.setDestinationAction(TumblerSystem.TumblerDestination.IDLE)
		));
		Actions.runBlocking(RunSequentially(
				robot.scoreStumbler.setDestinationAction(TumblerSystem.TumblerDestination.IDLE),
				WaitFor(0.2),
				robot.lift.moveLiftTo(LiftSystem.LiftLevel.IDLE),
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(0)
								.splineToLinearHeading(new Pose2d(13, 21, Math.toRadians(-3)), Math.toRadians(-3),(pose2dDual, posePath, v) -> 50, (pose2dDual, posePath, v) -> new MinMax(-50, 50))
								.build()
				),
				robot.extendo.extendAction(ExtendoSystem.ExtendoLevel.SECOND_SAMPLE),
				WaitFor(0.3),
				robot.rotator.setPositionAction(0.49f),
				robot.tumbler.setDestinationAction(TumblerSystem.TumblerDestination.BUSY),
				robot.smallTumbler.setDestinationAction(TumblerSystem.TumblerDestination.BUSY),
				WaitFor(0.5),
				robot.claw.closeAction(),
				WaitFor(0.3),
				robot.rotator.setPositionAction(0.476f),
				robot.tumbler.setDestinationAction(TumblerSystem.TumblerDestination.TRANSFER),
				robot.smallTumbler.setDestinationAction(TumblerSystem.TumblerDestination.TRANSFER),
				robot.extendo.extendAction(ExtendoSystem.ExtendoLevel.RETRACTED),
				WaitFor(0.4),
				robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.TRANSFER),
				WaitFor(0.4),
				robot.scoreClaw.closeAction(),
				robot.claw.openAction(),
				WaitFor(0.2),
				robot.lift.moveLiftTo(LiftSystem.LiftLevel.BASKET_HIGH),
				robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOVER),
				WaitFor(0.4),
				robot.scoreStumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOVER),
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(0)
								.splineToLinearHeading(new Pose2d(14, 18, Math.toRadians(-17)), Math.toRadians(-17),(pose2dDual, posePath, v) -> 150, (pose2dDual, posePath, v) -> new MinMax(-150, 150))
								.build(),
						robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOVER)
				),
				WaitFor(0.3),
				robot.scoreClaw.openAction(),
				WaitFor(0.3)
		));
		Actions.runBlocking(RunSequentially(
				robot.scoreStumbler.setDestinationAction(TumblerSystem.TumblerDestination.IDLE),
				robot.lift.moveLiftTo(LiftSystem.LiftLevel.IDLE),
				WaitFor(0.3),
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(0)
								.splineToLinearHeading(new Pose2d(15, 20, Math.toRadians(12.4)), Math.toRadians(12.4))
								.build()
				),
				robot.extendo.extendAction(ExtendoSystem.ExtendoLevel.THIRD_SAMPLE),
				WaitFor(0.5),
				robot.tumbler.setDestinationAction(TumblerSystem.TumblerDestination.BUSY),
				robot.smallTumbler.setDestinationAction(TumblerSystem.TumblerDestination.BUSY),
				robot.rotator.setPositionAction(0.463f),
				WaitFor(0.8),
				robot.claw.closeAction(),
				WaitFor(0.3),
				robot.rotator.setPositionAction(0.476f),
				robot.tumbler.setDestinationAction(TumblerSystem.TumblerDestination.TRANSFER),
				robot.smallTumbler.setDestinationAction(TumblerSystem.TumblerDestination.TRANSFER),
				robot.extendo.extendAction(ExtendoSystem.ExtendoLevel.RETRACTED),
				WaitFor(0.4),
				robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.TRANSFER),
				WaitFor(0.4),
				robot.scoreClaw.closeAction(),
				robot.claw.openAction(),
				WaitFor(0.3),
				robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOVER),
				robot.lift.moveLiftTo(LiftSystem.LiftLevel.BASKET_HIGH),
				WaitFor(0.4),
				robot.scoreStumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOVER),
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(0)
								.splineToLinearHeading(new Pose2d(14, 23.1, Math.toRadians(-17.5)), Math.toRadians(-17.5),(pose2dDual, posePath, v) -> 150, (pose2dDual, posePath, v) -> new MinMax(-150, 150))
								.build(),
						robot.lift.moveLiftTo(LiftSystem.LiftLevel.BASKET_HIGH),
						robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOVER)
				),
				WaitFor(0.3),
				robot.scoreClaw.openAction(),
				WaitFor(0.3)
		));
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(0)
								.lineToXLinearHeading(50, Math.toRadians(-90))
								.setTangent(Math.PI/2)
								.lineToY(-20)
								//.splineToLinearHeading(new Pose2d(55, 17, Math.toRadians(90)), Math.toRadians(90))
								.build(),
						robot.lift.moveLiftTo(LiftSystem.LiftLevel.IDLE),
						robot.scoreStumbler.setDestinationAction(TumblerSystem.TumblerDestination.IDLE)
				),
				WaitFor(1)
				//robot.scoreStumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOVER)

		));
		Actions.runBlocking(RunSequentially(
				//modify this to make the robot move until it sees something with the camera

		));
		/*
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(0)
								.splineToLinearHeading(new Pose2d(-16.85, 21.85, Math.toRadians(45)), Math.toRadians(0))
								.build()
				),
				WaitFor(100)
		));
		 */
/*
	}
}

 */