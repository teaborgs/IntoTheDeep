/*
package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.Utilities.DegToRad;
import static org.firstinspires.ftc.teamcode.Utilities.RunInParallel;
import static org.firstinspires.ftc.teamcode.Utilities.RunSequentially;
import static org.firstinspires.ftc.teamcode.Utilities.WaitFor;
import static org.firstinspires.ftc.teamcode.Utilities.centimetersToInches;

import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.systems.ExtendoSystem;
import org.firstinspires.ftc.teamcode.systems.LiftSystem;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;

@Autonomous(name = "TestPID", group = "Auto")
public class TestPID extends BaseOpMode
{
	private RobotHardware robot;

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
					RunInParallel(
							robot.drivetrain.actionBuilder(robot.drivetrain.pose)
									.setTangent(Math.PI)
									.splineToLinearHeading(new Pose2d(-12, 0, Math.toRadians(0)), Math.toRadians(0))
									.build()
					)
			));
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.toRadians(-90))
								.splineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(0)), Math.toRadians(0))
								.build()
				)
		));
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.toRadians(0))
								.splineToLinearHeading(new Pose2d(0, -12, Math.toRadians(0)), Math.toRadians(0))
								.build()
				)
		));
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.toRadians(90))
								.splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)), Math.toRadians(0))
								.build()
				)
		));
		//
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.PI)
								.splineToLinearHeading(new Pose2d(-12, 0, Math.toRadians(0)), Math.toRadians(0))
								.build()
				)
		));
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.toRadians(-90))
								.splineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(0)), Math.toRadians(0))
								.build()
				)
		));
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.toRadians(0))
								.splineToLinearHeading(new Pose2d(0, -12, Math.toRadians(0)), Math.toRadians(0))
								.build()
				)
		));
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.toRadians(90))
								.splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)), Math.toRadians(0))
								.build()
				)
		));
		//
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.PI)
								.splineToLinearHeading(new Pose2d(-12, 0, Math.toRadians(0)), Math.toRadians(0))
								.build()
				)
		));
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.toRadians(-90))
								.splineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(0)), Math.toRadians(0))
								.build()
				)
		));
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.toRadians(0))
								.splineToLinearHeading(new Pose2d(0, -12, Math.toRadians(0)), Math.toRadians(0))
								.build()
				)
		));
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.toRadians(90))
								.splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)), Math.toRadians(0))
								.build()
				)
		));
		//
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.PI)
								.splineToLinearHeading(new Pose2d(-12, 0, Math.toRadians(0)), Math.toRadians(0))
								.build()
				)
		));
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.toRadians(-90))
								.splineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(0)), Math.toRadians(0))
								.build()
				)
		));
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.toRadians(0))
								.splineToLinearHeading(new Pose2d(0, -12, Math.toRadians(0)), Math.toRadians(0))
								.build()
				)
		));
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.toRadians(90))
								.splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)), Math.toRadians(0))
								.build()
				)
		));
		//
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.PI)
								.splineToLinearHeading(new Pose2d(-12, 0, Math.toRadians(0)), Math.toRadians(0))
								.build()
				)
		));
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.toRadians(-90))
								.splineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(0)), Math.toRadians(0))
								.build()
				)
		));
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.toRadians(0))
								.splineToLinearHeading(new Pose2d(0, -12, Math.toRadians(0)), Math.toRadians(0))
								.build()
				)
		));
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.toRadians(90))
								.splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)), Math.toRadians(0))
								.build()
				)
		));
		//
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.PI)
								.splineToLinearHeading(new Pose2d(-12, 0, Math.toRadians(0)), Math.toRadians(0))
								.build()
				)
		));
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.toRadians(-90))
								.splineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(0)), Math.toRadians(0))
								.build()
				)
		));
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.toRadians(0))
								.splineToLinearHeading(new Pose2d(0, -12, Math.toRadians(0)), Math.toRadians(0))
								.build()
				)
		));
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.toRadians(90))
								.splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)), Math.toRadians(0))
								.build()
				)
		));
		//
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.PI)
								.splineToLinearHeading(new Pose2d(-12, 0, Math.toRadians(0)), Math.toRadians(0))
								.build()
				)
		));
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.toRadians(-90))
								.splineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(0)), Math.toRadians(0))
								.build()
				)
		));
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.toRadians(0))
								.splineToLinearHeading(new Pose2d(0, -12, Math.toRadians(0)), Math.toRadians(0))
								.build()
				)
		));
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.toRadians(90))
								.splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)), Math.toRadians(0))
								.build()
				)
		));
	}
}
 */