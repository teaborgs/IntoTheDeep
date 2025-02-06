//package org.firstinspires.ftc.teamcode.autonomous;
//
//import static org.firstinspires.ftc.teamcode.Utilities.RunInParallel;
//import static org.firstinspires.ftc.teamcode.Utilities.RunSequentially;
//import static org.firstinspires.ftc.teamcode.Utilities.WaitFor;
//import static org.firstinspires.ftc.teamcode.Utilities.centimetersToInches;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.RobotHardware;
//import org.firstinspires.ftc.teamcode.systems.LiftSystem;
//import org.firstinspires.ftc.teamcode.systems.TumblerSystem;
//
//@Autonomous(name = "Autonom_Dreapta_D", group = "Auto")
//public class Autonom_Dreapta_D extends BaseOpMode
//{
//	private RobotHardware robot;
//
//	@Override
//	protected void OnInitialize()
//	{
//		robot = new RobotHardware(hardwareMap);
//		robot.init();
//
//		robot.lift.setLiftPower(1);
//		if (!robot.distanceSystem.StatusOk())
//		{
//			telemetry.addLine("[ERROR] Distance sensor timeout!");
//			telemetry.update();
//		}
//	}
//
//	@Override
//	protected void OnRun()
//	{
//		// Deliver preload specimen
//		Actions.runBlocking(RunSequentially(
//				RunInParallel(
//						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
//								.setTangent(Math.PI)
//								.splineToLinearHeading(new Pose2d(-22.8, -4.7, Math.toRadians(0)), Math.toRadians(0))
//								.build(),
//						robot.lift.moveLiftTo(LiftSystem.LiftLevel.CHAMBER_HIGH),
//						robot.scoreRotator.closeAction(),
//						robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOLD)
//				),
//				robot.lift.moveLiftTo(LiftSystem.LiftLevel.IDLE),
//				WaitFor(0.25),
//				robot.scoreClaw.openAction()
//		));
//		Actions.runBlocking(RunSequentially(
//				RunInParallel(
//						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
//								.setTangent(0)
//								.splineToLinearHeading(new Pose2d(-16, 26, Math.toRadians(0)), Math.toRadians(90))
//								.setTangent(Math.PI)
//								.splineToLinearHeading(new Pose2d(-46, 35, Math.toRadians(0)), Math.toRadians(0))
//								.setTangent(0)
//								.splineToLinearHeading(new Pose2d(-14, 35, Math.toRadians(0)), Math.toRadians(0))
//								.setTangent(Math.PI)
//								.splineToLinearHeading(new Pose2d(-52, 40, Math.toRadians(0)), Math.toRadians(0))
//								.setTangent(0)
//								.splineToLinearHeading(new Pose2d(-14, 46, Math.toRadians(0)), Math.toRadians(0))
//								.setTangent(Math.PI)
//								.splineToLinearHeading(new Pose2d(-50, 49, Math.toRadians(0)), Math.toRadians(0))
//								.setTangent(0)
//								.splineToLinearHeading(new Pose2d(-14, 49, Math.toRadians(0)), Math.toRadians(0))
//								.build()
//				)
//		));
//		Actions.runBlocking(RunSequentially(
//				RunInParallel(
//						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
//								.setTangent(Math.PI)
//								.splineToLinearHeading(new Pose2d(0, 28, Math.toRadians(0)), Math.toRadians(0))
//								.build(),
//						robot.scoreRotator.openAction(),
//						robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.BUSY),
//						robot.scoreStumbler.setDestinationAction(TumblerSystem.TumblerDestination.BUSY)
//				),
//				WaitFor(0.5),
//				robot.scoreClaw.closeAction(),
//				WaitFor(100)
//		));
//		Actions.runBlocking(RunSequentially(
//				RunInParallel(
//						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
//								.setTangent(Math.PI)
//								.splineToLinearHeading(new Pose2d(-27.75, 0, Math.toRadians(0)), Math.toRadians(0))
//								.build(),
//						robot.lift.moveLiftTo(LiftSystem.LiftLevel.CHAMBER_HIGH),
//						robot.scoreRotator.closeAction(),
//						robot.scoreStumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOLD),
//						robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOLD)
//				),
//				robot.lift.moveLiftTo(LiftSystem.LiftLevel.IDLE),
//				WaitFor(0.25),
//				robot.scoreClaw.openAction(),
//				WaitFor(100)
//		));
//		/*
//		Actions.runBlocking(RunSequentially(
//				RunInParallel(
//						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
//								.setTangent(0)
//								.splineToLinearHeading(new Pose2d(-16.85, 21.85, Math.toRadians(45)), Math.toRadians(0))
//								.build()
//				),
//				WaitFor(100)
//		));
//		 */
//	}
//}