/*
package org.firstinspires.ftc.teamcode.autonomous;

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
import org.firstinspires.ftc.teamcode.RobotHardwareAutoDreapta;
import org.firstinspires.ftc.teamcode.systems.LiftSystem;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;

@Autonomous(name = "Autonom_Dreapta_Constant", group = "Auto")
public class Autonom_Dreapta_Constant extends BaseOpMode
{
	private RobotHardwareAutoDreapta robot;

	@Override
	protected void OnInitialize()
	{
		robot = new RobotHardwareAutoDreapta(hardwareMap);
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
								.splineToLinearHeading(new Pose2d(-23, -4.7, Math.toRadians(0)), Math.toRadians(0))
								.build(),
						robot.lift.moveLiftTo(LiftSystem.LiftLevel.CHAMBER_HIGH),
						robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOLD)
				),
				WaitFor(0.15),
				robot.lift.moveLiftTo(LiftSystem.LiftLevel.IDLE),
				WaitFor(0.1),
				robot.scoreClaw.openAction()
		));
		Actions.runBlocking(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.toRadians(45))
								.splineToConstantHeading(new Vector2d(-23.6, 25), Math.toRadians(90), (pose2dDual, posePath, v) -> 150, (pose2dDual, posePath, v) -> new MinMax(-150, 150))
								.setTangent(Math.toRadians(180))
								.splineToConstantHeading(new Vector2d(-46, 38), Math.toRadians(45), (pose2dDual, posePath, v) -> 150, (pose2dDual, posePath, v) -> new MinMax(-150, 150))
								.setTangent(0)
								.lineToX(-9.84)
								.setTangent(Math.toRadians(180))
								.splineToConstantHeading(new Vector2d(-45.27, 46), Math.toRadians(45), (pose2dDual, posePath, v) -> 150, (pose2dDual, posePath, v) -> new MinMax(-150, 150))
								.setTangent(0)
								.lineToX(-9.84)
								.setTangent(Math.toRadians(180))
								.splineToConstantHeading(new Vector2d(-45.27, 55), Math.toRadians(45), (pose2dDual, posePath, v) -> 150, (pose2dDual, posePath, v) -> new MinMax(-150, 150))
								.setTangent(0)
								.lineToX(-9.84)
								.setTangent(Math.toRadians(270))
								.splineToLinearHeading(new Pose2d(-0.78, 28.5, Math.toRadians(0)), Math.toRadians(0))
								.setTangent(Math.toRadians(0))
								.lineToX(0)
								.build(),
						robot.scoreRotator.closeAction(),
						robot.scoreStumbler.setDestinationAction(TumblerSystem.TumblerDestination.BUSY),
						RunSequentially(
								WaitFor(0.25),
								robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.BUSY)

						)
				)
		);

		Actions.runBlocking(RunSequentially(
				robot.scoreClaw.closeAction(),
				WaitFor(0.2)
		));


		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.PI/3.75)
								.lineToX(-28.5)
								//.lineToX(-5)
								.build(),
						robot.lift.moveLiftTo(LiftSystem.LiftLevel.CHAMBER_HIGH),
						robot.scoreRotator.openAction(),
						robot.scoreStumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOLD),
						robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOLD)
				),
				WaitFor(0.05),
				robot.lift.moveLiftTo(LiftSystem.LiftLevel.IDLE),
				WaitFor(0.15),
				robot.scoreClaw.openAction()
		));
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.PI/3.56)
								.lineToX(0.2)
								//.lineToX(-5)
								.build(),
						robot.scoreRotator.closeAction(),
						robot.scoreStumbler.setDestinationAction(TumblerSystem.TumblerDestination.BUSY),
						RunSequentially(
								WaitFor(0.25),
								robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.BUSY)
						)
				),
				WaitFor(0.05),
				robot.scoreClaw.closeAction(),
				WaitFor(0.2)
		));
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.PI/3.6)
								.lineToX(-28.7)
								//.lineToX(-5)
								.build(),
						robot.lift.moveLiftTo(LiftSystem.LiftLevel.CHAMBER_HIGH),
						robot.scoreRotator.openAction(),
						robot.scoreStumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOLD),
						robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOLD)
				),
				WaitFor(0.05),
				robot.lift.moveLiftTo(LiftSystem.LiftLevel.IDLE),
				WaitFor(0.15),
				robot.scoreClaw.openAction()
		));
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.PI/3.6)
								.lineToX(0.2)
								//.lineToX(-5)
								.build(),
						robot.scoreRotator.closeAction(),
						robot.scoreStumbler.setDestinationAction(TumblerSystem.TumblerDestination.BUSY),
						RunSequentially(
								WaitFor(0.25),
								robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.BUSY)
						)
				),
				WaitFor(0.05),
				robot.scoreClaw.closeAction(),
				WaitFor(0.2)
		));
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.PI/3.72)
								.lineToX(-28.7)
								//.lineToX(-5)
								.build(),
						robot.lift.moveLiftTo(LiftSystem.LiftLevel.CHAMBER_HIGH),
						robot.scoreRotator.openAction(),
						robot.scoreStumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOLD),
						robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOLD)
				),
				WaitFor(0.05),
				robot.lift.moveLiftTo(LiftSystem.LiftLevel.IDLE),
				WaitFor(0.15),
				robot.scoreClaw.openAction()

		));
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.PI/3.72)
								.lineToX(0.2)
								//.lineToX(-5)
								.build(),
						robot.scoreRotator.closeAction(),
						robot.scoreStumbler.setDestinationAction(TumblerSystem.TumblerDestination.BUSY),
						RunSequentially(
								WaitFor(0.25),
								robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.BUSY)
						)
				),
				WaitFor(0.05),
				robot.scoreClaw.closeAction(),
				WaitFor(0.2)
		));
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.PI/3.768)
								.lineToX(-28.7)
								//.lineToX(-5)
								.build(),
						robot.lift.moveLiftTo(LiftSystem.LiftLevel.CHAMBER_HIGH),
						robot.scoreRotator.openAction(),
						robot.scoreStumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOLD),
						robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOLD)
				),
				WaitFor(0.05),
				robot.lift.moveLiftTo(LiftSystem.LiftLevel.IDLE),
				WaitFor(0.15),
				robot.scoreClaw.openAction()
		));
		Actions.runBlocking(RunSequentially(
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(Math.PI/3.86)
								.lineToX(0.2)
								//.lineToX(-5)
								.build(),
						robot.scoreRotator.closeAction(),
						robot.scoreStumbler.setDestinationAction(TumblerSystem.TumblerDestination.BUSY),
						RunSequentially(
								WaitFor(0.25),
								robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.BUSY)
						)
				)
				//WaitFor(0.05),
				//robot.scoreClaw.closeAction(),
				//WaitFor(0.2)
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
