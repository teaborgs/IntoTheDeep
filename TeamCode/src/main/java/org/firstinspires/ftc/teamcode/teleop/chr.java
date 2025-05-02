/*package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.Utilities.setTimeout;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.InputSystem;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.systems.ExtendoSystem;
import org.firstinspires.ftc.teamcode.systems.LiftSystem;
import org.firstinspires.ftc.teamcode.systems.RotatorSystem;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.List;

@TeleOp(name = "\uD83D\uDC79Fane\uD83D\uDC79 Op", group = "TeleOp")
public final class FaneOp extends BaseOpMode
{
	private InputSystem driveInput, armInput;

	private RobotHardware robot;

	private static class Keybindings
	{
		public static class Arm
		{
			public static final InputSystem.Key BASKET_LOW_KEY = new InputSystem.Key("dpad_down");
			public static final InputSystem.Key BASKET_HIGH_KEY = new InputSystem.Key("dpad_up");
			public static final InputSystem.Key CHAMBER_HIGH_KEY = new InputSystem.Key("dpad_left");
			public static final InputSystem.Key PRIMARY_KEY = new InputSystem.Key("a");
			public static final InputSystem.Key SPECIMEN_KEY = new InputSystem.Key("b");
			public static final InputSystem.Key SUSPEND_KEY = new InputSystem.Key("y");
			public static final InputSystem.Key CANCEL_SUSPEND_KEY = new InputSystem.Key("x");
		}

		public static class Drive
		{
			public static final InputSystem.Key TURBO_KEY = new InputSystem.Key("right_bumper");
			public static final InputSystem.Key SUPPRESS_KEY = new InputSystem.Key("left_bumper");
			public static final InputSystem.Axis DRIVE_X = new InputSystem.Axis("left_stick_x");
			public static final InputSystem.Axis DRIVE_Y = new InputSystem.Axis("left_stick_y");
			public static final InputSystem.Axis DRIVE_ROT_L = new InputSystem.Axis("left_trigger");
			public static final InputSystem.Axis DRIVE_ROT_R = new InputSystem.Axis("right_trigger");
			public static final InputSystem.Key EXTENDO_FULL_KEY = new InputSystem.Key("dpad_up");
			public static final InputSystem.Key EXTENDO_HALF_KEY = new InputSystem.Key("dpad_left");
			public static final InputSystem.Key EXTENDO_ZERO_KEY = new InputSystem.Key("dpad_down");
			public static final InputSystem.Axis ROTATOR_ANGLE = new InputSystem.Axis("right_stick_x");
			public static final InputSystem.Key GRAB_TRANSFER_KEY = new InputSystem.Key("a");
			public static final InputSystem.Key GRAB_HOLD_KEY = new InputSystem.Key("b");
			public static final InputSystem.Key GRAB_WALL_KEY = new InputSystem.Key("x");
			public static final InputSystem.Key GRAB_RESET_KEY = new InputSystem.Key("y");
		}
	}

	@Override

	protected void OnInitialize()
	{
		driveInput = input1;
		armInput = input2;
	}

	@Override
	protected void OnStart()
	{
		robot = new RobotHardware(hardwareMap);
		robot.init();
	}

	@Override
	protected void OnRun()
	{
		robot.lift.autoPower();
		Suspender();
		if (suspending) return;
		Drive();
		if (suspendState == Utilities.State.BUSY) return;
		Extendo();
		Scorer();
	}

	private void Drive()
	{
		float speed = 0.7f;
		if (driveInput.isPressed(Keybindings.Drive.TURBO_KEY)) speed = 1.0f;
		else if (driveInput.isPressed(Keybindings.Drive.SUPPRESS_KEY)) speed = 0.4f;
		robot.drivetrain.setDrivePowers(
				new PoseVelocity2d(new Vector2d(driveInput.getValue(Keybindings.Drive.DRIVE_Y),
						driveInput.getValue(Keybindings.Drive.DRIVE_X)).times(-speed * (liftedSpecimen ? -1 : 1)),
						(driveInput.getValue(Keybindings.Drive.DRIVE_ROT_L) - driveInput.getValue(Keybindings.Drive.DRIVE_ROT_R)) * speed
				)
		);
	}

	private Utilities.State extendoState = Utilities.State.IDLE;
	private Utilities.State scorerState = Utilities.State.IDLE;
	private boolean rotatorLocked = false;
	private boolean waitingToDrop = false;
	private boolean grabbedWall = false;
	private boolean liftedSpecimen = false;

	private double rotatorPos = RotatorSystem.ROTATOR_BASE;
	private ExtendoSystem.ExtendoLevel extendoLevel = ExtendoSystem.ExtendoLevel.RETRACTED;
	private LiftSystem.LiftLevel liftLevel = LiftSystem.LiftLevel.BASKET_HIGH;
	private boolean liftOverride = false;

	private void Extendo()
	{
		// Rotator control
		if (!rotatorLocked)
		{
			rotatorPos += (float) (driveInput.getValue(Keybindings.Drive.ROTATOR_ANGLE) / 100f);
			rotatorPos = com.acmerobotics.roadrunner.Math.clamp(rotatorPos, 0, 1);
			robot.rotator.setPosition(rotatorPos);
		}

		if (extendoState == Utilities.State.BUSY)
		{
			if (driveInput.wasPressedThisFrame(Keybindings.Drive.EXTENDO_FULL_KEY))
				extendoLevel = ExtendoSystem.ExtendoLevel.EXTENDED;
			else if (driveInput.wasPressedThisFrame(Keybindings.Drive.EXTENDO_HALF_KEY))
				extendoLevel = ExtendoSystem.ExtendoLevel.HALF;
			else if (driveInput.wasPressedThisFrame(Keybindings.Drive.EXTENDO_ZERO_KEY))
				extendoLevel = ExtendoSystem.ExtendoLevel.RETRACTED;

			if (robot.extendo.getExtendoLevel() != extendoLevel)
				robot.extendo.extend(extendoLevel);
		}

		if (driveInput.wasPressedThisFrame(Keybindings.Drive.GRAB_RESET_KEY))
		{
			robot.extendo.extend(ExtendoSystem.ExtendoLevel.RETRACTED);
			robot.claw.open();
			robot.tumbler.setDestination(TumblerSystem.TumblerDestination.IDLE);
			robot.smallTumbler.setDestination(TumblerSystem.TumblerDestination.IDLE);
			robot.lift.setLiftLevel(LiftSystem.LiftLevel.IDLE);
			liftOverride = false;
			grabbedWall = false;
			liftedSpecimen = false;
		}

		// Grabbing logic
		if (driveInput.wasPressedThisFrame(Keybindings.Drive.GRAB_WALL_KEY))
		{if (!grabbedWall) // Grab
		{

			setTimeout(() -> {
				liftOverride = true;
				robot.lift.setLiftLevel(LiftSystem.LiftLevel.CLEAR_SPECIMEN);
				grabbedWall = true;
			}, 150);
		}
		else if (!liftedSpecimen)
		{
			robot.lift.setLiftLevel(LiftSystem.LiftLevel.CHAMBER_HIGH_FIXED_CLAW);
			liftedSpecimen = true;
		}
		else
		{
			robot.lift.setLiftLevel(LiftSystem.LiftLevel.CHAMBER_HIGH_PLACE_FIXED_CLAW);

			setTimeout(() -> {
				while (robot.lift.isLiftBusy()) sleep(15);

				setTimeout(() -> {
					scorerState = Utilities.State.IDLE;
					liftOverride = false;
					grabbedWall = false;
					liftedSpecimen = false;
				}, 150);
			}, 600);
		}
		}

		if (extendoState == Utilities.State.IDLE
				&& !grabbedWall
				&& (driveInput.wasPressedThisFrame(Keybindings.Drive.GRAB_TRANSFER_KEY)
				|| driveInput.wasPressedThisFrame(Keybindings.Drive.GRAB_HOLD_KEY)))
		{
			extendoState = Utilities.State.BUSY;
			extendoLevel = ExtendoSystem.ExtendoLevel.EXTENDED;
			rotatorLocked = false;

			robot.tumbler.setDestination(TumblerSystem.TumblerDestination.HOVER);
			robot.smallTumbler.setDestination(TumblerSystem.TumblerDestination.HOVER);
		}
		else if (extendoState == Utilities.State.BUSY)
		{
			// Grab and transfer
			if (driveInput.wasPressedThisFrame(Keybindings.Drive.GRAB_TRANSFER_KEY))
			{
				rotatorLocked = true;

				robot.scoreClaw.open();
				robot.tumbler.setDestination(TumblerSystem.TumblerDestination.BUSY);
				robot.smallTumbler.setDestination(TumblerSystem.TumblerDestination.BUSY);
				setTimeout(() -> {
					robot.claw.close();
					setTimeout(() -> {
						extendoLevel = ExtendoSystem.ExtendoLevel.RETRACTED;
						robot.extendo.extend(extendoLevel);

						robot.tumbler.setDestination(TumblerSystem.TumblerDestination.TRANSFER);
						robot.smallTumbler.setDestination(TumblerSystem.TumblerDestination.TRANSFER);


						rotatorPos = RotatorSystem.ROTATOR_BASE;
						robot.rotator.setPosition(rotatorPos);

						setTimeout(() -> {
							robot.scoreTumbler.setDestination(TumblerSystem.TumblerDestination.TRANSFER);
							robot.scoreStumbler.setDestination(TumblerSystem.TumblerDestination.TRANSFER);
							robot.scoreClaw.close();
							setTimeout(() -> {
								robot.claw.open();
								setTimeout(() -> {
									liftOverride = false;
									robot.tumbler.setDestination(TumblerSystem.TumblerDestination.IDLE);
									robot.smallTumbler.setDestination(TumblerSystem.TumblerDestination.IDLE);

									robot.scoreTumbler.setDestination(TumblerSystem.TumblerDestination.IDLE);
									robot.scoreStumbler.setDestination(TumblerSystem.TumblerDestination.IDLE);

									extendoState = Utilities.State.IDLE;
								}, 200);
							}, 150);
						}, 800);
					}, 150);
				}, 150);
			}
			// Grab and hold
			else if (driveInput.wasPressedThisFrame(Keybindings.Drive.GRAB_HOLD_KEY))
			{
				// Grab
				if (!waitingToDrop)
				{
					rotatorLocked = true;

					robot.tumbler.setDestination(TumblerSystem.TumblerDestination.BUSY);
					robot.smallTumbler.setDestination(TumblerSystem.TumblerDestination.BUSY);
					setTimeout(() -> {
						robot.claw.close();
						setTimeout(() -> {
							extendoLevel = ExtendoSystem.ExtendoLevel.RETRACTED;
							robot.extendo.extend(extendoLevel);

							robot.tumbler.setDestination(TumblerSystem.TumblerDestination.HOVER);
							robot.smallTumbler.setDestination(TumblerSystem.TumblerDestination.HOLD);

							rotatorPos = RotatorSystem.ROTATOR_BASE;
							robot.rotator.setPosition(rotatorPos);

							waitingToDrop = true;
						}, 150);
					}, 200);
				}
				else // Release
				{
					robot.claw.open();
					setTimeout(() -> {
						robot.tumbler.setDestination(TumblerSystem.TumblerDestination.IDLE);
						robot.smallTumbler.setDestination(TumblerSystem.TumblerDestination.IDLE);

						extendoLevel = ExtendoSystem.ExtendoLevel.RETRACTED;
						robot.extendo.extend(extendoLevel);

						extendoState = Utilities.State.IDLE;
						waitingToDrop = false;
					}, 150);
				}
			}
		}
	}

	private void Scorer()
	{
		// Lift control logic
		if (!liftOverride)
		{
			// Lift level
			if (armInput.wasPressedThisFrame(Keybindings.Arm.BASKET_LOW_KEY))
			{
				liftLevel = LiftSystem.LiftLevel.BASKET_LOW;
				if (scorerState == Utilities.State.BUSY)
					robot.scoreTumbler.setDestination(TumblerSystem.TumblerDestination.BUSY);
			}
			else if (armInput.wasPressedThisFrame(Keybindings.Arm.BASKET_HIGH_KEY))
			{
				liftLevel = LiftSystem.LiftLevel.BASKET_HIGH;
				if (scorerState == Utilities.State.BUSY)
					robot.scoreTumbler.setDestination(TumblerSystem.TumblerDestination.BUSY);
			}
			else if (armInput.wasPressedThisFrame(Keybindings.Arm.CHAMBER_HIGH_KEY))
			{
				liftLevel = LiftSystem.LiftLevel.CHAMBER_HIGH;
				if (scorerState == Utilities.State.BUSY)
					robot.scoreTumbler.setDestination(TumblerSystem.TumblerDestination.BUSY);
			}

			// Lift central control
			if (scorerState == Utilities.State.BUSY)
				robot.lift.setLiftLevel(liftLevel);
			else
				robot.lift.setLiftLevel(LiftSystem.LiftLevel.IDLE);
		}

		// Scoring logic
		if (armInput.wasPressedThisFrame(Keybindings.Arm.PRIMARY_KEY))
		{
			if (scorerState == Utilities.State.IDLE)
			{
				scorerState = Utilities.State.BUSY;
				if (grabbedWall) liftOverride = false;
				else
					setTimeout(() -> robot.scoreTumbler.setDestination(TumblerSystem.TumblerDestination.BUSY), 400);
			}
			else
			{
				robot.scoreClaw.open();
				setTimeout(() -> {
					robot.scoreTumbler.setDestination(TumblerSystem.TumblerDestination.IDLE);
					setTimeout(() -> scorerState = Utilities.State.IDLE, 300);
				}, 150);
			}
		}
		if (armInput.wasPressedThisFrame(Keybindings.Arm.SPECIMEN_KEY))
		{
			if (scorerState == Utilities.State.IDLE)
			{
				scorerState = Utilities.State.BUSY;

				liftLevel = LiftSystem.LiftLevel.CHAMBER_HIGH;
				setTimeout(() -> robot.scoreTumbler.setDestination(TumblerSystem.TumblerDestination.BUSY), 400);

				if (grabbedWall) liftOverride = false;
			}
			else
			{
				liftLevel = LiftSystem.LiftLevel.CHAMBER_HIGH_PLACE;

				setTimeout(() -> {
					robot.scoreClaw.open();
					setTimeout(() -> {
						robot.scoreTumbler.setDestination(TumblerSystem.TumblerDestination.IDLE);
						setTimeout(() -> scorerState = Utilities.State.IDLE, 500);
					}, 300);
				}, 500);
			}
		}
	}

	private Utilities.State suspendState = Utilities.State.IDLE;
	private boolean suspending = false;

	private void Suspender()
	{
		if (armInput.wasPressedThisFrame(Keybindings.Arm.CANCEL_SUSPEND_KEY) && suspendState == Utilities.State.BUSY)
		{
			robot.lift.setLiftLevel(LiftSystem.LiftLevel.IDLE);
			suspendState = Utilities.State.IDLE;
			return;
		}

		if (!armInput.wasPressedThisFrame(Keybindings.Arm.SUSPEND_KEY)) return;

		if (suspendState == Utilities.State.IDLE)
		{
			suspendState = Utilities.State.BUSY;
			robot.lift.setLiftLevel(LiftSystem.LiftLevel.SUSPEND);
		}
		else
		{
			suspending = true;
			robot.lift.setLiftLevel(LiftSystem.LiftLevel.IDLE);

			robot.axle1.close();
			robot.axle2.close();

			sleep(200);

			robot.drivetrain.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			robot.drivetrain.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			robot.drivetrain.leftBack.setPower(1);
			robot.drivetrain.rightBack.setPower(1);
		}
	}

	private void SampleDetection() // Compute rotator angle
	{
		if (rotatorLocked)
			return;

		List<ColorBlobLocatorProcessor.Blob> blobs = null; //processor.getBlobs();
		ColorBlobLocatorProcessor.Util.filterByArea(1500, 200000, blobs);

		if (blobs.isEmpty())
			return;

		// Find biggest blob
		ColorBlobLocatorProcessor.Blob blob = blobs.get(0);
		for (ColorBlobLocatorProcessor.Blob b : blobs)
		{
			if (b.getContourArea() > blob.getContourArea())
				blob = b;
		}

		double angle = blob.getBoxFit().angle;
		org.opencv.core.Size size = blob.getBoxFit().size;

		// Math the angle
		if (size.width > size.height)
			angle += 90.0;

		// Command servo
		if (angle > 100)
			rotatorPos -= 0.00005 * (Math.abs(angle - 90));
		else if (angle < 80)
			rotatorPos += 0.00005 * (Math.abs(angle - 90));

		rotatorPos = com.acmerobotics.roadrunner.Math.clamp(rotatorPos, 0.0, 1.0);
	}

	@Override
	protected void OnTelemetry(Telemetry telemetry)
	{
		if (suspendState == Utilities.State.BUSY)
		{
			telemetry.addLine("[INFO] In Suspend Mode");
			telemetry.addLine("[INFO] Press " + Keybindings.Arm.SUSPEND_KEY + " again to suspend");
			telemetry.addLine("[INFO] Press " + Keybindings.Arm.CANCEL_SUSPEND_KEY + " to cancel");
		}
		else
		{
			telemetry.addLine("[INFO] Welcome Drivers!");
			telemetry.addLine("[INFO] Press " + Keybindings.Arm.SUSPEND_KEY + " to enter Suspend Mode");
			telemetry.addLine("[INFO] Press " + Keybindings.Arm.BASKET_LOW_KEY + " to move to Basket Low");
			telemetry.addLine("[INFO] Press " + Keybindings.Arm.BASKET_HIGH_KEY + " to move to Basket High");
			telemetry.addLine("[INFO] Press " + Keybindings.Arm.CHAMBER_HIGH_KEY + " to move to Submersible High");
			telemetry.addLine("[INFO] Press " + Keybindings.Arm.PRIMARY_KEY + " to score");
			telemetry.addLine("[INFO] Press " + Keybindings.Drive.GRAB_TRANSFER_KEY + " to grab");
		}
	}
}*/