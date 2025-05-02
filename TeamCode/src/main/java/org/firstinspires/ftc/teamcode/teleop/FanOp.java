package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.Utilities.RunInParallel;
import static org.firstinspires.ftc.teamcode.Utilities.RunSequentially;
import static org.firstinspires.ftc.teamcode.Utilities.WaitFor;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.systems.LiftSystem;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;

import java.util.HashMap;

@TeleOp(name = "\uD83D\uDC79Fane Boss\uD83D\uDC79 Op", group = "TeleOp")
public final class FanOp extends LinearOpMode
{
	private RobotHardware robot;
	private static class Keybindings
	{
		public static class Arm
		{
			public static final InputSystem.Key BASKET_LOW_KEY = new InputSystem.Key("dpad_down");
			public static final InputSystem.Key BASKET_HIGH_KEY = new InputSystem.Key("dpad_up");
			public static final InputSystem.Key CHAMBER_HIGH_KEY = new InputSystem.Key("dpad_left");
			public static final InputSystem.Key PRIMARY_KEY = new InputSystem.Key("a");
			public static final InputSystem.Key SPECIMEN_HOLD = new InputSystem.Key("x");
			public static final InputSystem.Key SPECIMEN_KEY = new InputSystem.Key("b");
			public static final InputSystem.Key SUSPEND_KEY = new InputSystem.Key("y");
			public static final InputSystem.Key CANCEL_SUSPEND_KEY = new InputSystem.Key("x");
			public static final InputSystem.Key RIGHT_KEY = new InputSystem.Key("right_bumper");
			public static final InputSystem.Key LEFT_KEY = new InputSystem.Key("left_bumper");
			public static final InputSystem.Axis DRIVE_LIFT = new InputSystem.Axis("left_stick_x");
		}

		public static class Drive
		{
			public static final InputSystem.Key TURBO_KEY = new InputSystem.Key("dpad_right");
			public static final InputSystem.Key SUPPRESS_KEY = new InputSystem.Key("left_bumper");
			public static final InputSystem.Axis DRIVE_X = new InputSystem.Axis("left_stick_x");
			public static final InputSystem.Axis DRIVE_Y = new InputSystem.Axis("left_stick_y");
			public static final InputSystem.Axis DRIVE_ROT_L = new InputSystem.Axis("left_trigger");
			public static final InputSystem.Axis DRIVE_ROT_R = new InputSystem.Axis("right_trigger");
			public static final InputSystem.Key EXTENDO_FULL_KEY = new InputSystem.Key("right_bumper");
			public static final InputSystem.Key EXTENDO_HALF_KEY = new InputSystem.Key("dpad_left");
			public static final InputSystem.Key EXTENDO_ZERO_KEY = new InputSystem.Key("dpad_down");
			public static final InputSystem.Axis ROTATOR_ANGLE = new InputSystem.Axis("right_stick_x");
			public static final InputSystem.Key GRAB_TRANSFER_KEY = new InputSystem.Key("a");
			public static final InputSystem.Key GRAB_HOLD_KEY = new InputSystem.Key("b");
			public static final InputSystem.Key GRAB_WALL_KEY = new InputSystem.Key("x");
			public static final InputSystem.Key AUTO_MOVE = new InputSystem.Key("y");
		}
	}

	private static class InputSystem
	{
		private static final float DEADZONE = 0.75f;

		private final Gamepad gamepad;
		private final Class<? extends Gamepad> gamepadClass;
		private final Telemetry telemetry;
		private final HashMap<Key, Boolean> keyStates = new HashMap<>();

		public InputSystem(Gamepad gamepad)
		{
			this(gamepad, null);
		}

		public InputSystem(Gamepad gamepad, Telemetry telemetry)
		{
			this.gamepad = gamepad;
			this.gamepadClass = gamepad.getClass();
			this.telemetry = telemetry;
		}

		public boolean wasPressedThisFrame(Key key)
		{
			try
			{
				boolean currentState = gamepadClass.getField(key.getId()).getBoolean(gamepad);
				if (!keyStates.containsKey(key)) keyStates.put(key, currentState);
				else
				{
					Boolean state = keyStates.get(key);
					if (currentState && Boolean.FALSE.equals(state)) keyStates.put(key, true);
					else if (currentState && Boolean.TRUE.equals(state)) return false;
					else if (!currentState || Boolean.TRUE.equals(state)) keyStates.put(key, false);
				}
				return Boolean.TRUE.equals(keyStates.get(key));
			} catch (Exception e)
			{
				return false;
			}
		}

		public boolean wasPressedThisFrame(String key)
		{
			return wasPressedThisFrame(new Key(key));
		}

		public boolean wasPressedThisFrame(BindingCombo keyCombo)
		{
			for (Binding key : keyCombo.getBindings())
				if (key instanceof Key && !wasPressedThisFrame((Key) key)) return false;
				else if (key instanceof Axis && Math.abs(getValue((Axis) key)) <= DEADZONE)
					return false;
			return true;
		}

		public boolean isPressed(Key key)
		{
			try
			{
				return gamepadClass.getField(key.getId()).getBoolean(gamepad);
			} catch (Exception e)
			{
				return false;
			}
		}

		public boolean isPressed(String key)
		{
			return isPressed(new Key(key));
		}

		public boolean isPressed(BindingCombo keyCombo)
		{
			for (Binding key : keyCombo.getBindings())
				if (key instanceof Key && !isPressed((Key) key)) return false;
				else if (key instanceof Axis && Math.abs(getValue((Axis) key)) <= DEADZONE)
					return false;
			return true;
		}

		public double getValue(Axis key)
		{
			try
			{
				return gamepadClass.getField(key.getId()).getDouble(gamepad);
			} catch (Exception e)
			{
				return 0d;
			}
		}

		public double getValue(String key)
		{
			return getValue(new Axis(key));
		}

		public static abstract class Binding
		{
			private final String id;

			public Binding(String id)
			{
				this.id = id;
			}

			public String getId()
			{
				return id;
			}

			@NonNull
			@Override
			public String toString()
			{
				return id.toUpperCase();
			}
		}

		public static class Key extends Binding
		{
			public Key(String id)
			{
				super(id);
			}
		}

		public static class Axis extends Binding
		{
			public Axis(String id)
			{
				super(id);
			}
		}

		public static class BindingCombo extends Binding
		{
			private final Binding[] bindings;

			public BindingCombo(String id, Binding... bindings)
			{
				super(id);
				this.bindings = bindings;
				if (bindings.length == 0)
					throw new IllegalArgumentException();
				for (Binding binding : bindings)
					if (binding instanceof BindingCombo)
						throw new IllegalArgumentException();
			}

			public Binding[] getBindings()
			{
				return bindings;
			}
		}
	}

	private MecanumDrive drivetrain;
	private Servo extendo, tumbler, smallTumbler, rotator, claw, scoreClaw, scoreTumbler, scoreRotator, scoreSmallTumbler, axle1, axle2;
	private DcMotorEx lift1, lift2;
	private InputSystem driveInput, armInput;
	private boolean extendoState = false;
	private boolean scorerState = false;
	private boolean rotatorLocked = false;
	private boolean waitingToDrop = false;
	private boolean grabbedWall = false;
	private boolean liftedSpecimen = false;
	private double rotatorPos = 0.476;
	private int liftLevel = 0;
	private double extendoVal = 0;
	private boolean liftOverride = false;
	private boolean suspendState = false;
	private boolean suspending = false;
    private boolean holding = false;
	@Override
	public void runOpMode()
	{
		waitForStart();
		driveInput = new InputSystem(gamepad1);
		armInput = new InputSystem(gamepad2);
		drivetrain = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
		extendo = hardwareMap.get(Servo.class, "extendo");
		tumbler = hardwareMap.get(Servo.class, "tumbler");
		smallTumbler = hardwareMap.get(Servo.class, "sTumbler");
		rotator = hardwareMap.get(Servo.class, "rotator");
		claw = hardwareMap.get(Servo.class, "claw");
		scoreClaw = hardwareMap.get(Servo.class, "scoreClaw");
		scoreTumbler = hardwareMap.get(Servo.class, "scoreTumbler");
		scoreRotator = hardwareMap.get(Servo.class, "scoreRotator");
		scoreSmallTumbler = hardwareMap.get(Servo.class, "scoreStumbler");
		axle1 = hardwareMap.get(Servo.class, "axleServo1");
		axle2 = hardwareMap.get(Servo.class, "axleServo2");
		lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
		lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
		extendo.setPosition(0.637);
		tumbler.setPosition(0.14);
		smallTumbler.setPosition(0.99f);
		rotator.setPosition(0.476);
		claw.setPosition(0.61);
		scoreClaw.setPosition(0.48);
		scoreTumbler.setPosition(0.575);
		scoreRotator.setPosition(0.758f);
		scoreSmallTumbler.setPosition(0.43); //0.48
		axle1.setPosition(0.5);
		axle2.setPosition(0.5);
		lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		lift1.setTargetPosition(0);
		lift2.setTargetPosition(0);
		lift1.setPower(0);
		lift2.setPower(0);
		lift1.setTargetPositionTolerance(30);
		lift2.setTargetPositionTolerance(30);
		lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		waitForStart();
		while (!isStopRequested())
		{
			telemetry.addData("Lift1 Current (mA)", lift1.getCurrent(CurrentUnit.MILLIAMPS));
			telemetry.addData("Lift2 Current (mA)", lift2.getCurrent(CurrentUnit.MILLIAMPS));
			telemetry.addData("Extendo Position", extendo.getPosition());
			telemetry.addData("Tumbler Position", tumbler.getPosition());
			telemetry.addData("Small Tumbler Position", smallTumbler.getPosition());
			telemetry.addData("Rotator Position", rotator.getPosition());
			telemetry.addData("Claw Position", claw.getPosition());
			telemetry.addData("Score Claw Position", scoreClaw.getPosition());
			telemetry.addData("Score Tumbler Position", scoreTumbler.getPosition());
			telemetry.addData("Score Rotator Position", scoreRotator.getPosition());
			telemetry.addData("Score Small Tumbler Position", scoreSmallTumbler.getPosition());
			telemetry.addData("Axle1 Position", axle1.getPosition());
			telemetry.addData("Axle2 Position", axle2.getPosition());
			telemetry.addData("Lift1 Encoder Position", lift1.getCurrentPosition());
			telemetry.addData("Lift2 Encoder Position", lift2.getCurrentPosition());
			telemetry.update();
			/*
			double liftInput = armInput.getValue(Keybindings.Arm.DRIVE_LIFT);
			lift1.setPower(liftInput);
			lift2.setPower(liftInput);
			 */


			if (lift1.isBusy()) lift1.setPower(1);
			else lift1.setPower(0);
			if (lift2.isBusy()) lift2.setPower(1);
			else lift2.setPower(0);
			if (armInput.wasPressedThisFrame(Keybindings.Arm.CANCEL_SUSPEND_KEY) && suspendState)
			{
				lift1.setTargetPosition(0);
				lift2.setTargetPosition(0);
				suspendState = false;
			}

			if (armInput.wasPressedThisFrame(Keybindings.Arm.SUSPEND_KEY))
			{
				if (!suspendState)
				{
					suspendState = true;
					lift1.setTargetPosition(-600);
					lift2.setTargetPosition(-600);
				}
				else
				{
					suspending = true;
					lift1.setTargetPosition(0);
					lift2.setTargetPosition(0);
					axle1.setPosition(0.39f);
					axle2.setPosition(0.59f);
						setTimeout(() -> {
							drivetrain.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
							drivetrain.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
							drivetrain.leftBack.setPower(1f);
							drivetrain.rightBack.setPower(1f);
						},300);
						WaitFor(4000);

					/*drivetrain.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
					drivetrain.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
					drivetrain.leftFront.setPower(-1f);
					drivetrain.rightFront.setPower(-1f);

					sleep(2000);
					drivetrain.leftFront.setPower(0f);
					drivetrain.rightFront.setPower(0f);*/
				}
			}
			if (suspending) continue;
			float speed = 1f;
			if(armInput.isPressed(Keybindings.Arm.RIGHT_KEY) && armInput.isPressed(Keybindings.Arm.LEFT_KEY)){
				lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
				lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
				lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				lift1.setTargetPosition(0);
				lift2.setTargetPosition(0);
				lift1.setPower(0);
				lift2.setPower(0);
				lift1.setTargetPositionTolerance(30);
				lift2.setTargetPositionTolerance(30);
				lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			}
			if (driveInput.isPressed(Keybindings.Drive.TURBO_KEY)) speed = 1.0f;
			else if (driveInput.isPressed(Keybindings.Drive.SUPPRESS_KEY)) speed = 0.4f;
			drivetrain.setDrivePowers(
					new PoseVelocity2d(new Vector2d(driveInput.getValue(Keybindings.Drive.DRIVE_Y),
							driveInput.getValue(Keybindings.Drive.DRIVE_X)).times(-speed * (liftedSpecimen ? -1 : 1)),
							(driveInput.getValue(Keybindings.Drive.DRIVE_ROT_L) - driveInput.getValue(Keybindings.Drive.DRIVE_ROT_R)) * speed
					)
			);
			if (suspendState) continue;
			if (!rotatorLocked)
			{
				rotatorPos += (float) (driveInput.getValue(Keybindings.Drive.ROTATOR_ANGLE) / 100f);
				rotatorPos = com.acmerobotics.roadrunner.Math.clamp(rotatorPos, 0, 1);
				rotator.setPosition(rotatorPos);
			}

			if (extendoState)
			{
				if (driveInput.wasPressedThisFrame(Keybindings.Drive.EXTENDO_FULL_KEY))
					extendoVal = 0.455;
				else if (driveInput.wasPressedThisFrame(Keybindings.Drive.EXTENDO_HALF_KEY))
					extendoVal = 0.36955f;
				else if (driveInput.wasPressedThisFrame(Keybindings.Drive.EXTENDO_ZERO_KEY))
					extendoVal = 0.637;
				if (extendo.getPosition() != extendoVal)
					extendo.setPosition(extendoVal);
			}


			/*
			if (driveInput.wasPressedThisFrame(Keybindings.Drive.AUTO_MOVE)){
				Actions.runBlocking(RunSequentially(
						RunInParallel(
								robot.drivetrain.actionBuilder(robot.drivetrain.pose)
										.setTangent(Math.PI/3.7)
										.lineToX(-27.6)
										.build(),
								robot.lift.moveLiftTo(LiftSystem.LiftLevel.CHAMBER_HIGH),
								robot.scoreRotator.openAction(),
								robot.scoreStumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOLD),
								robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOLD)
						),
						WaitFor(0.1),
						robot.lift.moveLiftTo(LiftSystem.LiftLevel.IDLE),
						WaitFor(0.45),
						robot.scoreClaw.openAction()
				));
			}
			 */
			if (driveInput.wasPressedThisFrame(Keybindings.Drive.GRAB_WALL_KEY))
			{
				if (!grabbedWall)
				{
					scoreSmallTumbler.setPosition(0.77);
					setTimeout(() -> scoreTumbler.setPosition(0.38), 150);
					//scoreTumbler.setPosition(0.38);
					scoreClaw.setPosition(0.48);
					grabbedWall = true;
				}
				else
				{
					scoreClaw.setPosition(0.8);
					setTimeout(() -> {
						if (!scorerState)
						{
							scorerState = true;

							liftLevel = -395;
							scoreRotator.setPosition(0.06);
							scoreSmallTumbler.setPosition(0.5);
							scoreTumbler.setPosition(0.85);
							//setTimeout(() -> scoreTumbler.setPosition(0.412), 400);
							if (grabbedWall) liftOverride = false;
						}
					},200);
					/*
					setTimeout(() -> {
						scoreTumbler.setPosition(0.575);
						setTimeout(() -> scoreSmallTumbler.setPosition(0.926), 250);
					}, 400);
					 */
				}
			}

			if (!extendoState
					&& (driveInput.wasPressedThisFrame(Keybindings.Drive.GRAB_TRANSFER_KEY)
					|| driveInput.wasPressedThisFrame(Keybindings.Drive.GRAB_HOLD_KEY)))
			{
				extendoState = true;
				extendoVal = 0.455;
				rotatorLocked = false;

				tumbler.setPosition(0.14);
				smallTumbler.setPosition(0.41);
			}
			else if (extendoState)
			{
				if (driveInput.wasPressedThisFrame(Keybindings.Drive.GRAB_TRANSFER_KEY))
				{
					rotatorLocked = true;

					scoreClaw.setPosition(0.48);
					tumbler.setPosition(0.23); //0.49
					smallTumbler.setPosition(0.41);
					setTimeout(() -> {
						claw.setPosition(0.33);
						setTimeout(() -> {
							extendoVal = 0.637;
							extendo.setPosition(extendoVal);

							tumbler.setPosition(0.14); //0.525
							smallTumbler.setPosition(0.99f);

							rotatorPos = 0.476;
							rotator.setPosition(rotatorPos);

							setTimeout(() -> {
								scoreTumbler.setPosition(0.43);
								scoreSmallTumbler.setPosition(0.43);
								setTimeout(() -> {
									scoreClaw.setPosition(0.78f);
									claw.setPosition(0.61);
									setTimeout(() -> {
										liftOverride = false;
										tumbler.setPosition(0.14); //0.525
										smallTumbler.setPosition(0.99f);

										scoreTumbler.setPosition(0.575);
										scoreSmallTumbler.setPosition(0.43); //0.05

										extendoState = false;
									}, 200);
								}, 150);
							}, 700);
						}, 500);
					}, 150);
				}
				else if (driveInput.wasPressedThisFrame(Keybindings.Drive.GRAB_HOLD_KEY))
				{
					if (!waitingToDrop)
					{
						rotatorLocked = true;

						tumbler.setPosition(0.23); //0.49
						smallTumbler.setPosition(0.41);
						setTimeout(() -> {
							claw.setPosition(0.33);
							setTimeout(() -> {
								extendoVal = 0.637;
								extendo.setPosition(extendoVal);

								tumbler.setPosition(0.14); //0.525
								smallTumbler.setPosition(0.6);

								rotatorPos = 0.476;
								rotator.setPosition(rotatorPos);

								waitingToDrop = true;
							}, 300);
						}, 200);
					}
					else
					{
						claw.setPosition(0.61);
						setTimeout(() -> {
							tumbler.setPosition(0.14); //0.525
							smallTumbler.setPosition(0.99f);

							extendoVal = 0.637; //83
							extendo.setPosition(extendoVal);

							extendoState = false;
							waitingToDrop = false;
						}, 150);

					}
				}
			}

			if (!liftOverride)
			{
				if (armInput.wasPressedThisFrame(Keybindings.Arm.BASKET_LOW_KEY))
				{
					liftLevel = -580;
					scoreTumbler.setPosition(0.84);
					scoreSmallTumbler.setPosition(0.94);
				}
				else if (armInput.wasPressedThisFrame(Keybindings.Arm.BASKET_HIGH_KEY))
				{
					liftLevel = -1300;
					scoreTumbler.setPosition(0.84);
					scoreSmallTumbler.setPosition(0.94);
				}
				else if (armInput.wasPressedThisFrame(Keybindings.Arm.CHAMBER_HIGH_KEY))
				{
					liftLevel = -360;
					scoreTumbler.setPosition(0.84);
					scoreSmallTumbler.setPosition(0.55);
				}

				if (scorerState)
				{
					lift1.setTargetPosition(liftLevel);
					lift2.setTargetPosition(liftLevel);
				}
				else
				{
					lift1.setTargetPosition(0);
					lift2.setTargetPosition(0);

				}
			}

			if (armInput.wasPressedThisFrame(Keybindings.Arm.PRIMARY_KEY))
			{
				if (!scorerState)
				{
					scorerState = true;
					if (grabbedWall) liftOverride = false;
					else setTimeout(() -> {
						scoreTumbler.setPosition(0.8f);
					}, 400);
				}
				else
				{

					scoreClaw.setPosition(0.48);
					setTimeout(() -> {
						scoreClaw.setPosition(0.78);
						setTimeout(() ->{
							scorerState = false;
							scoreTumbler.setPosition(0.575);
							scoreSmallTumbler.setPosition(0.43);
						},300);
					}, 350);
				}
			}

			if (armInput.wasPressedThisFrame(Keybindings.Arm.SPECIMEN_KEY))
			{
					liftLevel = 0;
					setTimeout(() -> {
						grabbedWall=false;
						scoreClaw.setPosition(0.48);
						setTimeout(() -> {
							scoreTumbler.setPosition(0.575);
							scoreRotator.setPosition(0.758);
							setTimeout(() -> scorerState = false, 500);
						}, 300);
					}, 200);
			}
			if (armInput.wasPressedThisFrame(Keybindings.Arm.SPECIMEN_HOLD))
			{
				if(!holding)
				{
					liftLevel = -160;
					holding = true;
				}
				else{
					liftLevel = 0;
					holding=false;
					setTimeout(() -> {
						grabbedWall=false;
						scoreClaw.setPosition(0.48);
						setTimeout(() -> {
							scoreTumbler.setPosition(0.575);
							scoreRotator.setPosition(0.758);
							setTimeout(() -> scorerState = false, 500);
						}, 300);
					}, 500);
				}
			}
		}
	}

	private static void setTimeout(Runnable runnable, long delay)
	{
		new Thread(() -> {
			try
			{
				Thread.sleep(delay);
				runnable.run();
			} catch (Exception e)
			{
				e.printStackTrace();
			}
		}).start();
	}
}