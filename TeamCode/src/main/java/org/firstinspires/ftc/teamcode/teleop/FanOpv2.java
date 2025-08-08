package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.Utilities.RunInParallel;
import static org.firstinspires.ftc.teamcode.Utilities.RunSequentially;
import static org.firstinspires.ftc.teamcode.Utilities.WaitFor;
import static org.firstinspires.ftc.teamcode.Utilities.setTimeout;

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
import org.firstinspires.ftc.teamcode.InputSystem;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardwareNEW;
import org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.systems.ExtendoServoSystem;
import org.firstinspires.ftc.teamcode.systems.ExtendoMotorSystem;
import org.firstinspires.ftc.teamcode.systems.ExtendoSystem;
import org.firstinspires.ftc.teamcode.systems.IntakeSystem.IntakeDirection;
import org.firstinspires.ftc.teamcode.systems.LiftSystem;
import org.firstinspires.ftc.teamcode.systems.RotatorSystem;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;
import org.firstinspires.ftc.teamcode.systems.OpenCloseSystem;
import org.firstinspires.ftc.teamcode.systems.IntakeSystem;
import java.util.HashMap;

@TeleOp(name = "\uD83D\uDC79Fane Boss v2\uD83D\uDC79 Op", group = "TeleOp")
public final class FanOpv2 extends LinearOpMode
{
	private RobotHardwareNEW robot;
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

	public static class InputSystem
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
	private int liftLevel2 = 0;
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
		lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
		lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
		lift1.setDirection(DcMotor.Direction.REVERSE);
		lift2.setDirection(DcMotor.Direction.FORWARD);
		lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		lift1.setTargetPosition(0);
		lift2.setTargetPosition(0);
		lift1.setPower(0);
		lift2.setPower(0);
		lift1.setTargetPositionTolerance(20);
		lift2.setTargetPositionTolerance(20);
		lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		waitForStart();
		while (!isStopRequested())
		{
			telemetry.addData("Lift1 Current (mA)", lift1.getCurrent(CurrentUnit.MILLIAMPS));
			telemetry.addData("Lift2 Current (mA)", lift2.getCurrent(CurrentUnit.MILLIAMPS));

			telemetry.addData("Lift1 Encoder Position", lift1.getCurrentPosition());
			telemetry.addData("Lift2 Encoder Position", lift2.getCurrentPosition());
			telemetry.addData("LiftLevel2", liftLevel2);
			telemetry.update();

			if (lift1.isBusy()) lift1.setPower(1);
			else setTimeout(() -> {
				lift1.setPower(0);
			}, 150);
			if (lift2.isBusy()) lift2.setPower(1);
			else setTimeout(() -> {
				lift2.setPower(0);
			}, 150);

			if (armInput.wasPressedThisFrame(Keybindings.Arm.BASKET_LOW_KEY))
			{
				liftLevel = 0;
				liftLevel2 = 0;
				//scoreTumbler.setPosition(0.84);
				//scoreSmallTumbler.setPosition(0.94);
			}
			else if (armInput.wasPressedThisFrame(Keybindings.Arm.BASKET_HIGH_KEY))
			{
				liftLevel = 900;
				liftLevel2 = 900;
				//scoreTumbler.setPosition(0.84);
				//scoreSmallTumbler.setPosition(0.94);
			}
			else if (armInput.wasPressedThisFrame(Keybindings.Arm.CHAMBER_HIGH_KEY))
			{
				liftLevel = 300;
				liftLevel2 = 300;
				//scoreTumbler.setPosition(0.84);
				//scoreSmallTumbler.setPosition(0.55);
			}


				lift1.setTargetPosition(liftLevel);
				lift2.setTargetPosition(liftLevel2);




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