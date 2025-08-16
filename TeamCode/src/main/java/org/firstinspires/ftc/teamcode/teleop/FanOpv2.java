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
import com.acmerobotics.roadrunner.ftc.MidpointTimer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name = "\uD83D\uDC79Fane Boss v2\uD83D\uDC79 Op", group = "TeleOp")
public final class FanOpv2 extends LinearOpMode
{
	private static Runnable runnable;
	private static long delay;
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
			public static final InputSystem.Axis EXTENDO_Y = new InputSystem.Axis("right_stick_y");
			public static final InputSystem.Axis DRIVE_ROT_L = new InputSystem.Axis("left_trigger");
			public static final InputSystem.Axis DRIVE_ROT_R = new InputSystem.Axis("right_trigger");

			public static final	InputSystem.Key INTAKE_FORWARD = new InputSystem.Key("right_bumper");
			public static final InputSystem.Key INTAKE_REVERSE = new InputSystem.Key("b");

			public static final InputSystem.Axis ROTATOR_ANGLE = new InputSystem.Axis("right_stick_x");
			public static final InputSystem.Key SPECIMEN = new InputSystem.Key("a");
			public static final InputSystem.Key MODE = new InputSystem.Key("x");
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
	private DcMotorEx lift1, lift2, extendo;
	private InputSystem driveInput, armInput;
	private TumblerSystem intakeTumbler, scoreTumbler;
	private Servo axle, suspender1, suspender2;
	private boolean extendoState = false;
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
	private boolean mode_val = false;
	private boolean state_specimen = false;
	private boolean scoring = false;
	private boolean lift_change_in_status = false;
	private ExtendoMotorSystem.ExtendoLevel extendoLevel;
	ExtendoMotorSystem.ExtendoLevel newLevel = extendoLevel;
	public IntakeDirection intakeDirection = IntakeDirection.STOP;
	private Utilities.State scorerState = Utilities.State.IDLE;
	@Override
	public void runOpMode()
	{
		waitForStart();
		robot = new RobotHardwareNEW(hardwareMap);
		robot.init();
		intakeTumbler = robot.intakeTumbler;
		driveInput = new InputSystem(gamepad1);
		armInput = new InputSystem(gamepad2);
		drivetrain = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

		//scoreTumbler = hardwareMap.get(Servo.class, "scoreTumbler");
		axle = hardwareMap.get(Servo.class, "axle");
		suspender1= hardwareMap.get(Servo.class, "suspender1");
		suspender2 = hardwareMap.get(Servo.class, "suspender2");
		axle.setPosition(0.5);
		lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
		lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
		extendo = hardwareMap.get(DcMotorEx.class, "extendo");
		lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		lift1.setDirection(DcMotor.Direction.FORWARD);
		lift2.setDirection(DcMotor.Direction.REVERSE);
		lift1.setPower(1);
		lift2.setPower(1);
		ElapsedTime timer = new ElapsedTime();
		while(timer.seconds()<0.2){

		}
		lift1.setPower(0);
		lift2.setPower(0);
		sleep(200);
		lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		lift1.setTargetPosition(0);
		lift2.setTargetPosition(0);
		lift1.setTargetPositionTolerance(20);
		lift2.setTargetPositionTolerance(20);
		lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		lift1.setDirection(DcMotor.Direction.REVERSE);
		lift2.setDirection(DcMotor.Direction.FORWARD);
		extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		extendo.setDirection(DcMotor.Direction.FORWARD);
		extendo.setPower(1.0);
		timer = new ElapsedTime();
		while(timer.seconds()<0.5){
			telemetry.addData("Motor Power", extendo.getPower());
			telemetry.addData("Motor Encoder Position", extendo.getCurrentPosition());
			telemetry.addData("Extendo Current (mA)", extendo.getCurrent(CurrentUnit.MILLIAMPS));
			telemetry.update();
		}
		extendo.setPower(0);
		extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		extendo.setDirection(DcMotor.Direction.REVERSE);
		extendo.setTargetPosition(0);
		extendo.setTargetPositionTolerance(20);
		//waitForStart();
		while (!isStopRequested())
		{
			telemetry.addData("Lift1 Current (mA)", lift1.getCurrent(CurrentUnit.MILLIAMPS));
			telemetry.addData("Lift2 Current (mA)", lift2.getCurrent(CurrentUnit.MILLIAMPS));
			telemetry.addData("Lift1 Encoder Position", lift1.getCurrentPosition());
			telemetry.addData("Lift2 Encoder Position", lift2.getCurrentPosition());
			telemetry.addData("Extendo Current (mA)", extendo.getCurrent(CurrentUnit.MILLIAMPS));
			telemetry.addData("Extendo Encoder Position", extendo.getCurrentPosition());
			telemetry.addData("Extendo joystick", driveInput.getValue(Keybindings.Drive.EXTENDO_Y));
			telemetry.addData("Y Pressed", armInput.isPressed(Keybindings.Arm.SUSPEND_KEY));
			telemetry.addData("Y Was Pressed This Frame", armInput.wasPressedThisFrame(Keybindings.Arm.SUSPEND_KEY));

			//telemetry.addData("Servo scoretumbler", scoreTumbler.getPosition());


			telemetry.update();

			float speed = 1f;
			if (driveInput.isPressed(Keybindings.Drive.SUPPRESS_KEY)) speed = 0.4f;
			drivetrain.setDrivePowers(
					new PoseVelocity2d(new Vector2d(driveInput.getValue(Keybindings.Drive.DRIVE_Y),
							driveInput.getValue(Keybindings.Drive.DRIVE_X)).times(-speed * (liftedSpecimen ? -1 : 1)),
							(driveInput.getValue(Keybindings.Drive.DRIVE_ROT_L) - driveInput.getValue(Keybindings.Drive.DRIVE_ROT_R)) * speed
					)
			);
			if(armInput.isPressed(Keybindings.Arm.RIGHT_KEY) && armInput.isPressed(Keybindings.Arm.LEFT_KEY)){
				lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
			}
			float joystick_extendo = (float) driveInput.getValue(Keybindings.Drive.EXTENDO_Y);
			if(joystick_extendo < 0){
				int nivel = (int) (joystick_extendo*-800);
				extendo.setTargetPosition(nivel);
				extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				extendo.setPower(1.0);
			}
			else if(joystick_extendo == 0){
				int nivel = (int) (0);
				extendo.setTargetPosition(nivel);
				extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				extendo.setPower(1.0);
			}


			if(driveInput.isPressed(Keybindings.Drive.INTAKE_FORWARD))
			{
				intakeDirection = IntakeDirection.FORWARD;
				intakeTumbler.setDestination(TumblerSystem.TumblerDestination.BUSY);
			}
			else if(driveInput.isPressed(Keybindings.Drive.INTAKE_REVERSE))
			{
				intakeDirection = IntakeDirection.REVERSE;
				intakeTumbler.setDestination(TumblerSystem.TumblerDestination.HOVER);
			}
			else
			{
				intakeDirection = IntakeDirection.STOP;
				intakeTumbler.setDestination(TumblerSystem.TumblerDestination.IDLE);
			}
			robot.intake.setIntakeDirection(intakeDirection);

			if(armInput.isPressed(Keybindings.Arm.PRIMARY_KEY))
			{
				if (scoring == false)
				{
					setTimeout(() -> {
						int nivel = (int) (0);
						extendo.setTargetPosition(nivel);
						extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
						extendo.setPower(1.0);
					}, 50);
					robot.scoreExtendo.extend(ExtendoServoSystem.ExtendoLevel.RETRACTED);
					setTimeout(() -> {
						robot.scoreClaw.open();
					}, 50);
					setTimeout(() -> {
						robot.scoreExtendo.extend(ExtendoServoSystem.ExtendoLevel.EXTENDED);
					}, 150);
					setTimeout(() -> {
						robot.scoreClaw.close();
					}, 250);
					setTimeout(() -> {
						robot.scoreExtendo.extend(ExtendoServoSystem.ExtendoLevel.RETRACTED);
					}, 600);
					setTimeout(() -> {
						liftLevel = 900;
						lift1.setTargetPosition(liftLevel);
						lift2.setTargetPosition(liftLevel);
					}, 700);
					setTimeout(() -> {
						robot.scoreTumbler.setDestination(TumblerSystem.TumblerDestination.HOVER);
						scoring = true;
					}, 900);
				}

				else if(scoring == true){
					setTimeout(() -> {
					setTimeout(() -> {
						robot.scoreClaw.open();
					}, 50);
					setTimeout(() -> {
						robot.scoreTumbler.setDestination(TumblerSystem.TumblerDestination.IDLE);
					}, 300);
					setTimeout(() -> {
						liftLevel = 0;
						lift1.setTargetPosition(liftLevel);
						lift2.setTargetPosition(liftLevel);
						scoring = false;
					}, 500);
					},100);
				}
			}

			if(driveInput.isPressed(Keybindings.Drive.MODE)){
				if(mode_val == false) {
					setTimeout(() -> {
						robot.scoreTumbler.setDestination(TumblerSystem.TumblerDestination.HOLD);
						mode_val = true;
					},200);
				}
				else{
					setTimeout(() -> {
						robot.scoreTumbler.setDestination(TumblerSystem.TumblerDestination.IDLE);
						mode_val = false;
					},200);
				}
			}

			if(driveInput.isPressed(Keybindings.Drive.SPECIMEN)){
				if(state_specimen == false) {
					setTimeout(() -> {
						robot.scoreClaw.close();
						setTimeout(() ->{
							liftLevel = 400;
							lift1.setTargetPosition(liftLevel);
							lift2.setTargetPosition(liftLevel);
						}, 150);
						setTimeout(() -> {
							robot.scoreTumbler.setDestination(TumblerSystem.TumblerDestination.BUSY);
						},500);
						setTimeout(() -> {
							robot.scoreExtendo.extend(ExtendoServoSystem.ExtendoLevel.EXTENDED);
						},600);
						state_specimen = true;
					},200);
				}
				else{
					setTimeout(() -> {
						robot.scoreClaw.open();
						setTimeout(() -> {
							robot.scoreExtendo.extend(ExtendoServoSystem.ExtendoLevel.RETRACTED);
						}, 150);
						setTimeout(() ->{
							liftLevel = -5;
							lift1.setTargetPosition(liftLevel);
							lift2.setTargetPosition(liftLevel);
						}, 350);
						setTimeout(() -> {
							robot.scoreTumbler.setDestination(TumblerSystem.TumblerDestination.HOLD);
							mode_val = true;
						},450);
						state_specimen = false;
					},250);
				}

			}

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
				liftLevel = 400;
				robot.scoreTumbler.setDestination(TumblerSystem.TumblerDestination.HOVER);
				lift_change_in_status = true;
				liftLevel2 = 0;
				//scoreTumbler.setPosition(0.84);
				//scoreSmallTumbler.setPosition(0.94);
			}
			else if (armInput.wasPressedThisFrame(Keybindings.Arm.BASKET_HIGH_KEY))
			{
				liftLevel = 900;
				lift_change_in_status = true;
				robot.scoreTumbler.setDestination(TumblerSystem.TumblerDestination.HOVER);
				liftLevel2 = 900;
				//scoreTumbler.setPosition(0.84);
				//scoreSmallTumbler.setPosition(0.94);
			}

			if(lift_change_in_status == true)
			{
				lift1.setTargetPosition(liftLevel);
				lift2.setTargetPosition(liftLevel);
				lift_change_in_status = false;
			}

			//SUSPEND
			if (lift1.isBusy()) lift1.setPower(1);
			else lift1.setPower(0);
			if (lift2.isBusy()) lift2.setPower(1);
			else lift2.setPower(0);
			if(extendo.isBusy()) extendo.setPower(1);
			else extendo.setPower(0);
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
					liftLevel = 400;
					liftLevel2 = 400;
					lift1.setTargetPosition(liftLevel);
					lift2.setTargetPosition(liftLevel);
				}
				else
				{
					//PICIORUSE SI MODIFICAT DELAYURI PT OPTIMIZARE
					suspending = true;
					liftLevel = 0;
					liftLevel2 = 0;
					lift1.setTargetPosition(liftLevel);
					lift2.setTargetPosition(liftLevel);
					axle.setPosition(0.59f);
						drivetrain.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
						drivetrain.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
						drivetrain.leftBack.setPower(1f);
						drivetrain.rightBack.setPower(1f);
					setTimeout(() -> {
						liftLevel = 750;
						liftLevel2 = 750;
						lift1.setTargetPosition(liftLevel);
						lift2.setTargetPosition(liftLevel);
					setTimeout(() -> {
						liftLevel = 0;
						liftLevel2 = 0;
						lift1.setTargetPosition(liftLevel);
						lift2.setTargetPosition(liftLevel);
					}, 5000);
					}, 5000);


				}
			}






		}
	}

	private static void setTimeout(Runnable runnable, long delay)
	{
		FanOpv2.runnable = runnable;
		FanOpv2.delay = delay;
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