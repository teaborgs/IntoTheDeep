package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.systems.DistanceSystem;
import org.firstinspires.ftc.teamcode.systems.ExtendoServoSystem;
import org.firstinspires.ftc.teamcode.systems.LiftSystem;
import org.firstinspires.ftc.teamcode.systems.OpenCloseSystem;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;
import org.firstinspires.ftc.teamcode.systems.ExtendoMotorSystem;
import org.firstinspires.ftc.teamcode.systems.IntakeSystem;

public final class RobotHardwareNEW {
	public final MecanumDrive drivetrain;
	public TumblerSystem intakeTumbler, scoreTumbler;
	public ExtendoMotorSystem extendo;
	public ExtendoServoSystem scoreExtendo;
	public LiftSystem lift;
	public OpenCloseSystem scoreClaw, axle;
	public DistanceSystem distanceSystem;
	public IntakeSystem intake;

	public RobotHardwareNEW(HardwareMap hardwareMap) {
		drivetrain = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

		intake = new IntakeSystem(hardwareMap.get(DcMotorEx.class, "intake"));
		extendo = new ExtendoMotorSystem(hardwareMap.get(DcMotorEx.class, "extendo"));
		lift = new LiftSystem(hardwareMap.get(DcMotorEx.class, "lift1"), hardwareMap.get(DcMotorEx.class, "lift2"));
		scoreClaw = new OpenCloseSystem(hardwareMap.get(Servo.class, "scoreClaw"), 0.55f, 0.3f);
		intakeTumbler = new TumblerSystem(hardwareMap.get(Servo.class, "intakeTumbler"), 0.11f, 0.47f, 0.15f, 0.5f, 0.65f);
		scoreTumbler = new TumblerSystem(hardwareMap.get(Servo.class, "scoreTumbler"), 0.01f, 0.72f, 0.5f, 0.94f, -0.5f);
		scoreExtendo = new ExtendoServoSystem(hardwareMap.get(Servo.class, "scoreExtendo"));

		// Uncomment and edit these as needed
		/*
		intakeTumbler = new TumblerSystem(hardwareMap.get(Servo.class, "intakeTumbler"), 0.14f, 0.14f, 0.14f, 0.14f, 0.23f);
		scoreTumbler = new TumblerSystem(hardwareMap.get(Servo.class, "scoreTumbler"), 0.99f, 0.3f, 0.5f, 0.32f, 0.41f);
		lift = new LiftSystem(hardwareMap.get(DcMotorEx.c	lass, "lift1"), hardwareMap.get(DcMotorEx.class, "lift2"));
		axle = new OpenCloseSystem(hardwareMap.get(Servo.class, "axleServo1"), 0.5f, 0.39f);
		distanceSystem = new DistanceSystem(hardwareMap.get(Rev2mDistanceSensor.class, "leftDist"),
											hardwareMap.get(Rev2mDistanceSensor.class, "rightDist"));
		*/
	}

	public void init() {
		// ✅ Initialize intake system
		intake.init();
		extendo.init();
		lift.init();
		intakeTumbler.init();
		intakeTumbler.setDestination(TumblerSystem.TumblerDestination.IDLE);
		scoreClaw.init();
		scoreTumbler.init();
		scoreExtendo.init();

		// Uncomment and edit these as needed
		/*
		intakeTumbler.init();
		scoreTumbler.setDestination(TumblerSystem.TumblerDestination.TRANSFER);
		lift.init();
		scoreTumbler.init();
		scoreClaw.init();
		scoreTumbler.setDestination(TumblerSystem.TumblerDestination.TRANSFER);
		scoreClaw.close();
		axle.init();
		*/
	}
}
