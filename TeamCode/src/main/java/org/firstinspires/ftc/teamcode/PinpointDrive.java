package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.zyxOrientation;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

@Config
public class PinpointDrive extends MecanumDrive {
	public static class Params {
		public String pinpointDeviceName = "odo";
		public double xOffset = -3.3071;
		public double yOffset = -6.6142;
		public double encoderResolution = GoBildaPinpointDriver.goBILDA_4_BAR_POD;
		public GoBildaPinpointDriver.EncoderDirection xDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
		public GoBildaPinpointDriver.EncoderDirection yDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
		public boolean usePinpointIMUForTuning = true;
	}

	public static Params PARAMS = new Params();
	public GoBildaPinpointDriver pinpoint;
	private Pose2d lastPinpointPose = pose;

	public PinpointDrive(HardwareMap hardwareMap, Pose2d pose) {
		super(hardwareMap, pose);
		FlightRecorder.write("PINPOINT_PARAMS", PARAMS);
		pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PARAMS.pinpointDeviceName);

		if (PARAMS.usePinpointIMUForTuning) {
			lazyImu = new LazyImu(hardwareMap, PARAMS.pinpointDeviceName,
					new RevHubOrientationOnRobot(zyxOrientation(0, 0, 0)));
		}

		pinpoint.setOffsets(DistanceUnit.MM.fromInches(PARAMS.xOffset),
				DistanceUnit.MM.fromInches(PARAMS.yOffset));
		pinpoint.setEncoderResolution(PARAMS.encoderResolution);
		pinpoint.setEncoderDirections(PARAMS.xDirection, PARAMS.yDirection);

		pinpoint.resetPosAndIMU();
		try {
			Thread.sleep(300);
		} catch (InterruptedException e) {
			throw new RuntimeException(e);
		}

		Pose2D ftcPose = new Pose2D(
				DistanceUnit.MM,
				pose.position.x,
				pose.position.y,
				AngleUnit.RADIANS,
				pose.heading.toDouble()
		);
		pinpoint.setPosition(ftcPose);
	}

	@Override
	public PoseVelocity2d updatePoseEstimate() {
		// Update position if it was changed externally
		if (lastPinpointPose != pose) {
			Pose2D ftcPose = new Pose2D(
					DistanceUnit.MM,
					pose.position.x,
					pose.position.y,
					AngleUnit.RADIANS,
					pose.heading.toDouble()
			);
			pinpoint.setPosition(ftcPose);
		}

		// Get updated pose from pinpoint
		pinpoint.update();

		// Convert FTC Pose2D to Road Runner Pose2d
		Pose2D ftcPose = pinpoint.getPosition();
		pose = new Pose2d(
				ftcPose.getX(DistanceUnit.MM),
				ftcPose.getY(DistanceUnit.MM),
				ftcPose.getHeading(AngleUnit.RADIANS)
		);
		lastPinpointPose = pose;

		// Maintain pose history
		poseHistory.add(pose);
		while (poseHistory.size() > 100) {
			poseHistory.removeFirst();
		}

		// Log data
		FlightRecorder.write("ESTIMATED_POSE", new PoseMessage(pose));
		FlightRecorder.write("PINPOINT_RAW_POSE", new FTCPoseMessage(pinpoint.getPosition()));
		FlightRecorder.write("PINPOINT_STATUS", pinpoint.getDeviceStatus());

		// Get and convert velocity
		Pose2D ftcVelocity = pinpoint.getVelocity();
		return new PoseVelocity2d(
				new Vector2d(ftcVelocity.getX(DistanceUnit.MM),
						ftcVelocity.getY(DistanceUnit.MM)),
				ftcVelocity.getHeading(AngleUnit.RADIANS)
		);
	}

	// Debug logging class
	public static final class FTCPoseMessage {
		public long timestamp;
		public double x;
		public double y;
		public double heading;

		public FTCPoseMessage(Pose2D pose) {
			this.timestamp = System.nanoTime();
			this.x = pose.getX(DistanceUnit.INCH);
			this.y = pose.getY(DistanceUnit.INCH);
			this.heading = pose.getHeading(AngleUnit.RADIANS);
		}
	}
}
