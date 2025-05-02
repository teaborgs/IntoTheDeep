package org.firstinspires.ftc.teamcode.tuning.vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Read Camera Stats", group = "Vision")
public class ReadCameraStats extends LinearOpMode
{
	VisionPortal portal;

	@Override
	public void runOpMode() throws InterruptedException
	{
		Init();
		waitForStart();
		while (!isStopRequested())
			sleep(100);
	}

	private void Init()
	{
		portal = new VisionPortal.Builder()
				.setCameraResolution(new Size(640, 480))
				.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
				.build();

		while (portal.getCameraState() != VisionPortal.CameraState.STREAMING)
			sleep(50);

		ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
		telemetry.addData("Min Exp:  ", (int)exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1);
		telemetry.addData("Max Exp:  ", (int)exposureControl.getMaxExposure(TimeUnit.MILLISECONDS));

		GainControl gainControl = portal.getCameraControl(GainControl.class);
		telemetry.addData("Min Gain: ", gainControl.getMinGain());
		telemetry.addData("Max Gain: ", gainControl.getMaxGain());

		telemetry.update();
	}
}
