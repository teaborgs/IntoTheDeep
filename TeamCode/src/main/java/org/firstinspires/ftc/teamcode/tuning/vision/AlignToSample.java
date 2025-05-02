package org.firstinspires.ftc.teamcode.tuning.vision;

import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Align To Sample Test", group = "Vision")
public class AlignToSample extends LinearOpMode {
	private RobotHardware robot;
	private MecanumDrive drivetrain;
	Utilities.Alliance alliance;

	ColorBlobLocatorProcessor processor;
	VisionPortal portal;

	Servo servo;
	double servoPos = 0.18;

	@Override
	public void runOpMode() throws InterruptedException {
		Init();
		waitForStart();
		Run();
	}

	private void Init() {
		telemetry.setMsTransmissionInterval(50);
		telemetry.update();
		telemetry.clear();
		robot = new RobotHardware(hardwareMap);
		robot.init();
		// Initialize the custom pipeline
		processor = new GreyToBlackPipeline();

		portal = new VisionPortal.Builder()
				.addProcessor(processor)
				.setCameraResolution(new Size(640, 480))
				.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
				.setLiveViewContainerId(hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()))
				.enableLiveView(true) // Enable live view
				.build();

		while (portal.getCameraState() != VisionPortal.CameraState.STREAMING)
			sleep(10);

		portal.getCameraControl(GainControl.class).setGain(255);
		portal.getCameraControl(ExposureControl.class).setExposure(20, TimeUnit.MILLISECONDS);
		servo = hardwareMap.get(Servo.class, "extendo");
		servo.setPosition(servoPos);
	}

	private void Run() {
		// Continuously process frames until the op mode is stopped
		while (!isStopRequested()) {
			// After processing, we can get the distance and adjust the servo
			double distanceToCenterCm = ((GreyToBlackPipeline) processor).getDistanceToCenterCm();
			telemetry.addData("Distance to Center (cm): ", distanceToCenterCm);
			telemetry.update();
			if (distanceToCenterCm == -10) {

			}
			// Sleep for a short duration to prevent overwhelming the CPU
			sleep(50); // Adjust the sleep time as needed
		}
	}

	// Custom pipeline
	public static class GreyToBlackPipeline extends ColorBlobLocatorProcessor {
		private double distanceToCenterCm = -1; // Default to -1 if no contours are found

		@Override
		public void init(int width, int height, CameraCalibration calibration) {
			// Initialize any resources if needed
		}

		@Override
		public Object processFrame(Mat frame, long captureTime) {
			// Convert the image to grayscale
			Mat grey = new Mat();
			Imgproc.cvtColor(frame, grey, Imgproc.COLOR_RGB2GRAY);

			// Define the range of grey values (adjust these values as needed)
			Scalar lowerGrey = new Scalar(20); // Lower bound for grey
			Scalar upperGrey = new Scalar(170); // Upper bound for grey

			// Create a mask for grey pixels
			Mat mask = new Mat();
			Core.inRange(grey, lowerGrey, upperGrey, mask);

			grey.setTo(new Scalar(0), mask);

			// Adjust brightness and contrast manually
			double alpha = 1.5; // Contrast control (1.0 means no change)
			double beta = 2;   // Brightness control (0 means no change)
			Mat adjusted = new Mat();
			Core.convertScaleAbs(grey, adjusted, alpha, beta);

			int distanceFromBottom = 350; // Distance from the bottom in pixels (adjust as needed)
			int bottomStartRow = adjusted.rows() - distanceFromBottom;
			if (bottomStartRow > 0) {
				Mat roi = new Mat(adjusted, new org.opencv.core.Rect(0, 0, adjusted.cols(), bottomStartRow));
				roi.setTo(new Scalar(0));
			}
			int distanceFromLeft = 280; // Distance from the left in pixels (adjust as needed)
			if (distanceFromLeft < adjusted.cols()) {
				Mat leftRoi = new Mat(adjusted, new org.opencv.core.Rect(0, 0, distanceFromLeft, adjusted.rows()));
				leftRoi.setTo(new Scalar(0));
			}
			int distanceFromRight = 285; // Distance from the right in pixels (adjust as needed)
			if (distanceFromRight < adjusted.cols()) {
				Mat rightRoi = new Mat(adjusted, new org.opencv.core.Rect(adjusted.cols() - distanceFromRight, 0, distanceFromRight, adjusted.rows()));
				rightRoi.setTo(new Scalar(0));
			}
			// Set pixels below a brightness threshold to black
			int brightnessThreshold = 110; // Brightness threshold (adjust as needed)
			Mat brightnessMask = new Mat();
			Core.inRange(adjusted, new Scalar(0), new Scalar(brightnessThreshold), brightnessMask);
			adjusted.setTo(new Scalar(0), brightnessMask);

			// Find contours of white pixel groups
			Mat binary = new Mat();
			Imgproc.threshold(adjusted, binary, brightnessThreshold, 255, Imgproc.THRESH_BINARY);
			List<MatOfPoint> contours = new ArrayList<>();
			Mat hierarchy = new Mat();
			Imgproc.findContours(binary, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

			contours.removeIf(contour -> Imgproc.contourArea(contour) < 2000);

			// Check if there are no white groups (contours)
			Mat output = null;
			if (contours.isEmpty()) {
				contours.clear(); // Clear the contours list
				distanceToCenterCm = -10; // Set distance to -10
			} else {
				// Find the closest contour to the bottom center of the image
				int imageCenterX = adjusted.cols() / 2; // X-coordinate of the bottom center
				int imageBottomY = adjusted.rows(); // Y-coordinate of the bottom center

				MatOfPoint closestContour = null;
				double minDistance = Double.MAX_VALUE;

				for (MatOfPoint contour : contours) {
					// Calculate the center of the contour
					org.opencv.core.Rect rect = Imgproc.boundingRect(contour);
					int centerX = rect.x + rect.width / 2;
					int centerY = rect.y + rect.height / 2;

					// Calculate the Euclidean distance to the bottom center
					double distance = Math.sqrt(Math.pow(centerX - imageCenterX, 2) + Math.pow(centerY - imageBottomY, 2));

					// Keep the contour with the smallest distance
					if (distance < minDistance) {
						minDistance = distance;
						closestContour = contour;
					}
				}

				// If a closest contour is found, process it
				if (closestContour != null) {
					// Create a mask for the closest contour
					Mat closestContourMask = new Mat(adjusted.size(), adjusted.type(), new Scalar(0));
					List<MatOfPoint> closestContourList = new ArrayList<>();
					closestContourList.add(closestContour);
					Imgproc.drawContours(closestContourMask, closestContourList, -1, new Scalar(255), -1);

					// Set all pixels not in the closest contour to black
					adjusted.setTo(new Scalar(0), closestContourMask);

					// Convert the adjusted grayscale image
					output = new Mat();
					Imgproc.cvtColor(adjusted, output, Imgproc.COLOR_GRAY2RGB);

					// Draw the closest contour on the output image
					org.opencv.core.Rect rect = Imgproc.boundingRect(closestContour);
					int centerX = rect.x + rect.width / 2;
					int centerY = rect.y + rect.height / 2;

					Imgproc.circle(output, new org.opencv.core.Point(centerX, centerY), 5, new Scalar(0, 0, 255), -1);

					String areaText = String.format("%.0f", Imgproc.contourArea(closestContour));
					Imgproc.putText(
							output,
							areaText,
							new org.opencv.core.Point(rect.x + rect.width / 4, rect.y + rect.height / 2),
							Imgproc.FONT_HERSHEY_SIMPLEX,
							0.5,
							new Scalar(0, 255, 0),
							1
					);

					String coordText = String.format("(%d, %d)", centerX, centerY);
					Imgproc.putText(
							output,
							coordText,
							new org.opencv.core.Point(rect.x + rect.width / 4, rect.y + rect.height / 2 + 20),
							Imgproc.FONT_HERSHEY_SIMPLEX,
							0.5,
							new Scalar(255, 0, 0),
							1
					);

					// Calculate the distance to the center of the closest contour
					int distanceInPixels = imageBottomY - centerY;
					double scaleFactor = 0.1; // Example: 0.1 cm per pixel
					distanceToCenterCm = distanceInPixels * scaleFactor;
				} else {
					// If no closest contour is found, set the distance to -10
					distanceToCenterCm = -10;
				}
			}

			if (output != null) {
				output.copyTo(frame);
			}
			return frame;
		}

		public double getDistanceToCenterCm() {
			return distanceToCenterCm;
		}

		@Override
		public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
			// Draw on the canvas if needed
		}

		@Override
		public void addFilter(BlobFilter filter) {
			// Implement this method if you need to add custom filters
		}

		@Override
		public void removeFilter(BlobFilter filter) {
			// Implement this method if you need to remove custom filters
		}

		@Override
		public void removeAllFilters() {
			// Implement this method if you need to remove all filters
		}

		@Override
		public void setSort(BlobSort sort) {
			// Implement this method if you need to set a sorting method for blobs
		}

		@Override
		public List<Blob> getBlobs() {
			// Return the list of detected blobs
			return null; // Replace with actual blob list if needed
		}
	}
}