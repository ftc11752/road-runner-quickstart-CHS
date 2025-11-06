package org.firstinspires.ftc.teamcode;

//public class Limelight3A2pipes {
//}

//package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.BarcodeResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.ClassifierResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.ColorResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "Limelight3A: Tags + Color Demo", group = "Vision")
public class Limelight3A2pipes extends OpMode {

    // TODO: set these to your actual pipeline indices from the Limelight web UI
    private static final int APRILTAG_PIPELINE = 0;
    private static final int COLOR_PIPELINE    = 1;

    private Limelight3A limelight;

    // simple toggle debounce
    private boolean aWasPressed = false;
    private int currentPipeline = APRILTAG_PIPELINE;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // Poll frequently; Limelight handles throttling internally
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(currentPipeline);
        limelight.start();

        telemetry.addLine("Limelight initialized. A=toggle pipeline (AprilTags <-> Color).");
        telemetry.addData("Pipeline", currentPipeline == APRILTAG_PIPELINE ? "AprilTags" : "Color");
    }

    @Override
    public void loop() {
        // Pipeline toggle with gamepad1.a
        boolean aNow = gamepad1.a;
        if (aNow && !aWasPressed) {
            currentPipeline = (currentPipeline == APRILTAG_PIPELINE) ? COLOR_PIPELINE : APRILTAG_PIPELINE;
            limelight.pipelineSwitch(currentPipeline);
        }
        aWasPressed = aNow;

        // Get latest frame’s results
        LLResult result = limelight.getLatestResult();

        telemetry.addData("Pipeline", currentPipeline == APRILTAG_PIPELINE ? "AprilTags" : "Color");

        if (result == null || !result.isValid()) {
            telemetry.addLine("No valid targets");
            telemetry.update();
            return;
        }

        // Base measurements (present for all pipelines): tx/ty/ta and freshness
        telemetry.addData("tx (deg)", result.getTx());
        telemetry.addData("ty (deg)", result.getTy());
        telemetry.addData("ta (%)",  result.getTa());
        telemetry.addData("staleness (ms)", result.getStaleness());

        if (currentPipeline == APRILTAG_PIPELINE) {
            // ---- APRILTAG / FIDUCIALS ----
            List<FiducialResult> fiducials = result.getFiducialResults();
            telemetry.addData("Tags Detected", fiducials.size());

            for (int i = 0; i < fiducials.size(); i++) {
                FiducialResult f = fiducials.get(i);
                int id = f.getFiducialId();               // AprilTag ID
                double xDeg = f.getTargetXDegrees();      // left/right offset
                double yDeg = f.getTargetYDegrees();      // up/down offset
                double area = f.getTargetArea();          // 0..100

                telemetry.addData(
                        String.format("Tag[%d] id", i), id
                );
                telemetry.addData(
                        String.format("Tag[%d] tx/ty/area", i),
                        String.format("%.2f / %.2f / %.2f", xDeg, yDeg, area)
                );

                // Optional: 3D pose relative to tag (most useful for motion)
                Pose3D robotPoseTS = f.getRobotPoseTargetSpace(); // (x,y,z, roll,pitch,yaw)
                if (robotPoseTS != null) {
                    telemetry.addData(
                            String.format("Tag[%d] robotY (m, target-space)", i),
                            String.format("%.3f", robotPoseTS.getPosition().y)
                    );
                }
            }

            // Optional: field-relative fused pose (MegaTag) if enabled in pipeline
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                telemetry.addData("botpose (x,y m)",
                        String.format("%.3f, %.3f", botpose.getPosition().x, botpose.getPosition().y));
            }

        } else {
            // ---- COLOR / RETRO PIPELINES ----
            List<ColorResult> colors = result.getColorResults();
            telemetry.addData("Color Targets", colors.size());

            for (int i = 0; i < colors.size(); i++) {
                ColorResult c = colors.get(i);
                double xDeg = c.getTargetXDegrees();
                double yDeg = c.getTargetYDegrees();
                double area = c.getTargetArea(); // 0..100

                telemetry.addData(
                        String.format("Color[%d] tx/ty/area", i),
                        String.format("%.2f / %.2f / %.2f", xDeg, yDeg, area)
                );
            }
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}
