package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "RRBotAutonomous")
public class RRBotAutonomous extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    private static final String VUFORIA_KEY = "INSERT KEY HERE";
    
    private VuforiaLocalizer vuforia; // Local instance of Vuforia

    private TFObjectDetector tfod; // Local instance of TensorFlow Object

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();

        // Activate TensorFlow Object Detection
        if (tfod != null) {
            tfod.activate();

            tfod.setZoom(2, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData("Status", "Robot initialized, locating object");
        telemetry.update();

        int objectPos = getObjectPos();

        telemetry.addData("Status", "Robot initialized, object located");
        telemetry.update();

        waitForStart();



        if (opModeIsActive()) {
            while (opModeIsActive()){
                objectPos = getObjectPos();

                telemetry.addData("Status", "Robot running, object located");
                telemetry.update();
            }
        }
    }

    public int getObjectPos() {
        runtime.reset();

        int objectY = 0;
        int objectPos = 0;

        while (runtime.seconds() < 5) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // Get y position of recognized object with highest confidence
                    boolean objectDetected = false;
                    if (updatedRecognitions.size() == 1) {
                        objectY = (int) updatedRecognitions.get(0).getLeft();

                        objectDetected = true;

                        telemetry.addData("Object Detection", objectY);
                    } else if (updatedRecognitions.size() == 0) {
                        telemetry.addData("Object Detection", "No object detected");
                    } else {
                        telemetry.addData("Object Detection", "More than one object detected");

                        objectDetected = true;

                        int object = 0;
                        float highestConfidence = 0;
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getConfidence() > highestConfidence) {
                                object = i;
                                highestConfidence = recognition.getConfidence();
                            }
                            i++;
                        }

                        objectY = (int) updatedRecognitions.get(object).getLeft();
                        telemetry.addData("Object Detection", objectY);
                    }
                    
                    if (!objectDetected) {
                        objectPos = 0;
                        telemetry.addData("Target Location", "[\uD83D\uDC24][   ][   ]");
                    } else if (objectDetected && objectY > 600) {
                        objectPos = 1;
                        telemetry.addData("Target Location", "[   ][\uD83D\uDC24][   ]");
                    } else if (objectDetected) {
                        objectPos = 2;
                        telemetry.addData("Target Location", "[   ][   ][\uD83D\uDC24]");
                    } else {
                        telemetry.addData("Target Location", "[   ][   ][   ]");
                    }

                    telemetry.update();

                }
            }
        }
        return objectPos;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
