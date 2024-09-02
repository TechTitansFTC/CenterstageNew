package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Bot extends RobotDrive{
    double currspdmul = 0;
    double intkpwr = 0.0;
    public DcMotor intake;
    public DcMotorEx rr;
    public DcMotorEx rl;
    public Servo servoTransfer;
    public Servo servoHL;
    public Servo servoHR;
    public Servo servoSLB;
    public Servo servoSLT;
    public Servo servoSRT;
    public Servo servoSRB;
    public Servo servoFlight;
    public Servo servoRR;
    public Servo servoRL;
    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;

    public void init(HardwareMap hardwareMap) {
//        fl = hardwareMap.dcMotor.get("fl"); //port 3 EH - RED (In RobotDrive)
//        bl = hardwareMap.dcMotor.get("bl"); //port 1 CH - YELLOW (In RobotDrive)
//        br = hardwareMap.dcMotor.get("br"); //port 0 CH - ORANGE (In RobotDrive)
//        fr = hardwareMap.dcMotor.get("fr"); //port 0 EH - BLUE (In RobotDrive)
        intake = (DcMotor) hardwareMap.dcMotor.get("intake"); //port 1 EH - PURPLE
        rr = (DcMotorEx) hardwareMap.dcMotor.get("rr"); // rigging right, port 3 CH - DARK GREEN
        rl = (DcMotorEx) hardwareMap.dcMotor.get("rl"); // rigging left, port 2 CH - LIGHT GREEN


        servoTransfer = hardwareMap.servo.get("transfer"); // port 3 EH, Transfer
        servoHL = hardwareMap.servo.get("hl"); // port 0 CH, Hook Left
        servoHR = hardwareMap.servo.get("hr"); // port 5 CH, Right Hook
        servoSLB = hardwareMap.servo.get("slb"); // port 2 CH, Slides Left Bottom
        servoSLT = hardwareMap.servo.get("slt"); // port 1 CH, Slides Left Top
        servoSRT = hardwareMap.servo.get("srt"); // port 4 CH, Slides Right Top
        servoSRB = hardwareMap.servo.get("srb"); // port 3 CH, Slides Right Bottom
        servoFlight = hardwareMap.servo.get("flight"); //port 1 EH, Flight
        servoRR = hardwareMap.servo.get("rigr"); // port 5 EH, Rigging Right
        servoRL = hardwareMap.servo.get("rigl"); //port 0 EH, Rigging Left


        //add servos and sensors to be used in all autonmous and teleop classes
        //add all the servo positions and stuff
        super.init(hardwareMap); //runs the robot drive part
    }

    public void TransferToggle() {
        if (servoTransfer.getPosition() < 0.11) {
            servoTransfer.setPosition(0.5);
        } else {
            servoTransfer.setPosition(0.1);
        }

    }

    public void initAprilTag(HardwareMap hardwareMap) {
        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
    }

    public String format(AprilTagDetection detection) {
        double range = detection.ftcPose.range;
        double bearing = detection.ftcPose.bearing;
        double yaw = detection.ftcPose.yaw;
        double tagx = detection.metadata.fieldPosition.get(0);
        double tagy = detection.metadata.fieldPosition.get(1);
        double theta = Math.toRadians(getHeading() + bearing);
        double fx = tagx - Math.cos(theta) * range;
        double fy = tagy - Math.sin(theta) * range;
        return String.format("id=%d R=%.2f B=%.2f Y=%.2f\n   fx=%.2f fy=%.2f",
                detection.id, range, bearing, yaw, fx, fy );
    }

    public boolean setManualExposure(LinearOpMode op,
                                     int exposureMS,
                                     int gain) {
        if (visionPortal == null) { return false; }
        while (!op.isStopRequested()
                && (visionPortal.getCameraState()
                != VisionPortal.CameraState.STREAMING)) {
            op.sleep(20);
        }
        if (!op.isStopRequested()) {
            ExposureControl exposureControl =
                    visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                op.sleep(50);
            }
            exposureControl.setExposure((long)exposureMS,
                    TimeUnit.MILLISECONDS);
            op.sleep(20);
            GainControl gainControl =
                    visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            op.sleep(20);
            return (true);
        }
        return (false);
    }

    //rigging
    public void rig() {
        // up: 0.7
        // down:
        double position = 0.0;
        if (position != 0.7) {
            position = 0.7;
        } else {
            position = 0.0;
        }

        servoHL.setPosition(position);
        servoHR.setPosition(position);
    }

    //intakeonoff


    //lefttrigger for power changing
    public double setspeed(double LTstats, double ogpwr){
        if (LTstats > 0.5){
            currspdmul = 1;
            return ogpwr;
        }
        else if (LTstats < 0.5 && LTstats > 0.0) {
            currspdmul =  0.6;
            return ogpwr * currspdmul;
        }
        else{
            currspdmul = 0.8;
            return ogpwr * currspdmul;
        }
    }


}
