package org.firstinspires.ftc.teamcode.pedroPathing.circuitBreaker;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor; // Added for motors
import com.qualcomm.robotcore.hardware.Servo;   // Added for servos

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "manualPedroPathExample", group = "TeleOp")
public class manualPedroPathExample extends OpMode {

    // Pedro Pathing
    private Follower follower;
    private Path autoPath;
    private boolean isAutomated = false;

    // --- 1. DECLARE MECHANISM HARDWARE ---
    private DcMotor climberMotor;
    private Servo clawServo;

    // Servo position constants
    private final double CLAW_OPEN_POS = 0.3;
    private final double CLAW_CLOSED_POS = 0.8;

    @Override
    public void init() {
        // Pedro Setup
        follower = Constants.createFollower(hardwareMap);
        Pose startPose = new Pose(0, 0, 0);
        follower.setStartingPose(startPose);

        autoPath = new Path(new BezierLine(startPose, new Pose(24, 24, 0)));
        autoPath.setConstantHeadingInterpolation(0);

        // --- 2. INITIALIZE MECHANISMS FROM HARDWARE MAP ---
        // "climber" and "claw" MUST match the names in your Driver Station Robot Configuration
        climberMotor = hardwareMap.get(DcMotor.class, "climber");
        clawServo = hardwareMap.get(Servo.class, "claw");

        // Optional Motor Settings:
        // Brake mode prevents the climber/slide from slipping when power is 0
        climberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initial Servo Position
        clawServo.setPosition(CLAW_CLOSED_POS);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        // -------------------------------------------------------------
        // DRIVETRAIN (Pedro Pathing)
        // -------------------------------------------------------------
        if (!isAutomated) {
            double forward = -gamepad1.left_stick_y;
            double strafe  = -gamepad1.left_stick_x;
            double turn    = -gamepad1.right_stick_x;

            follower.setTeleOpDrive(forward, strafe, turn, true);
        }

        if (gamepad1.a && !isAutomated) {
            follower.followPath(autoPath);
            isAutomated = true;
        }

        if (isAutomated && (gamepad1.b || !follower.isBusy())) {
            follower.startTeleopDrive();
            isAutomated = false;
        }

        // -------------------------------------------------------------
        // 3. CONTROL ADDITIONAL MECHANISMS
        // -------------------------------------------------------------

        // --- CLIMBER / LIFT CONTROL (Using Triggers or Joysticks) ---
        // Right trigger raises climber, Left trigger lowers climber
        double climberPower = gamepad1.right_trigger - gamepad1.left_trigger;
        climberMotor.setPower(climberPower);

        // --- CLAW CONTROL (Using Bumpers) ---
        // Pressing Right Bumper opens, Left Bumper closes
        if (gamepad1.right_bumper) {
            clawServo.setPosition(CLAW_OPEN_POS);
        } else if (gamepad1.left_bumper) {
            clawServo.setPosition(CLAW_CLOSED_POS);
        }

        // Telemetry
        telemetry.addData("Climber Power", climberPower);
        telemetry.addData("Claw Position", clawServo.getPosition());
        telemetry.update();
    }
}