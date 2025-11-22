package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

@Autonomous(name="Auto", group="Linear OpMode")
public abstract class autonanouse extends LinearOpMode {
private DcMotor leftRearDrive = null;
private DcMotor rightRearDrive = null;
private DcMotor leftFrontDrive = null;
private DcMotor rightFrontDrive = null;
private DcMotor Launcher = null;
private DcMotor Intake = null;
private Servo Lift = null;
private CRServo Loader = null;

private GoBildaPinpointDriver pinPoint = null;
public void runOpMode() {
    leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
    rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
    leftRearDrive = hardwareMap.get(DcMotor.class, "leftRearDrive");
    rightRearDrive = hardwareMap.get(DcMotor.class, "rightRearDrive");
    Launcher = hardwareMap.get(DcMotor.class, "launcher");
    Intake = hardwareMap.get(DcMotor.class, "intake");
    Lift = hardwareMap.get(Servo.class, "lift");
    Loader = hardwareMap.get(CRServo.class, "loader");
    pinPoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinPoint");

    leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
    leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
    rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
    rightRearDrive.setDirection(DcMotor.Direction.FORWARD);

    leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    double leftFrontPower = 0;
    double leftRearPower = 0;
    double rightFrontPower = 0;
    double rightRearPower = 0;
    waitForStart();
    while (opModeIsActive()) {

    }
}
       }
