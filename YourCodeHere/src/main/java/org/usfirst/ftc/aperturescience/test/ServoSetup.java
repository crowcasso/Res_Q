package org.usfirst.ftc.aperturescience.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.swerverobotics.library.ClassFactory;
import org.swerverobotics.library.interfaces.Disabled;
import org.swerverobotics.library.interfaces.TeleOp;
import org.swerverobotics.library.internal.AdaFruitTCS34725ColorSensor;

/**
 * Servo Setup -- for setting up Servos
 *
 * @author FTC 5064 Aperture Science
 */
@TeleOp
//@Disabled
public class ServoSetup extends OpMode {

    // Hardware
    /*
    private DcMotor turntable;
    private DcMotor motorR;
    private DcMotor motorL;
    private DcMotor sweeper;
    */
    private DcMotor arm;
    private Servo leftShield;
    private Servo rightShield;
    private Servo backShield;
    private Servo wrist;
    private Servo servo;
    private Servo redArm;
    private Servo leftTape;
    private Servo rightTape;
    private Servo tapeLock;
    private Servo blueArm;
    private Servo armExt;

    private AdaFruitTCS34725ColorSensor colorSensor;

    boolean pressed = false;
    double position = .5;
    private final double TRIGGER_THRESHOLD = 0.3;

    @Override
    public void init() {

        // Hardware Map
        arm = hardwareMap.dcMotor.get("arm");

        /*
        motorR = hardwareMap.dcMotor.get("motorR");
        motorL = hardwareMap.dcMotor.get("motorL");
        turntable = hardwareMap.dcMotor.get("turntable");
        sweeper = hardwareMap.dcMotor.get("sweeper");
        */

        leftShield = hardwareMap.servo.get("leftShield");
        rightShield = hardwareMap.servo.get("rightShield");
        backShield = hardwareMap.servo.get("backShield");
        wrist = hardwareMap.servo.get("wrist");
        redArm = hardwareMap.servo.get("redArm");
        leftTape = hardwareMap.servo.get("leftTape");
        rightTape = hardwareMap.servo.get("rightTape");
        tapeLock = hardwareMap.servo.get("tapeLock");
        blueArm = hardwareMap.servo.get("blueArm");
        armExt = hardwareMap.servo.get("armExt");


        leftShield.setPosition(position);
        wrist.setPosition(position);
        rightShield.setPosition(position);
        backShield.setPosition(.84); //position);
        redArm.setPosition(position);
        leftTape.setPosition(position);
        rightTape.setPosition(position);
        tapeLock.setPosition(position);
        blueArm.setPosition(position);
        armExt.setPosition(position);

        servo = armExt;

        arm.setPower(0);

        //colorSensor = ClassFactory.


        // Drive Base Setup
        /*
        motorL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        arm.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        turntable.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        motorL.setDirection(DcMotor.Direction.REVERSE);

        sweeper.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        sweeper.setDirection(DcMotor.Direction.REVERSE);
        */

    }

    @Override
    public void loop() {
        if (!gamepad1.a && !gamepad1.b && !gamepad1.x && !gamepad1.y) {
            pressed = false;
        }

            if (gamepad1.a && pressed == false) {
                position += .05;
                pressed = true;
            }
            if (gamepad1.b && pressed == false) {
                position -= .05;
                pressed = true;
            }
            if (gamepad1.x && pressed == false) {
                position += .01;
                pressed = true;
            }
            if (gamepad1.y && pressed == false) {
                position -= .01;
                pressed = true;
            }

        position = Range.clip(position, 0, 1);
        servo.setPosition(position);
        telemetry.addData("Servo Position:",position);


        if (gamepad1.right_trigger > TRIGGER_THRESHOLD){
            arm.setPower(-0.1);
        }
        else if (gamepad1.right_bumper){
            arm.setPower(0.1);
        }
        else {
            arm.setPower(0);
        }
        telemetry.addData("Trigger position:",gamepad1.right_trigger);
    }

}