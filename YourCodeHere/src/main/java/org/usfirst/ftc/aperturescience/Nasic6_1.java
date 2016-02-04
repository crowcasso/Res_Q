package org.usfirst.ftc.aperturescience;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.swerverobotics.library.interfaces.TeleOp;

/**
 * Nasic 6.0 - TeleOp
 *
 * @author FTC 5064 Aperture Science
 */
//@TeleOp
public class Nasic6_1 extends OpMode {

    private DcMotor motorR;
    private DcMotor motorL;
    private DcMotor sweeper;
    private DcMotor arm;
    private Servo wrist;
    private TouchSensor limit;
    private boolean prevButPress = false;
    private double amount = .1;
    double wristPosition = .5;


    @Override
    public void init() {
        motorR = hardwareMap.dcMotor.get("motorR");
        motorL = hardwareMap.dcMotor.get("motorL");
        arm = hardwareMap.dcMotor.get("elbow");
        wrist = hardwareMap.servo.get("wrist");
        motorL.setDirection(DcMotor.Direction.REVERSE);
        limit = hardwareMap.touchSensor.get("limit");
        motorL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        arm.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        sweeper = hardwareMap.dcMotor.get("sweeper");
        arm.setDirection(DcMotor.Direction.REVERSE);

        wristPosition = 0.2;
    }

    @Override
    public void loop() {

        // driving
        float throttle = -gamepad1.left_stick_y;
        float direction = gamepad1.right_stick_x;
        float right = throttle - direction;
        float left = throttle + direction;

        double speedMult = .5;
        if (gamepad1.right_bumper) speedMult = 1;

        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        right = (float)scaleInput(right);
        left =  (float)scaleInput(left);

        motorR.setPower(right * speedMult);
        motorL.setPower(left * speedMult);


        //wrist
        if (gamepad1.x) {
            amount = .1;
        } else if (gamepad1.b) {
            amount = .01;
        }

        if (gamepad1.a) {
            if (!prevButPress) {
                if(wristPosition - amount> 0) {
                    wristPosition -= amount;
                }
            }
            prevButPress = true;

        } else if (gamepad1.y) {
            if (!prevButPress) {
                if(wristPosition + amount < 1) {
                    wristPosition += amount;
                }
            }
            prevButPress = true;

        } else {
            prevButPress = false;
        }

        wrist.setPosition(wristPosition);


        //arm
        if (gamepad1.left_bumper){
            arm.setPower(0.25);
        }
        else if (gamepad1.left_trigger > 0.3f && limit.isPressed() == false){ //------------------------------------------------------------------
            arm.setPower(-0.25);
        }
        else{
            arm.setPower(0.0);
        }

        /*if (gamepad1.right_bumper){
            if (wristPosition == 0.4){
                wristPosition = 0;
            }
            //while (wristPosition != 0){}
            arm.setPower(-.5);
        }
        else if (gamepad1.left_bumper){
            if (wristPosition == 0.0){
                wristPosition = 1.0;
            }
            while (wristPosition != 1.0){}
            arm.setPower(.5);
        }*/



        /*
        if(gamepad2.right_bumper == true)   speedMult = 1;

        if(right > -.1 && right < .1 && left > -.1 && left < .1) {
            throttle = -gamepad1.left_stick_y;
            direction = -gamepad1.right_stick_x;
            right = throttle + direction;
            left = throttle - direction;
            if (gamepad1.right_bumper == true) speedMult = 1;
            else speedMult = .5;
        }*/


        // sweeper
        if (gamepad1.dpad_down) {
            sweeper.setPower(-0.5);
        }
        if (gamepad1.dpad_left || gamepad1.dpad_right){
            sweeper.setPower(0);
        }
        if (gamepad1.dpad_up) {
            sweeper.setPower(0.5);
        }


        //telemetry
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("left tgt pwr", "left  pwr: "
                + String.format("%.2f", left));
        telemetry.addData("right tgt pwr", "right pwr: "
                + String.format("%.2f", right));
        telemetry.addData("Wrist Position: ",wristPosition);
        telemetry.addData("Arm Encoder: ",arm.getCurrentPosition());
        telemetry.addData("Touch Sensor: ", limit.getValue());
    }

    double scaleInput(double dVal)  {
        double[] scaleArray = {
                0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00
        };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * (scaleArray.length - 1));
        if (index < 0) {
            index = -index;
        } else if (index > (scaleArray.length - 1)) {
            index = scaleArray.length - 1;
        }

        double dScale;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }
}
