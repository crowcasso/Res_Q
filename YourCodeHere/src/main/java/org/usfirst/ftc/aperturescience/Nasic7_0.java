package org.usfirst.ftc.aperturescience;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.swerverobotics.library.interfaces.TeleOp;

/**
 * Nasic 6.0 - TeleOp
 *
 * @author FTC 5064 Aperture Science
 */
//@TeleOp
public class Nasic7_0 extends OpMode {

    // Hardware
    private DcMotor motorR;
    private DcMotor motorL;

    @Override
    public void init() {

        // Hardware Map
        motorR = hardwareMap.dcMotor.get("motorR");
        motorL = hardwareMap.dcMotor.get("motorL");

        // Drive Base Setup
        motorL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorL.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        driveControl();         // drive the robot (or not)
    }



    /**
     ******************************
     *          DRIVING           *
     ******************************
     **/


    private final double NORMAL_DRIVE_SPEED = 0.5;
    private final double LOW_DRIVE_SPEED = 0.3;
    private final double HIGH_DRIVE_SPEED = 1.0;
    private final double TRIGGER_THRESHOLD = 0.2;

    /**
     * Drive the robot!
     */
    private void driveControl() {
        float throttle = gamepad1.left_stick_y;
        float direction = -gamepad1.right_stick_x;

        double speedMult = NORMAL_DRIVE_SPEED;

        if (gamepad1.right_bumper) {
            speedMult = LOW_DRIVE_SPEED;
        }
        else if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            speedMult = HIGH_DRIVE_SPEED;
        }

        /*
        if (throttle < STICK_THRESHOLD && throttle > -STICK_THRESHOLD
                && direction < STICK_THRESHOLD && direction > -STICK_THRESHOLD) {
            throttle = -gamepad2.left_stick_y;
            direction = gamepad2.right_stick_x;
            if (gamepad2.right_bumper) {
                speedMult = LOW_DRIVE_SPEED;
                watcher = 3;
            }
            else if (gamepad2.right_trigger > TRIGGER_THRESHOLD) {
                speedMult = HIGH_DRIVE_SPEED;
                watcher = 4;
            }
        }*/

        // computer motor power
        float right = throttle - direction;
        float left = throttle + direction;

        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        right = (float)scaleInput(right);
        left =  (float)scaleInput(left);

        // set the motors
        motorR.setPower(right * speedMult);
        motorL.setPower(left * speedMult);

        // motor telemetry
        telemetry.addData("speedMult: ", speedMult);
        telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f", left));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
    }


    /**
     ******************************
     *         UTILITY            *
     ******************************
     **/


    /**
     * Scale the speed of the drive motors to allow for
     * fine control and speed.
     *
     * @param dVal value to sacale
     * @return scaled value
     */
    private double scaleInput(double dVal)  {
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