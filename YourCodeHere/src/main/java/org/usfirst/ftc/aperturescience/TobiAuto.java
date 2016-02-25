package org.usfirst.ftc.aperturescience;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.swerverobotics.library.ClassFactory;
import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.IBNO055IMU;
import org.swerverobotics.library.interfaces.Position;
import org.swerverobotics.library.interfaces.Velocity;

/**
 * AutoPeopleNormal (Autonomous)
 *
 * This is the main code for autonomous. Override main()
 * to add run specific instructions.
 *
 * @author FTC 5064 Aperture Science
 */
@org.swerverobotics.library.interfaces.Autonomous
public class TobiAuto extends SynchronousOpMode {

    // Hardware
    private DcMotor motor;

    /* motor constants */
    private final int ENCODER_CPR = 1120;
    private final double GEAR_RATIO = 1;
    private final double WHEEL_DIAMETER = 4.9;
    private final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;



    @Override
    protected void main() throws InterruptedException {

        // motors
        motor = hardwareMap.dcMotor.get("motor");

        // run certain motors using encoders
        motor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        waitForStart();

        driveAcc(50000, 22000, 1, .4);
        Thread.sleep(1000);
    }

    /* convert inches (distance) to rotations (motor) */
    public int inchesToRotations(double distance) {
        double rotations = distance/CIRCUMFERENCE;
        return (int)(ENCODER_CPR * rotations * GEAR_RATIO);
    }

    /*  drive forward without error correction */
    public void driveAcc(double totalDist, double rampDist, double finalPower, double initPower ) throws InterruptedException {
        //int n = inchesToRotations(distance);
        int start = motor.getCurrentPosition();
        finalPower = Range.clip(finalPower, .2, 1);
        initPower = Range.clip(initPower, .2, 1);
        if(initPower > finalPower) initPower = finalPower;
        if(2*rampDist > totalDist) rampDist = totalDist / 2;

        motor.setPower(initPower);
        int motorPos = motor.getCurrentPosition();
        double newPower = 0;
        double oldPower = 0;

        while (motorPos < (start + totalDist)) {
            if(motorPos < (start + rampDist)) {
                double distThruRamp = ( (motorPos - start) ) / rampDist;
                newPower = initPower + (distThruRamp * (finalPower - initPower));
            } else if (motorPos < (start + totalDist - rampDist)) {
                newPower = finalPower;
            } else {
                double distThruRamp = (totalDist - (motorPos - start)) / rampDist;
                newPower = initPower + (distThruRamp * (finalPower - initPower));
                //newPower = 0.2;
            }
            Thread.sleep(20);

            newPower = Range.clip(newPower, initPower, finalPower);
            if (oldPower != newPower) {
                motor.setPower(newPower);
                oldPower = newPower;
                System.out.println("just set power to " + newPower);
            }
            motorPos = motor.getCurrentPosition();
            telemetry.addData("motorPos", motorPos);
            telemetry.addData("newPower", newPower);
            telemetry.update();
            this.idle();
        }

        motor.setPower(0.0);
    }

    /* drive back without error correction */
    public void driveBack(double power, double distance) throws InterruptedException {
        int n = inchesToRotations(distance);
        int start = motor.getCurrentPosition();

        motor.setPower(-power);

        while (motor.getCurrentPosition() > (start - n)) {
            Thread.sleep(10);
        }

        motor.setPower(0.0);
    }
}