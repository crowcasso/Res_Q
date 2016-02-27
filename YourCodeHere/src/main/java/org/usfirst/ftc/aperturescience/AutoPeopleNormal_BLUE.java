package org.usfirst.ftc.aperturescience;

/**
 * AutoPeopleNormal_BLUE (Autonomous)
 *
 * @author FTC 5064 Aperture Science
 */
//@org.swerverobotics.library.interfaces.Autonomous
public class AutoPeopleNormal_BLUE extends JimmyCentral {

    @Override
    public void main() throws InterruptedException {

        // everything up to waitForStart
        super.main();

        // alliance specific code
        while (opModeIsActive()) {
            // drive back 35 inches
            driveBack(.3, 35);
            Thread.sleep(200);

            // turn left 45 degrees
            turnGyro(45);
            Thread.sleep(200);

            // drive back 51 inches
            driveBack(.4, 51);
            Thread.sleep(200);

            // turn left 45 degrees
            turnGyro(45);
            Thread.sleep(200);

            // drive back 10 inches
            driveBackDistance(.3, 10, 10);
            Thread.sleep(200);

            // drop the climbers
            autoArm();

            // error checking
            Thread.sleep(1000);
            telemetry.addData("Final Heading", getHeading());
            telemetry.update();

            // we're done!
            break;
        }
    }

}
