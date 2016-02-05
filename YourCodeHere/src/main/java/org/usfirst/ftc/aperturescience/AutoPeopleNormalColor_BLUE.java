package org.usfirst.ftc.aperturescience;

/**
 * AutoPeopleNormal_BLUE (Autonomous)
 *
 * @author FTC 5064 Aperture Science
 */
@org.swerverobotics.library.interfaces.Autonomous
public class AutoPeopleNormalColor_BLUE extends AutoPeopleNormal {

    @Override
    public void main() throws InterruptedException {

        // everything up to waitForStart
        super.main();

        // alliance specific code
        while (opModeIsActive()) {

            // drive back 20 inches
            driveBack(.3, 20);
            Thread.sleep(1000);

            // turn left 40 degrees
            turnGyro(40);
            Thread.sleep(200);

            // pull out the climber arm
            setRedArm();

            // drive back 69 inches or until we find the white line
            boolean foundWhite = driveBackToWhite(.3, 69);
            Thread.sleep(200);

            if (foundWhite) {   // yeah! found the white line

                //System.out.println("found white!");
                /*
                // back up 1.5 inches
                drive(.1, 1.5);
                */
                Thread.sleep(1000);

                // turn left 50 degrees
                turnGyro(50);
                Thread.sleep(200);

                // drive away from wall looking for blue
                boolean foundBlue = driveToBlue(.2, 18);
                Thread.sleep(200);

                if (!foundBlue){
                    driveBack(.3,26);
                } else {
                    driveBack(.2, 13.5);
                    Thread.sleep(200);
                }

            } else {
                // did not find white line -- let's try to correct
                //System.out.println("did not find white!");

                // drive forward 4.5 inches
                drive(.1, 4.5);
                Thread.sleep(500);

                // turn left 50 degrees
                turnGyro(50);
                Thread.sleep(200);

                // drive away from wall looking for blue line
                boolean foundBlue = driveToBlue(.2, 12);
                Thread.sleep(500);

                // missed it!
                if (!foundBlue) {
                    // drive back looking for blue or white
                    foundBlue = driveBackToBlueOrWhite(.2, 26);
                    Thread.sleep(500);
                }

                // drive back 13.5 inches
                if (foundBlue) {
                    driveBack(.2, 13.5);
                    Thread.sleep(200);
                }
            }

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