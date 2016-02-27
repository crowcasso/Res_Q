package org.usfirst.ftc.aperturescience;

/**
 * AutoPeopleNormal_BLUE (Autonomous)
 *
 * @author FTC 5064 Aperture Science
 */
@org.swerverobotics.library.interfaces.Autonomous(name="BLUE Beach Bum Jimmy", group="Blue")
public class AutoBluePosition2 extends JimmyCentral {

    @Override
    public void main() throws InterruptedException {

        // everything up to waitForStart
        super.main();

        // alliance specific code
        //while (opModeIsActive()) {


        // pull out the climber arm
        setRedArm();

        // pull out the tapes
        setTapes();

        // drive back 69 inches or until we find the white line
        boolean foundWhite = driveBackToWhite(.3, 99);
        Thread.sleep(200);

        if (foundWhite) {   // yeah! found the white line

            System.out.println("found white!");
            Thread.sleep(800);

            // turn left 50 degrees
            turnGyro(50);
            Thread.sleep(200);

            // drive away from wall looking for blue
            boolean foundBlue = driveToBlue(.2, 18);
            Thread.sleep(200);

            if (!foundBlue){
                driveBack(.3,26);
            } else {
                driveBack(.2, 14.5);
                Thread.sleep(200);
            }

        } else {
            // did not find white line -- let's try to correct
            System.out.println("did not find white!");

            // drive forward 2.5 inches
            drive(.1, 2.5);
            Thread.sleep(1000);

            // turn left 50 degrees
            turnGyro(50);
            Thread.sleep(200);

            // drive away from wall looking for blue line
            boolean foundBlue = driveToBlue(.2, 12);
            Thread.sleep(200);

            // missed it!
            if (!foundBlue) {
                // drive back looking for blue or white
                foundBlue = driveBackToBlueOrWhite(.2, 23);
                Thread.sleep(200);
            }

            // found it! drive back 14.5 inches
            if (foundBlue) {
                driveBack(.2, 14.5);
                Thread.sleep(200);
            }
        }

        // drop the climbers
        autoArmNoJimmy();
        
        //stop();

        // error checking
            /*
            Thread.sleep(1000);
            telemetry.addData("Final Heading", getHeading());
            telemetry.update();
            */

        // we're done!
        //break;
        //}
    }
}