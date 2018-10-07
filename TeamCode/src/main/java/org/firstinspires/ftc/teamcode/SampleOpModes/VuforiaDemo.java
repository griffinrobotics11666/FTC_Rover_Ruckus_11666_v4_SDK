package org.firstinspires.ftc.teamcode.SampleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/*
 * This OpMode was written for the VuforiaDemo Basics video. This demonstrates basic principles of
 * using VuforiaDemo in FTC.
 */
@Autonomous(name = "Vuforia")
public class VuforiaDemo extends LinearOpMode
{
    // Variables to be used for later
    private VuforiaLocalizer vuforiaLocalizer;
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables visionTargets;
    private VuforiaTrackable target;
    private VuforiaTrackableDefaultListener listener;

    private OpenGLMatrix lastKnownLocation;
    private OpenGLMatrix phoneLocation;

    private static final String VUFORIA_KEY = "AdYYXLj/////AAABmbrz6/MNLUKlnU5JIPwkiDQ5jX+GIjfuIEgba3irGu46iS/W1Q9Z55uLSl31zGtBX3k5prkoSK6UxLR9gyvyIwSzRe2FOFGHEvJ19uG+pqiJJfkaRb0mCUkrx4U/fH6+Agp+7lOHB8IYjziNSuBMgABbrii5tAQiXOGfGojY+IQ/enBoy+zWiwVBx9cPRBsEHu+ipK6RXQe7CeODCRN8anBfAsn5b2BoO9lcGE0DgZdRysyByQ4wuwNQxKjba18fnzSDWpm12Brx3Ao1vkGYxTyLQfsON5VotphvWwoZpoyD+Iav/yQmOxrQDBLox6SosF8jqG9sUC5LAAdiRIWr6sNRrGzeCtsHSJBplHboPMB3";

    private float robotX = 0;
    private float robotY = 0;
    private float robotAngle = 0;

    public void runOpMode() throws InterruptedException
    {
        setupVuforia();

        // We don't know where the robot is, so set it to the origin
        // If we don't include this, it would be null, which would cause errors later on
        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);

        waitForStart();

        // Start tracking the targets
        visionTargets.activate();

        while(opModeIsActive())
        {
            // Ask the listener for the latest information on where the robot is
            OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

            // The listener will sometimes return null, so we check for that to prevent errors
            if(latestLocation != null)
                lastKnownLocation = latestLocation;

            float[] coordinates = lastKnownLocation.getTranslation().getData();

            robotX = coordinates[0];
            robotY = coordinates[1];
            robotAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

            // Send information about whether the target is visible, and where the robot is
            telemetry.addData("Tracking " + target.getName(), listener.isVisible());
            telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));

            // Send telemetry and idle to let hardware catch up
            telemetry.update();
            idle();
        }
    }
//R.id.cameraMonitorViewId
    private void setupVuforia()
    {
        float distanceFromCenter = 1829;
        // Setup parameters to create localizer
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId); // To remove the camera view from the screen, remove the cameraMonitorViewId
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        // These are the vision targets that we want to use
        // The string needs to be the name of the appropriate .xml file in the assets folder
        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("RoverRuckus");
       /*0: <ImageTarget name="BluePerimeter" size="254.000000 183.280029" />
         1: <ImageTarget name="RedPerimeter" size="254.000000 181.783096" />
         2: <ImageTarget name="FrontPerimeter" size="254.000000 158.794403" />
         3: <ImageTarget name="BackPerimeter" size="254.000000 199.508209" />
         6ft is 1828.8 mm
    */
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        // Setup the target to be tracked
        target = visionTargets.get(0); // 0 corresponds to the wheels target
        target.setName("BluePerimeter - Rover");
        target.setLocation(createMatrix(0, distanceFromCenter, 0, 90, 0, 0));

        target = visionTargets.get(1); // 0 corresponds to the wheels target
        target.setName("RedPerimeter - Footprint");
        target.setLocation(createMatrix(0, -distanceFromCenter, 0, 90, 0, 180));

        target = visionTargets.get(2); // 0 corresponds to the wheels target
        target.setName("FrontPerimeter - MoonSurface");
        target.setLocation(createMatrix(-distanceFromCenter, 0, 0, 90, 0, 90));

        target = visionTargets.get(3); // 0 corresponds to the wheels target
        target.setName("BackPerimeter - Nebula");
        target.setLocation(createMatrix(distanceFromCenter, 0, 0, 90, 0, -90));


        // Set phone location on robot
        phoneLocation = createMatrix(0, 225, 0, 90, 0, 0);

        // Setup listener and inform it of phone information
        listener = (VuforiaTrackableDefaultListener) target.getListener();
        listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
    }

    // Creates a matrix for determining the locations and orientations of objects
    // Units are millimeters for x, y, and z, and degrees for u, v, and w
    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    // Formats a matrix into a readable string
    private String formatMatrix(OpenGLMatrix matrix)
    {
        return matrix.formatAsTransform();
    }
}
