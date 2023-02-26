
package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
/*import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;
import com.ctre.phoenix.CANifier.LEDPulseTime;
import com.ctre.phoenix.CANifier.LEDThrottleTime;
import com.ctre.phoenix.CANifier.PWMChannel;
import com.ctre.phoenix.CANifierStatusFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;
import edu.wpi.first.cameraserver.CameraServer;
import com.arcrobotics.ftclib.vision.Limelight;
*/


public class LimelightPipeline {

  //Limelight limelight = new Limelight();
  //limelight.enable();

  public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  public NetworkTableEntry tv = table.getEntry("tv"); // Whether the limelight has any valid targets (0 or 1)
  
  public NetworkTableEntry ty = table.getEntry("ty"); // Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
  public NetworkTableEntry ta = table.getEntry("ta"); // Target Area (0% of image to 100% of image)
  public boolean tapePipeline = table.getEntry("FTape").setNumber(0);
  public boolean tagPipelineRed = table.getEntry("FTagR").setNumber(1);
  public boolean tagPipelineBlue = table.getEntry("FTagB").setNumber(2);


  /*table.getEntry("camMode").setNumber(0);
  int imageWidth = (int) table.getEntry("camWidth").getDouble();
  int imageHeight = (int) table.getEntry("camHeight").getDouble();
  Mat image = new Mat();
  VideoCapture camera = new VideoCapture(0);
  

  while (true) {
    if (camera.read(image)) {
        // Process the image with your pipeline code
        // For example, you could use OpenCV to apply image filters
        Imgproc.cvtColor(image, image, Imgproc.COLOR_BGR2HSV);

        // Display the processed image on the dashboard
        SmartDashboard.putNumber("Image Width", image.width());
        SmartDashboard.putNumber("Image Height", image.height());
        CameraServer.getInstance().putVideo("Processed Image", image);
    }
}
*/


}