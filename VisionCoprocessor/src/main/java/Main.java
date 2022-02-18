/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionThread;

/*
   JSON format:
   {
       "team": <team number>,
       "ntmode": <"client" or "server", "client" if unspecified>
       "cameras": [
           {
               "name": <camera name>
               "path": <path, e.g. "/dev/video0">
               "pixel format": <"MJPEG", "YUYV", etc>   // optional
               "width": <video mode width>              // optional
               "height": <video mode height>            // optional
               "fps": <video mode fps>                  // optional
               "brightness": <percentage brightness>    // optional
               "white balance": <"auto", "hold", value> // optional
               "exposure": <"auto", "hold", value>      // optional
               "properties": [                          // optional
                   {
                       "name": <property name>
                       "value": <property value>
                   }
               ]
           }
       ]
   }
 */

public final class Main {
  private static String configFile = "/boot/frc.json";

  private static boolean isCamHumanVisible = true;
  private final static String humanVisibleSettingsFile = "/home/pi/humanSettings.json";
  private static JsonObject humanVisibleSettings;
  private final static String machineVisibleSettingsFile = "/home/pi/machineSettings.json";
  private static JsonObject machineVisibleSettings;

  // How far to the right of center is the camera?
  private final static double CAM_X_DEFAULT_OFFSET_IN = 10;
  // How far back from the front of the robot is the camera?
  private final static double CAM_Y_DEFAULT_OFFSET_IN = 10;

  // @SuppressWarnings("MemberName")
  public static class CameraConfig {
    public String name;
    public String path;
    public JsonObject config;
  }

  public static int team;
  public static boolean server;
  public static List<CameraConfig> cameraConfigs = new ArrayList<>();

  private Main() {
  }

  /**
   * Report parse error.
   */
  public static void parseError(String str) {
    System.err.println("config error in '" + configFile + "': " + str);
  }

  /**
   * Read single camera configuration.
   */
  public static boolean readCameraConfig(JsonObject config) {
    CameraConfig cam = new CameraConfig();

    // name
    JsonElement nameElement = config.get("name");
    if (nameElement == null) {
      parseError("could not read camera name");
      return false;
    }
    cam.name = nameElement.getAsString();

    // path
    JsonElement pathElement = config.get("path");
    if (pathElement == null) {
      parseError("camera '" + cam.name + "': could not read path");
      return false;
    }
    cam.path = pathElement.getAsString();

    cam.config = config;

    cameraConfigs.add(cam);
    return true;
  }

public static JsonObject readJsonFile(String path) {
    // parse file
    JsonElement top;
    try {
      top = new JsonParser().parse(Files.newBufferedReader(Paths.get(path)));
    } catch (IOException ex) {
      System.err.println("could not open '" + path + "': " + ex);
      return null;
    }

    // top level must be an object
    if (!top.isJsonObject()) {
      parseError("must be JSON object");
      return null;
    }
    return top.getAsJsonObject();
}

  /**
   * Read configuration file.
   */
  // @SuppressWarnings("PMD.CyclomaticComplexity")
  public static boolean readConfig() {
    JsonObject obj = readJsonFile(configFile);

    if (obj == null) {
      return false;
    }

    // team number
    JsonElement teamElement = obj.get("team");
    if (teamElement == null) {
      parseError("could not read team number");
      return false;
    }
    team = teamElement.getAsInt();

    // ntmode (optional)
    if (obj.has("ntmode")) {
      String str = obj.get("ntmode").getAsString();
      if ("client".equalsIgnoreCase(str)) {
        server = false;
      } else if ("server".equalsIgnoreCase(str)) {
        server = true;
      } else {
        parseError("could not understand ntmode value '" + str + "'");
      }
    }

    // cameras
    JsonElement camerasElement = obj.get("cameras");
    if (camerasElement == null) {
      parseError("could not read cameras");
      return false;
    }
    JsonArray cameras = camerasElement.getAsJsonArray();
    for (JsonElement camera : cameras) {
      if (!readCameraConfig(camera.getAsJsonObject())) {
        return false;
      }
    }

    // Read settings for specific scenarios
    humanVisibleSettings = readJsonFile(humanVisibleSettingsFile);
    if(humanVisibleSettings == null) { return false; }
    machineVisibleSettings = readJsonFile(machineVisibleSettingsFile);
    if(machineVisibleSettings == null) { return false; }

    return true;
  }

  /**
   * Start running the camera.
   */
  public static VideoSource startCamera(CameraConfig config) {
    System.out.println("Starting camera '" + config.name + "' on " + config.path);
    VideoSource camera = CameraServer.getInstance().startAutomaticCapture(
        config.name, config.path);

    Gson gson = new GsonBuilder().create();

    camera.setConfigJson(gson.toJson(config.config));

    return camera;
  }

  /**
   * Main.
   */
  public static void main(String... args) {
    if (args.length > 0) {
      configFile = args[0];
    }

    // read configuration
    if (!readConfig()) {
      return;
    }

    // start NetworkTables
    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
    if (server) {
      System.out.println("Setting up NetworkTables server");
      ntinst.startServer();
    } else {
      System.out.println("Setting up NetworkTables client for team " + team);
      ntinst.startClientTeam(team);
    }
    NetworkTable analysisOutputTable = ntinst.getTable("vision_metrics");
    NetworkTable cameraControlTable = ntinst.getTable("camera_control");
    cameraControlTable.getEntry("camera_for_humans").setBoolean(false);
    cameraControlTable.getEntry("camera_rightward_from_center_in").setDouble(CAM_X_DEFAULT_OFFSET_IN);
    cameraControlTable.getEntry("camera_forward_from_center_in").setDouble(CAM_Y_DEFAULT_OFFSET_IN);
    
    // start cameras
    List<VideoSource> cameras = new ArrayList<>();
    for (CameraConfig cameraConfig : cameraConfigs) {
      cameras.add(startCamera(cameraConfig));
    }

    // start image processing on camera 0 if present
    if (cameras.size() >= 1) {
      VisionThread visionThread = new VisionThread(cameras.get(0),
              new HatchVisionTargetsFromImage(), pipeline -> {
                //analysisOutputTable.getEntry("Hello").setString("World");
                List<HatchVisionTargetsFromImage.HatchVisionTarget> hvts = pipeline.getDetectedTargets();
                double[] bearings = new double[hvts.size()];
                double[] ranges = new double[hvts.size()];

                double dx = cameraControlTable.getEntry("camera_rightward_from_center_in").getDouble(CAM_X_DEFAULT_OFFSET_IN);
                double dy = cameraControlTable.getEntry("camera_forward_from_center_in").getDouble(CAM_Y_DEFAULT_OFFSET_IN);
            
                double[] bearingsRelRobot = new double[hvts.size()];
                double[] rangesRelRobot = new double[hvts.size()];
                
                int i = 0;
                int imgWidth = cameras.get(0).getVideoMode().width;
                for (HatchVisionTargetsFromImage.HatchVisionTarget hvt : hvts) {
                  // Get bearing and range, relative to the camera
                  ranges[i] = hvt.computeRangeInches(imgWidth, HatchVisionTargetsFromImage.CAMERA_FOV_WIDTH_DEG);
                  bearings[i] = hvt.computeBearingDegrees(imgWidth, HatchVisionTargetsFromImage.CAMERA_FOV_WIDTH_DEG);

                  // Convert to cartesian coordinates.  Y+ is ahead of the robot, X+ is rightwards.
                  double xc = ranges[i] * Math.sin(Math.toRadians(bearings[i]));
                  double yc = ranges[i] * Math.cos(Math.toRadians(bearings[i]));
                  // Shift from being camera-relative to being robot-relative
                  double xr = xc + dx;
                  double yr = yc + dy;
                  // Convert back to polar for delivery
                  bearingsRelRobot[i] = Math.toDegrees(Math.atan2(xr, yr));
                  rangesRelRobot[i] = Math.sqrt(xr * xr + yr * yr);

                  i++;
                }
                analysisOutputTable.getEntry("target bearings relative to camera (deg)").setDoubleArray(bearings);
                analysisOutputTable.getEntry("target ranges relative to camera (in)").setDoubleArray(ranges);
                analysisOutputTable.getEntry("target bearings (deg)").setDoubleArray(bearingsRelRobot);
                analysisOutputTable.getEntry("target ranges (in)").setDoubleArray(rangesRelRobot);
                analysisOutputTable.getEntry("target count").setNumber(hvts.size());
      });
      visionThread.start();
    }

    //cameras.get(0).set
    boolean shouldbeSetForHumans = !isCamHumanVisible; // Be contrarian to make sure we set it the first time
    // loop forever
    for (;;) {
      try {
        shouldbeSetForHumans = cameraControlTable.getEntry("camera_for_humans").getBoolean(false);
        if(shouldbeSetForHumans && !isCamHumanVisible) {
          System.out.print("Switching to human-visible settings... ");
          cameras.get(0).setConfigJson(humanVisibleSettings.toString());
          isCamHumanVisible = shouldbeSetForHumans;
          System.out.println("done.");
        } else if (!shouldbeSetForHumans && isCamHumanVisible) {
          System.out.print("Switching to machine-visible settings... ");
          cameras.get(0).setConfigJson(machineVisibleSettings.toString());
          isCamHumanVisible = shouldbeSetForHumans;
          System.out.println("done.");
        }
        Thread.sleep(10);
      } catch (InterruptedException ex) {
        return;
      }
    }
  }
}
