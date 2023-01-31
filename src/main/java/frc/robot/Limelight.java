package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {

  //============== Limelight Constants ================
    public static final int LED_ON = 3;
    public static final int LED_OFF = 1;
    public static final int REFLECTIVE_PIPELINE = 0;
    public static final int DEFAULT_PIPELINE = 1;
    public static final int APRILTAG_PIPELINE = 2;
  //===================================================


  // Instance Variables
    private NetworkTable table;
    private NetworkTableEntry ledMode;

    // =================== Limelight Constructor ======================
    public Limelight(String NetworkTableName, int DefaultPipeline) {

        this.table = NetworkTableInstance.getDefault().getTable(NetworkTableName);
        this.ledMode = table.getEntry("ledMode");

        setPipeline(DefaultPipeline);
        setLedOn(false);

    }

    // =========================================================================
    // ======================= Create Lightlights here =========================

    public static Limelight limelight1 = new Limelight("limelight", APRILTAG_PIPELINE);


    // ========================================================================
    // ======================== Setters and Getters ===========================

    public void setPipeline(int pipeline){
        this.table.getEntry("pipeline").setNumber(pipeline);
      }

    public void setLedOn(boolean isOn) {
        if (isOn){
          ledMode.setNumber(LED_ON);
        } else {
          ledMode.setNumber(LED_OFF);
        }
      }

      public void togglePipeline() {
        if (getPipelineInt() == APRILTAG_PIPELINE){
          this.table.getEntry("pipeline").setNumber(REFLECTIVE_PIPELINE);
        } else if (getPipelineInt() == REFLECTIVE_PIPELINE){
          this.table.getEntry("pipeline").setNumber(APRILTAG_PIPELINE);
        }
      }

      public double getTagID() {
        NetworkTableEntry tid = table.getEntry("tid");
        return tid.getDouble(0.0);
    }

    public double getTagYaw() {
        NetworkTableEntry camtran = table.getEntry("campose");
        double[] temp = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double[] values = camtran.getDoubleArray(temp);
        return values[4];
    }
    /**
     * tv   Whether the limelight has any valid targets (0 or 1)
     * @return True if target found and vise-versa
     */
    public boolean getIsTargetFound() {
        NetworkTableEntry tv = table.getEntry("tv");
        double v = tv.getDouble(0);
        return (v == 0.0f ? false : true);
    }
    /**
     * tx Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
     * @return Current Horizontal Error
     */
    public double getHorizontalError() {
        NetworkTableEntry tx = table.getEntry("tx");
        double x = tx.getDouble(0.0);
        return x;
    }
    /**
     * ty Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
     * @return Current Vertical Error
     */
    public double getVerticalError() {
        NetworkTableEntry ty = table.getEntry("ty");
        double y = ty.getDouble(0.0);
        return y;
    }
    /**
     * ta Target Area (0% of image to 100% of image)
     * @return Area taken up by target
     */
    public double getTargetArea() {
        NetworkTableEntry ta = table.getEntry("ta");
        double a = ta.getDouble(0.0);
        return a;
    }

    public Integer getPipelineInt(){
      NetworkTableEntry pipeline = table.getEntry("pipeline");
      Integer pipe = (int) pipeline.getDouble(0.0);
      return pipe;
  }

  public NetworkTable getNetworkTable(Limelight limelight) {
    return table;
  }

    // =======================================================================
    // ======================= Enable Pipelines ==============================

    public void enableApriltagTargeting() {
        setLedOn(false);
        setPipeline(APRILTAG_PIPELINE);
      }
      
      public void enableReflectionTargeting() {
        setPipeline(REFLECTIVE_PIPELINE);
        setLedOn(true);
      }
}
