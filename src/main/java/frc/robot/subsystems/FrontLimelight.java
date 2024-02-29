package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceIds;

public class FrontLimelight extends SubsystemBase {

   private double ta;
   private double tx;
   private double ty;
   private double tl;
   private double ts;

   NetworkTableEntry prelimtx;
   NetworkTableEntry prelimty;
   NetworkTableEntry prelimta;
   NetworkTableEntry prelimtl;
   NetworkTableEntry prelimts;
   NetworkTableEntry prelimCamtran;
   NetworkTable table;
   NetworkTableInstance Inst;

   public FrontLimelight() {
      Inst = NetworkTableInstance.getDefault();
      table = Inst.getTable(DeviceIds.Limelight.FrontTableName);
      prelimta = table.getEntry("ta");
      prelimtx = table.getEntry("tx");
      prelimty = table.getEntry("ty");
      prelimtl = table.getEntry("tlong");
      prelimts = table.getEntry("tshort");
   }

   public void updateGameState(){
      ta = prelimta.getDouble(ta);
      tx = prelimtx.getDouble(0);
      ty = prelimty.getDouble(ty);
      tl = prelimtl.getDouble(tl);
      ts = prelimts.getDouble(ts);
   }

   public double getArea(){
      ta = prelimta.getDouble(ta);
      return ta;
   }

   public double getX(){
      tx = prelimtx.getDouble(tx);
      return tx;
   }

   public double getY(){
      ty = prelimty.getDouble(ty);
      return ty;
   }

   public double getShort() {
      ts = prelimts.getDouble(ts);
      return ts;
   }

   public double getLong() {
      tl = prelimtl.getDouble(tl);
      return tl;
   }

   public void visionMode(){
      NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("ledMode").setNumber(3);
      NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("camMode").setNumber(0);
   }

   public void cameraMode(){
      NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("ledMode").setNumber(1);
      NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("camMode").setNumber(1);
   }

   public void updateDashboard() {
	   SmartDashboard.putNumber("front ta", getArea());
      SmartDashboard.putNumber("front tx", getX());
      SmartDashboard.putNumber("front ty", getY());
      SmartDashboard.putNumber("front tl", getLong());
      SmartDashboard.putNumber("front ts", getShort());


	}
}