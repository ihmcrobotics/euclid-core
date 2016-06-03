package us.ihmc.geometry.axisAngle.interfaces;

public interface AxisAngleBasics extends AxisAngleReadOnly
{
   public abstract void setToZero();
   public abstract void setToNaN();
   
   public abstract void set(double x, double y, double z, double angle);

   public abstract void setAngle(double angle);

   public abstract void setX(double x);

   public abstract void setY(double y);

   public abstract void setZ(double z);
}