package us.ihmc.geometry.tuple4D.interfaces;

public interface QuaternionBasics extends QuaternionReadOnly, Tuple4DBasics
{
   public void setUnsafe(double qx, double qy, double qz, double qs);
}
