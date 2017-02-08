package us.ihmc.geometry.tuple4D.interfaces;

public interface QuaternionBasics extends QuaternionReadOnly
{
   void negate();

   void setToZero();

   void setToNaN();

   void set(Tuple4DReadOnly tuple);

   void set(double x, double y, double z, double s);

   void setUnsafe(double qx, double qy, double qz, double qs);
}
