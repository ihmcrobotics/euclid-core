package us.ihmc.geometry.tuple4D.interfaces;

import us.ihmc.geometry.interfaces.GeometryObject;

public interface QuaternionBasics<T extends QuaternionBasics<T>> extends QuaternionReadOnly<T>, GeometryObject<T>
{
   void negate();

   @Override
   void setToZero();

   @Override
   void setToNaN();

   void set(Tuple4DReadOnly<T> tuple);

   void set(double x, double y, double z, double s);

   void setUnsafe(double qx, double qy, double qz, double qs);
}
