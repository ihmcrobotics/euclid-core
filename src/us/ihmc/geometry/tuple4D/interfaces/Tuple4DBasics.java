package us.ihmc.geometry.tuple4D.interfaces;

import us.ihmc.geometry.interfaces.GeometryObject;

public interface Tuple4DBasics<T extends Tuple4DBasics<T>> extends Tuple4DReadOnly<T>, GeometryObject<T>
{
   void negate();

   @Override
   void setToZero();

   @Override
   void setToNaN();

   void set(Tuple4DReadOnly<?> tuple);

   void set(double x, double y, double z, double s);
}