package us.ihmc.geometry.tuple4D.interfaces;

import us.ihmc.geometry.transform.interfaces.Transform;
import us.ihmc.geometry.tuple3D.Tuple3DTools;

public interface Vector4DBasics<T extends Vector4DBasics<T>> extends Vector4DReadOnly<T>, Tuple4DBasics<T>
{
   void setX(double x);

   void setY(double y);

   void setZ(double z);

   void setS(double s);

   @Override
   default void setToZero()
   {
      set(0.0, 0.0, 0.0, 0.0);
   }

   @Override
   default void normalize()
   {
      if (containsNaN())
         return;
      scale(1.0 / length());
   }

   default void clipToMax(double max)
   {
      set(Math.min(max, getX()), Math.min(max, getY()), Math.min(max, getZ()), Math.min(max, getS()));
   }

   default void clipToMin(double min)
   {
      set(Math.max(min, getX()), Math.max(min, getY()), Math.max(min, getZ()), Math.max(min, getS()));
   }

   default void clipToMinMax(double min, double max)
   {
      clipToMax(max);
      clipToMin(min);
   }

   default void set(int index, double value)
   {
      switch (index)
      {
      case 0:
         setX(value);
         break;
      case 1:
         setY(value);
         break;
      case 2:
         setZ(value);
         break;
      case 3:
         setS(value);
         break;
      default:
         throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   @Override
   default void set(double x, double y, double z, double s)
   {
      setX(x);
      setY(y);
      setZ(z);
      setS(s);
   }

   default void setAndScale(double scalar, Tuple4DReadOnly<?> other)
   {
      set(scalar * other.getX(), scalar * other.getY(), scalar * other.getZ(), scalar * other.getS());
   }

   default void setAndClipToMax(double max, Tuple4DReadOnly<?> other)
   {
      set(Math.min(max, other.getX()), Math.min(max, other.getY()), Math.min(max, other.getZ()), Math.min(max, other.getS()));
   }

   default void setAndClipToMin(double min, Tuple4DReadOnly<?> other)
   {
      set(Math.max(min, other.getX()), Math.max(min, other.getY()), Math.max(min, other.getZ()), Math.max(min, other.getS()));
   }

   default void setAndClipToMinMax(double min, double max, Tuple4DReadOnly<?> other)
   {
      set(other);
      clipToMinMax(min, max);
   }

   default void add(double x, double y, double z, double s)
   {
      set(getX() + x, getY() + y, getZ() + z, getS() + s);
   }

   default void add(Tuple4DReadOnly<?> other)
   {
      add(other.getX(), other.getY(), other.getZ(), other.getS());
   }

   default void add(Tuple4DReadOnly<?> tuple1, Tuple4DReadOnly<?> tuple2)
   {
      set(tuple1.getX() + tuple2.getX(), tuple1.getY() + tuple2.getY(), tuple1.getZ() + tuple2.getZ(), tuple1.getS() + tuple2.getS());
   }

   default void sub(double x, double y, double z, double s)
   {
      set(getX() - x, getY() - y, getZ() - z, getZ() - s);
   }

   default void sub(Tuple4DReadOnly<?> tupleReadOnly)
   {
      sub(tupleReadOnly.getX(), tupleReadOnly.getY(), tupleReadOnly.getZ(), tupleReadOnly.getS());
   }

   default void sub(Tuple4DReadOnly<?> tuple1, Tuple4DReadOnly<?> tuple2)
   {
      set(tuple1.getX() - tuple2.getX(), tuple1.getY() - tuple2.getY(), tuple1.getZ() - tuple2.getZ(), tuple1.getS() - tuple2.getS());
   }

   default void scale(double scalar)
   {
      scale(scalar, scalar, scalar, scalar);
   }

   default void scale(double scalarX, double scalarY, double scalarZ, double scalarS)
   {
      set(scalarX * getX(), scalarY * getY(), scalarZ * getZ(), scalarS * getS());
   }

   default void scaleAdd(double scalar, Tuple4DReadOnly<?> other)
   {
      scale(scalar);
      add(other);
   }

   default void scaleAdd(double scalar, Tuple4DReadOnly<?> tuple1, Tuple4DReadOnly<?> tuple2)
   {
      setAndScale(scalar, tuple1);
      add(tuple2);
   }

   default void interpolate(Tuple4DReadOnly<?> other, double alpha)
   {
      interpolate(this, other, alpha);
   }

   default void interpolate(Tuple4DReadOnly<?> tuple1, Tuple4DReadOnly<?> tuple2, double alpha)
   {
      double x = Tuple3DTools.interpolate(tuple1.getX(), tuple2.getX(), alpha);
      double y = Tuple3DTools.interpolate(tuple1.getY(), tuple2.getY(), alpha);
      double z = Tuple3DTools.interpolate(tuple1.getZ(), tuple2.getZ(), alpha);
      double s = Tuple3DTools.interpolate(tuple1.getS(), tuple2.getS(), alpha);
      set(x, y, z, s);
   }

   @Override
   default void applyTransform(Transform transform)
   {
      transform.transform(this);
   }
}
