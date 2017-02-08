package us.ihmc.geometry.tuple3D.interfaces;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.geometry.interfaces.GeometryObject;
import us.ihmc.geometry.tuple3D.Tuple3DTools;

public interface Tuple3DBasics<T extends Tuple3DBasics<T>> extends Tuple3DReadOnly<T>, GeometryObject<T>
{
   public void setX(double x);

   public void setY(double y);

   public void setZ(double z);

   @Override
   default void setToZero()
   {
      set(0.0, 0.0, 0.0);
   }

   @Override
   default void setToNaN()
   {
      set(Double.NaN, Double.NaN, Double.NaN);
   }

   @Override
   default boolean containsNaN()
   {
      return Tuple3DReadOnly.super.containsNaN();
   }

   default void absolute()
   {
      set(Math.abs(getX()), Math.abs(getY()), Math.abs(getZ()));
   }

   default void negate()
   {
      set(-getX(), -getY(), -getZ());
   }

   default void clipToMax(double max)
   {
      set(Math.min(max, getX()), Math.min(max, getY()), Math.min(max, getZ()));
   }

   default void clipToMin(double min)
   {
      set(Math.max(min, getX()), Math.max(min, getY()), Math.max(min, getZ()));
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
      default:
         throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   default void set(double x, double y, double z)
   {
      setX(x);
      setY(y);
      setZ(z);
   }

   @Override
   default void set(T other)
   {
      set(other.getX(), other.getY(), other.getZ());
   }

   default void set(Tuple3DReadOnly<?> tupleReadOnly)
   {
      set(tupleReadOnly.getX(), tupleReadOnly.getY(), tupleReadOnly.getZ());
   }

   default void set(double[] tupleArray)
   {
      set(tupleArray[0], tupleArray[1], tupleArray[2]);
   }

   default void set(int startIndex, double[] tupleArray)
   {
      set(tupleArray[startIndex++], tupleArray[startIndex++], tupleArray[startIndex]);
   }

   default void set(float[] tupleArray)
   {
      set(tupleArray[0], tupleArray[1], tupleArray[2]);
   }

   default void set(int startIndex, float[] tupleArray)
   {
      set(tupleArray[startIndex++], tupleArray[startIndex++], tupleArray[startIndex]);
   }

   default void set(DenseMatrix64F matrix)
   {
      set(matrix.get(0, 0), matrix.get(1, 0), matrix.get(2, 0));
   }

   default void set(int startRow, DenseMatrix64F matrix)
   {
      set(matrix.get(startRow++, 0), matrix.get(startRow++, 0), matrix.get(startRow, 0));
   }

   default void set(int startRow, int column, DenseMatrix64F matrix)
   {
      set(matrix.get(startRow++, column), matrix.get(startRow++, column), matrix.get(startRow, column));
   }

   default void setAndAbsolute(Tuple3DReadOnly<?> other)
   {
      set(Math.abs(other.getX()), Math.abs(other.getY()), Math.abs(other.getZ()));
   }

   default void setAndNegate(Tuple3DReadOnly<?> other)
   {
      set(-other.getX(), -other.getY(), -other.getZ());
   }

   default void setAndScale(double scalar, Tuple3DReadOnly<?> other)
   {
      set(scalar * other.getX(), scalar * other.getY(), scalar * other.getZ());
   }

   default void setAndClipToMax(double max, Tuple3DReadOnly<?> other)
   {
      set(Math.min(max, other.getX()), Math.min(max, other.getY()), Math.min(max, other.getZ()));
   }

   default void setAndClipToMin(double min, Tuple3DReadOnly<?> other)
   {
      set(Math.max(min, other.getX()), Math.max(min, other.getY()), Math.max(min, other.getZ()));
   }

   default void setAndClipToMinMax(double min, double max, Tuple3DReadOnly<?> other)
   {
      set(other);
      clipToMinMax(min, max);
   }

   default void add(double x, double y, double z)
   {
      set(getX() + x, getY() + y, getZ() + z);
   }

   default void add(Tuple3DReadOnly<?> other)
   {
      add(other.getX(), other.getY(), other.getZ());
   }

   default void add(Tuple3DReadOnly<?> tuple1, Tuple3DReadOnly<?> tuple2)
   {
      set(tuple1.getX() + tuple2.getX(), tuple1.getY() + tuple2.getY(), tuple1.getZ() + tuple2.getZ());
   }

   default void sub(double x, double y, double z)
   {
      set(getX() - x, getY() - y, getZ() - z);
   }

   default void sub(Tuple3DReadOnly<?> tupleReadOnly)
   {
      sub(tupleReadOnly.getX(), tupleReadOnly.getY(), tupleReadOnly.getZ());
   }

   default void sub(Tuple3DReadOnly<?> tuple1, Tuple3DReadOnly<?> tuple2)
   {
      set(tuple1.getX() - tuple2.getX(), tuple1.getY() - tuple2.getY(), tuple1.getZ() - tuple2.getZ());
   }

   default void scale(double scalar)
   {
      scale(scalar, scalar, scalar);
   }

   default void scale(double scalarX, double scalarY, double scalarZ)
   {
      set(scalarX * getX(), scalarY * getY(), scalarZ * getZ());
   }

   default void scaleAdd(double scalar, Tuple3DReadOnly<?> other)
   {
      scale(scalar);
      add(other);
   }

   default void scaleAdd(double scalar, Tuple3DReadOnly<?> tuple1, Tuple3DReadOnly<?> tuple2)
   {
      setAndScale(scalar, tuple1);
      add(tuple2);
   }

   default void interpolate(Tuple3DReadOnly<?> other, double alpha)
   {
      interpolate(this, other, alpha);
   }

   default void interpolate(Tuple3DReadOnly<?> tuple1, Tuple3DReadOnly<?> tuple2, double alpha)
   {
      double x = Tuple3DTools.interpolate(tuple1.getX(), tuple2.getX(), alpha);
      double y = Tuple3DTools.interpolate(tuple1.getY(), tuple2.getY(), alpha);
      double z = Tuple3DTools.interpolate(tuple1.getZ(), tuple2.getZ(), alpha);
      set(x, y, z);
   }

}