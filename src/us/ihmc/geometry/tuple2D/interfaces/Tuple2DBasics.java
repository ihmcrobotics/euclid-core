package us.ihmc.geometry.tuple2D.interfaces;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.geometry.interfaces.GeometryObject;
import us.ihmc.geometry.tuple3D.Tuple3DTools;

public interface Tuple2DBasics<T extends Tuple2DBasics<T>> extends Tuple2DReadOnly<T>, GeometryObject<T>
{
   void setX(double x);

   void setY(double y);

   @Override
   default void setToZero()
   {
      set(0.0, 0.0);
   }

   @Override
   default void setToNaN()
   {
      set(Double.NaN, Double.NaN);
   }

   @Override
   default boolean containsNaN()
   {
      return Tuple2DReadOnly.super.containsNaN();
   }

   default void absolute()
   {
      set(Math.abs(getX()), Math.abs(getY()));
   }

   default void negate()
   {
      set(-getX(), -getY());
   }

   default void clipToMax(double max)
   {
      set(Math.min(max, getX()), Math.min(max, getY()));
   }

   default void clipToMin(double min)
   {
      set(Math.max(min, getX()), Math.max(min, getY()));
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
      default:
         throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   default void set(double x, double y)
   {
      setX(x);
      setY(y);
   }

   @Override
   default void set(T other)
   {
      set(other.getX(), other.getY());
   }

   default void set(Tuple2DReadOnly<?> tupleBasics)
   {
      set(tupleBasics.getX(), tupleBasics.getY());
   }

   default void set(double[] tupleArray)
   {
      set(tupleArray[0], tupleArray[1]);
   }

   default void set(double[] tupleArray, int startIndex)
   {
      set(tupleArray[startIndex++], tupleArray[startIndex]);
   }

   default void set(float[] tupleArray)
   {
      set(tupleArray[0], tupleArray[1]);
   }

   default void set(float[] tupleArray, int startIndex)
   {
      set(tupleArray[startIndex++], tupleArray[startIndex]);
   }

   default void set(DenseMatrix64F matrix)
   {
      set(matrix.get(0, 0), matrix.get(1, 0));
   }

   default void set(DenseMatrix64F matrix, int startRow)
   {
      set(matrix.get(startRow++, 0), matrix.get(startRow, 0));
   }

   default void set(DenseMatrix64F matrix, int startRow, int column)
   {
      set(matrix.get(startRow++, column), matrix.get(startRow, column));
   }

   default void setAndAbsolute(Tuple2DReadOnly<?> other)
   {
      set(Math.abs(other.getX()), Math.abs(other.getY()));
   }

   default void setAndNegate(Tuple2DReadOnly<?> other)
   {
      set(-other.getX(), -other.getY());
   }

   default void setAndClipToMax(double max, Tuple2DReadOnly<?> other)
   {
      set(Math.min(max, other.getX()), Math.min(max, other.getY()));
   }

   default void setAndClipToMin(double min, Tuple2DReadOnly<?> other)
   {
      set(Math.max(min, other.getX()), Math.max(min, other.getY()));
   }

   default void setAndClipToMinMax(double min, double max, Tuple2DReadOnly<?> other)
   {
      set(other);
      clipToMinMax(min, max);
   }

   default void add(double x, double y)
   {
      set(getX() + x, getY() + y);
   }

   default void add(Tuple2DReadOnly<?> tupleBasics)
   {
      add(tupleBasics.getX(), tupleBasics.getY());
   }

   default void add(Tuple2DReadOnly<?> tuple1, Tuple2DReadOnly<?> tuple2)
   {
      set(tuple1.getX() + tuple2.getX(), tuple2.getY() + tuple2.getY());
   }

   default void sub(double x, double y)
   {
      set(getX() - x, getY() - y);
   }

   default void sub(Tuple2DReadOnly<?> tupleBasics)
   {
      sub(tupleBasics.getX(), tupleBasics.getY());
   }

   default void sub(Tuple2DReadOnly<?> tuple1, Tuple2DReadOnly<?> tuple2)
   {
      set(tuple1.getX() - tuple2.getX(), tuple2.getY() - tuple2.getY());
   }

   default void scale(double scalar)
   {
      scale(scalar, scalar);
   }

   default void scale(double scalarX, double scalarY)
   {
      set(scalarX * getX(), scalarY * getY());
   }

   default void scale(double scalar, Tuple2DReadOnly<?> other)
   {
      set(scalar * other.getX(), scalar * other.getY());
   }

   default void scaleAdd(double scalar, Tuple2DReadOnly<?> other)
   {
      scale(scalar);
      add(other);
   }

   default void scaleAdd(double scalar, Tuple2DReadOnly<?> tuple1, Tuple2DReadOnly<?> tuple2)
   {
      scale(scalar, tuple1);
      add(tuple2);
   }

   default void interpolate(Tuple2DReadOnly<?> other, double alpha)
   {
      interpolate(this, other, alpha);
   }

   default void interpolate(Tuple2DReadOnly<?> tuple1, Tuple2DReadOnly<?> tuple2, double alpha)
   {
      double x = Tuple3DTools.interpolate(tuple1.getX(), tuple2.getX(), alpha);
      double y = Tuple3DTools.interpolate(tuple1.getY(), tuple2.getY(), alpha);
      set(x, y);
   }
}
