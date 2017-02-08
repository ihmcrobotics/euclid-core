package us.ihmc.geometry.tuple4D.interfaces;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.geometry.interfaces.GeometryObject;

public interface Tuple4DBasics<T extends Tuple4DBasics<T>> extends Tuple4DReadOnly<T>, GeometryObject<T>
{
   void set(double x, double y, double z, double s);

   void normalize();

   @Override
   default void setToNaN()
   {
      set(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
   }

   @Override
   default boolean containsNaN()
   {
      return Tuple4DReadOnly.super.containsNaN();
   }

   default void absolute()
   {
      set(Math.abs(getX()), Math.abs(getY()), Math.abs(getZ()), Math.abs(getS()));
   }

   default void negate()
   {
      set(-getX(), -getY(), -getZ(), -getS());
   }

   @Override
   default void set(T other)
   {
      set(other.getX(), other.getY(), other.getZ(), other.getS());
   }

   default void set(Tuple4DReadOnly<?> tupleReadOnly)
   {
      set(tupleReadOnly.getX(), tupleReadOnly.getY(), tupleReadOnly.getZ(), tupleReadOnly.getS());
   }

   default void set(double[] tupleArray)
   {
      set(tupleArray[0], tupleArray[1], tupleArray[2], tupleArray[3]);
   }

   default void set(int startIndex, double[] tupleArray)
   {
      set(tupleArray[startIndex++], tupleArray[startIndex++], tupleArray[startIndex++], tupleArray[startIndex]);
   }

   default void set(float[] tupleArray)
   {
      set(tupleArray[0], tupleArray[1], tupleArray[2], tupleArray[3]);
   }

   default void set(int startIndex, float[] tupleArray)
   {
      set(tupleArray[startIndex++], tupleArray[startIndex++], tupleArray[startIndex++], tupleArray[startIndex]);
   }

   default void set(DenseMatrix64F matrix)
   {
      set(matrix.get(0, 0), matrix.get(1, 0), matrix.get(2, 0), matrix.get(3, 0));
   }

   default void set(int startRow, DenseMatrix64F matrix)
   {
      set(matrix.get(startRow++, 0), matrix.get(startRow++, 0), matrix.get(startRow++, 0), matrix.get(startRow, 0));
   }

   default void set(int startRow, int column, DenseMatrix64F matrix)
   {
      set(matrix.get(startRow++, column), matrix.get(startRow++, column), matrix.get(startRow++, column), matrix.get(startRow, column));
   }

   default void setAndAbsolute(Tuple4DReadOnly<?> other)
   {
      set(Math.abs(other.getX()), Math.abs(other.getY()), Math.abs(other.getZ()), Math.abs(other.getS()));
   }

   default void setAndNegate(Tuple4DReadOnly<?> other)
   {
      set(-other.getX(), -other.getY(), -other.getZ(), -other.getS());
   }

   default void setAndNormalize(Tuple4DReadOnly<?> other)
   {
      set(other);
      normalize();
   }
}