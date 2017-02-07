package us.ihmc.geometry.tuple2D.interfaces;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.geometry.interfaces.EpsilonComparable;
import us.ihmc.geometry.tuple3D.Tuple3DTools;

public interface Tuple2DReadOnly<T extends Tuple2DReadOnly<T>> extends EpsilonComparable<T>
{
   double getX();

   double getY();

   default float getX32()
   {
      return (float) getX();
   }
   
   default float getY32()
   {
      return (float) getY();
   }

   default boolean containsNaN()
   {
      return Double.isNaN(getX()) || Double.isNaN(getY());
   }

   default double get(int index)
   {
      switch (index)
      {
      case 0:
         return getX();
      case 1:
         return getY();
      default:
         throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   default void get(double[] tupleArrayToPack)
   {
      get(tupleArrayToPack, 0);
   }

   default void get(double[] tupleArrayToPack, int startIndex)
   {
      tupleArrayToPack[startIndex++] = getX();
      tupleArrayToPack[startIndex] = getY();
   }

   default void get(float[] tupleArrayToPack)
   {
      get(tupleArrayToPack, 0);
   }

   default void get(float[] tupleArrayToPack, int startIndex)
   {
      tupleArrayToPack[startIndex++] = getX32();
      tupleArrayToPack[startIndex] = getY32();
   }

   default void get(DenseMatrix64F tupleMatrixToPack)
   {
      get(tupleMatrixToPack, 0, 0);
   }

   default void get(DenseMatrix64F tupleMatrixToPack, int startRow)
   {
      get(tupleMatrixToPack, startRow, 0);
   }

   default void get(DenseMatrix64F tupleMatrixToPack, int startRow, int column)
   {
      tupleMatrixToPack.set(startRow++, column, getX());
      tupleMatrixToPack.set(startRow, column, getY());
   }

   @Override
   default boolean epsilonEquals(T other, double epsilon)
   {
      return Tuple3DTools.epsilonEquals(this, other, epsilon);
   }

   default boolean equals(T other)
   {
      try
      {
         return getX() == other.getX() && getY() == other.getY();
      }
      catch (NullPointerException e)
      {
         return false;
      }
   }
}