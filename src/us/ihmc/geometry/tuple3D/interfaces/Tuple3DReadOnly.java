package us.ihmc.geometry.tuple3D.interfaces;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.geometry.interfaces.EpsilonComparable;
import us.ihmc.geometry.tuple3D.Tuple3DTools;

public interface Tuple3DReadOnly<T extends Tuple3DReadOnly<T>> extends EpsilonComparable<T>
{
   double getX();

   double getY();

   double getZ();

   default float getX32()
   {
      return (float) getX();
   }

   default float getY32()
   {
      return (float) getY();
   }

   default float getZ32()
   {
      return (float) getZ();
   }

   default boolean containsNaN()
   {
      return Double.isNaN(getX()) || Double.isNaN(getY()) || Double.isNaN(getZ());
   }

   default double get(int index)
   {
      switch (index)
      {
      case 0:
         return getX();
      case 1:
         return getY();
      case 2:
         return getZ();
      default:
         throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   default double get32(int index)
   {
      switch (index)
      {
      case 0:
         return getX32();
      case 1:
         return getY32();
      case 2:
         return getZ32();
      default:
         throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   default void get(double[] tupleArrayToPack)
   {
      get(0, tupleArrayToPack);
   }

   default void get(int startIndex, double[] tupleArrayToPack)
   {
      tupleArrayToPack[startIndex++] = getX();
      tupleArrayToPack[startIndex++] = getY();
      tupleArrayToPack[startIndex] = getZ();
   }

   default void get(float[] tupleArrayToPack)
   {
      get(0, tupleArrayToPack);
   }

   default void get(int startIndex, float[] tupleArrayToPack)
   {
      tupleArrayToPack[startIndex++] = getX32();
      tupleArrayToPack[startIndex++] = getY32();
      tupleArrayToPack[startIndex] = getZ32();
   }

   default void get(DenseMatrix64F tupleMatrixToPack)
   {
      get(0, 0, tupleMatrixToPack);
   }

   default void get(int startRow, DenseMatrix64F tupleMatrixToPack)
   {
      get(startRow, 0, tupleMatrixToPack);
   }

   default void get(int startRow, int column, DenseMatrix64F tupleMatrixToPack)
   {
      tupleMatrixToPack.set(startRow++, column, getX());
      tupleMatrixToPack.set(startRow++, column, getY());
      tupleMatrixToPack.set(startRow, column, getZ());
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
         return getX() == other.getX() && getY() == other.getY() && getZ() == other.getZ();
      }
      catch (NullPointerException e)
      {
         return false;
      }
   }
}