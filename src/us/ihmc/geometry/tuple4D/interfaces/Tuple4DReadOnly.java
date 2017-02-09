package us.ihmc.geometry.tuple4D.interfaces;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.geometry.interfaces.EpsilonComparable;
import us.ihmc.geometry.tuple4D.Tuple4DTools;

/**
 * Read-only inter
 * 
 * @author Sylvain Bertrand
 *
 * @param <T>
 */
public interface Tuple4DReadOnly<T extends Tuple4DReadOnly<T>> extends EpsilonComparable<T>
{
   double getX();

   double getY();

   double getZ();

   double getS();

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

   default float getS32()
   {
      return (float) getS();
   }

   default boolean containsNaN()
   {
      return Double.isNaN(getX()) || Double.isNaN(getY()) || Double.isNaN(getZ()) || Double.isNaN(getS());
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
      case 3:
         return getS();
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
      case 3:
         return getS32();
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
      tupleArrayToPack[startIndex++] = getZ();
      tupleArrayToPack[startIndex] = getS();
   }

   default void get(float[] tupleArrayToPack)
   {
      get(0, tupleArrayToPack);
   }

   default void get(int startIndex, float[] tupleArrayToPack)
   {
      tupleArrayToPack[startIndex++] = getX32();
      tupleArrayToPack[startIndex++] = getY32();
      tupleArrayToPack[startIndex++] = getZ32();
      tupleArrayToPack[startIndex] = getS32();
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
      tupleMatrixToPack.set(startRow++, column, getZ());
      tupleMatrixToPack.set(startRow, column, getS());
   }

   default double length()
   {
      return Math.sqrt(lengthSquared());
   }

   default double lengthSquared()
   {
      return dot(this);
   }

   default double dot(Tuple4DReadOnly<?> other)
   {
      return getX() * other.getX() + getY() * other.getY() + getZ() * other.getZ() + getS() * other.getS();
   }

   @Override
   default boolean epsilonEquals(T other, double epsilon)
   {
      return Tuple4DTools.epsilonEquals(this, other, epsilon);
   }

   default boolean equals(T other)
   {
      try
      {
         return getX() == other.getX() && getY() == other.getY() && getZ() == other.getZ() && getS() == other.getS();
      }
      catch (NullPointerException e)
      {
         return false;
      }
   }
}