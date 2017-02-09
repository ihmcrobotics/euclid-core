package us.ihmc.geometry.tuple4D.interfaces;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.geometry.GeometryBasicsTools;
import us.ihmc.geometry.TupleTools;
import us.ihmc.geometry.interfaces.EpsilonComparable;

/**
 * Read-only interface for a 4 dimensional tuple.
 * <p>
 * A tuple 4D represents what is commonly called a quaternion.
 * Although from definition, a quaternion does not necessarily represent an 3D orientation, in this library
 * the classes implementing {@link QuaternionReadOnly} and {@link QuaternionBasics} represent unit-quaternions
 * meant to represent 3D orientations.
 * The classes implementing {@link Vector4DReadOnly} and {@link Vector4DBasics} are used to represent generic quaternions.
 * </p>
 * <p>
 * When describing a 4D tuple, its 4 components are often gathered in two groups: the scalar part {@code s}
 * and the vector part ({@code x}, {@code y}, {@code z}).
 * </p>
 * <p>
 * Note on the difference between applying a 3D transform on a quaternion and a 4D vector:
 * <ul>
 *    <li> When transformed by a homogeneous transformation matrix, a quaternion is only
 *     pre-multiplied by the rotation part of the transform, resulting in concatenating
 *     the orientations of the transform and the quaternion.
 *    <li> When transformed by a homogeneous transformation matrix, a 4D vector scalar
 *     part {@code s} remains unchanged. The vector part ({@code x}, {@code y}, {@code z})
 *     is scaled and rotated, and translated by {@code s} times the translation part of the transform.
 *     Note that for {@code s = 0}, a 4D vector behaves as a 3D vector, and for {@code s = 1}
 *     it behaves as a 3D point.
 * </ul>
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 * @param <T> The final type of the tuple used.
 */
public interface Tuple4DReadOnly<T extends Tuple4DReadOnly<T>> extends EpsilonComparable<T>
{
   /**
    * Returns the x-component of this tuple.
    * 
    * @return the x-component.
    */
   double getX();

   /**
    * Returns the y-component of this tuple.
    * 
    * @return the y-component.
    */
   double getY();

   /**
    * Returns the z-component of this tuple.
    * 
    * @return the z-component.
    */
   double getZ();

   /**
    * Returns the s-component of this tuple.
    * 
    * @return the s-component.
    */
   double getS();

   /**
    * Returns the x-component of this tuple.
    * 
    * @return the x-component.
    */
   default float getX32()
   {
      return (float) getX();
   }

   /**
    * Returns the y-component of this tuple.
    * 
    * @return the y-component.
    */
   default float getY32()
   {
      return (float) getY();
   }

   /**
    * Returns the z-component of this tuple.
    * 
    * @return the z-component.
    */
   default float getZ32()
   {
      return (float) getZ();
   }

   /**
    * Returns the s-component of this tuple.
    * 
    * @return the s-component.
    */
   default float getS32()
   {
      return (float) getS();
   }

   /**
    * Tests if this tuple contains a {@link Double#NaN}.
    * 
    * @return {@code true} if this tuple contains a {@link Double#NaN}, {@code false} otherwise.
    */
   default boolean containsNaN()
   {
      return GeometryBasicsTools.containsNaN(getX(), getY(), getZ(), getS());
   }

   /**
    * Selects a component of this tuple based on {@code index}
    * and returns its value.
    * <p>
    * For an {@code index} value going from 0 up to 3, the corresponding components
    * are {@code x}, {@code y}, {@code z}, and {@code s}, respectively.
    * </p>
    * 
    * @param index the index of the component to get.
    * @return the value of the component.
    * @throws IndexOutOfBoundsException if {@code index} &notin; [0, 3].
    */
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

   /**
    * Selects a component of this tuple based on {@code index}
    * and returns its value.
    * <p>
    * For an {@code index} value going from 0 up to 3, the corresponding components
    * are {@code x}, {@code y}, {@code z}, and {@code s}, respectively.
    * </p>
    * 
    * @param index the index of the component to get.
    * @return the value of the component.
    * @throws IndexOutOfBoundsException if {@code index} &notin; [0, 3].
    */
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

   /**
    * Packs the components {@code x}, {@code y}, {@code z}, {@code s} in order
    * in an array starting from its first index.
    * 
    * @param tupleArrayToPack the array in which this tuple is stored. Modified.
    */
   default void get(double[] tupleArrayToPack)
   {
      get(0, tupleArrayToPack);
   }

   /**
    * Packs the components {@code x}, {@code y}, {@code z}, {@code s} in order
    * in an array starting from {@code startIndex}.
    *
    * @param startIndex the index in the array where the first component is stored.
    * @param tupleArrayToPack the array in which this tuple is stored. Modified.
    */
   default void get(int startIndex, double[] tupleArrayToPack)
   {
      tupleArrayToPack[startIndex++] = getX();
      tupleArrayToPack[startIndex++] = getY();
      tupleArrayToPack[startIndex++] = getZ();
      tupleArrayToPack[startIndex] = getS();
   }

   /**
    * Packs the components {@code x}, {@code y}, {@code z}, {@code s} in order
    * in an array starting from its first index.
    * 
    * @param tupleArrayToPack the array in which this tuple is stored. Modified.
    */
   default void get(float[] tupleArrayToPack)
   {
      get(0, tupleArrayToPack);
   }

   /**
    * Packs the components {@code x}, {@code y}, {@code z}, {@code s} in order
    * in an array starting from {@code startIndex}.
    *
    * @param startIndex the index in the array where the first component is stored.
    * @param tupleArrayToPack the array in which this tuple is stored. Modified.
    */
   default void get(int startIndex, float[] tupleArrayToPack)
   {
      tupleArrayToPack[startIndex++] = getX32();
      tupleArrayToPack[startIndex++] = getY32();
      tupleArrayToPack[startIndex++] = getZ32();
      tupleArrayToPack[startIndex] = getS32();
   }

   /**
    * Packs the components {@code x}, {@code y}, {@code z}, {@code s} in order
    * in a column vector starting from its first row index.
    * 
    * @param tupleMatrixToPack the array in which this tuple is stored. Modified.
    */
   default void get(DenseMatrix64F tupleMatrixToPack)
   {
      get(0, 0, tupleMatrixToPack);
   }

   /**
    * Packs the components {@code x}, {@code y}, {@code z}, {@code s} in order
    * in a column vector starting from {@code startRow}.
    * 
    * @param startRow the first row index to start writing in the dense-matrix.
    * @param tupleMatrixToPack the column vector in which this tuple is stored. Modified.
    */
   default void get(int startRow, DenseMatrix64F tupleMatrixToPack)
   {
      get(startRow, 0, tupleMatrixToPack);
   }

   /**
    * Packs the components {@code x}, {@code y}, {@code z}, {@code s} in order
    * in a column vector starting from {@code startRow} at the column index {@code column}.
    * 
    * @param startRow the first row index to start writing in the dense-matrix.
    * @param column the column index to write in the dense-matrix.
    * @param tupleMatrixToPack the matrix in which this tuple is stored. Modified.
    */
   default void get(int startRow, int column, DenseMatrix64F tupleMatrixToPack)
   {
      tupleMatrixToPack.set(startRow++, column, getX());
      tupleMatrixToPack.set(startRow++, column, getY());
      tupleMatrixToPack.set(startRow++, column, getZ());
      tupleMatrixToPack.set(startRow, column, getS());
   }

   /**
    * Calculates and returns the norm of this tuple.
    * <p>
    * norm = &radic;(x<sup>2</sup> + y<sup>2</sup> + z<sup>2</sup> + s<sup>2</sup>)
    * </p>
    * 
    * @return the norm's value of this tuple.
    */
   default double norm()
   {
      return Math.sqrt(normSquared());
   }

   /**
    * Calculates and returns the square of the norm of this tuple.
    * <p>
    * norm<sup>2</sup> = x<sup>2</sup> + y<sup>2</sup> + z<sup>2</sup> + s<sup>2</sup>
    * </p>
    * <p>
    * This method is usually preferred over {@link #norm()}
    * when calculation speed matters and knowledge of the actual norm does not,
    * i.e. when comparing several tuples by theirs norm.
    * </p>
    * 
    * @return the norm's value of this tuple.
    */
   default double normSquared()
   {
      return dot(this);
   }

   /**
    * Calculates and returns the value of the dot product of this tuple with {@code other}.
    * <p>
    * For instance, the dot product of two tuples p and q is defined as:
    * <br> p . q = &sum;<sub>i=1:4</sub>(p<sub>i</sub> * q<sub>i</sub>)
    * </p>
    * 
    * @param other the other vector used for the dot product. Not modified.
    * @return the value of the dot product.
    */
   default double dot(Tuple4DReadOnly<?> other)
   {
      return getX() * other.getX() + getY() * other.getY() + getZ() * other.getZ() + getS() * other.getS();
   }

   /**
    * Tests on a per component basis if this tuple is equal to the given {@code other} to an {@code epsilon}.
    * 
    * @param other the other tuple to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two tuples are equal, {@code false} otherwise.
    */
   @Override
   default boolean epsilonEquals(T other, double epsilon)
   {
      return TupleTools.epsilonEquals(this, other, epsilon);
   }

   /**
    * Tests on a per component basis, if this tuple is exactly equal to {@code other}.
    * 
    * @param other the other tuple to compare against this. Not modified.
    * @return {@code true} if the two tuples are exactly equal component-wise, {@code false} otherwise.
    */
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