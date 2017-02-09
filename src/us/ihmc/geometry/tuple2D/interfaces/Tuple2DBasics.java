package us.ihmc.geometry.tuple2D.interfaces;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.geometry.TupleTools;
import us.ihmc.geometry.interfaces.GeometryObject;

/**
 * Write and read interface for a 2 dimensional tuple.
 * <p>
 * A tuple is an abstract geometry object holding onto the common math between a 2D point and vector.
 * </p>
 * <p>
 * Although a point and vector hold onto the same type of information, the distinction is made between them
 * as they represent different geometry objects and are typically not handled the same way:
 * <ul>
 *    <li> a point represents the coordinate of a location in space.
 *     A notable difference with a vector is that the distance between two points has a physical meaning.
 *     When a point is transformed with a homogeneous transformation matrix,
 *     a point's coordinates are susceptible to be scaled, rotated, and translated.
 *    <li> a vector is not constrained to a location in space. Instead, a vector represents some
 *     physical quantity that has a direction and a magnitude such as: a velocity, a force, the translation from one
 *     point to another, etc.
 *     When a vector is transformed with a homogeneous transformation matrix,
 *     its components are susceptible to be scaled and rotated, but never to be translated.
 * </ul> 
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 * @param <T> The final type of the tuple used.
 */
// TODO Think about renaming this interface to Tuple2D
public interface Tuple2DBasics<T extends Tuple2DBasics<T>> extends Tuple2DReadOnly<T>, GeometryObject<T>
{
   /**
    * Sets the x-component of this tuple.
    * 
    * @param x the new x-component's value.
    */
   void setX(double x);

   /**
    * Sets the y-component of this tuple.
    * 
    * @param y the new y-component's value.
    */
   void setY(double y);

   /**
    * Sets all the components of this tuple to zero.
    */
   @Override
   default void setToZero()
   {
      set(0.0, 0.0);
   }

   /**
    * Sets all the components of this tuple to {@link Double#NaN}.
    */
   @Override
   default void setToNaN()
   {
      set(Double.NaN, Double.NaN);
   }

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return Tuple2DReadOnly.super.containsNaN();
   }

   /**
    * Sets each component of this tuple to its absolute value.
    */
   default void absolute()
   {
      set(Math.abs(getX()), Math.abs(getY()));
   }

   /**
    * Changes the sign of each component of this tuple.
    */
   default void negate()
   {
      set(-getX(), -getY());
   }

   /**
    * Clips each component of this tuple to a maximum value {@code max}.
    * 
    * @param max the maximum value for each component.
    */
   default void clipToMax(double max)
   {
      set(Math.min(max, getX()), Math.min(max, getY()));
   }

   /**
    * Clips each component of this tuple to a minimum value {@code min}.
    * 
    * @param min the minimum value for each component.
    */
   default void clipToMin(double min)
   {
      set(Math.max(min, getX()), Math.max(min, getY()));
   }

   /**
    * Clips each component of this tuple to a minimum value {@code min} and a maximum value {@code max}.
    * 
    * @param min the minimum value for each component.
    * @param max the maximum value for each component.
    */
   default void clipToMinMax(double min, double max)
   {
      clipToMax(max);
      clipToMin(min);
   }

   /**
    * Selects a component of this tuple based on {@code index}
    * and sets it to {@code value}.
    * <p>
    * For an {@code index} of 0, the corresponding component is {@code x}, while for 1
    * it is {@code y}.
    * </p>
    * 
    * @param index the index of the component to set.
    * @param value the new value of the selected component.
    * @throws IndexOutOfBoundsException if {@code index} &notin; [0, 1].
    */
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

   /**
    * Sets this tuple's components to {@code x} and {@code y}.
    * 
    * @param x the new value for the x-component of this tuple.
    * @param y the new value for the y-component of this tuple.
    */
   default void set(double x, double y)
   {
      setX(x);
      setY(y);
   }

   /**
    * Sets this tuple to {@code other}.
    * 
    * @param other the other tuple to copy the values from. Not modified.
    */
   @Override
   default void set(T other)
   {
      set(other.getX(), other.getY());
   }

   /**
    * Sets this tuple to {@code tupleReadOnly}.
    * 
    * @param tupleReadOnly the other tuple to copy the values from. Not modified.
    */
   default void set(Tuple2DReadOnly<?> tupleReadOnly)
   {
      set(tupleReadOnly.getX(), tupleReadOnly.getY());
   }

   /**
    * Sets this tuple's components {@code x}, {@code y} in order
    * from the given array {@code tupleArray}.
    * 
    * @param tupleArray the array containing the new values for this tuple's components. Not modified.
    */
   default void set(double[] tupleArray)
   {
      set(tupleArray[0], tupleArray[1]);
   }

   /**
    * Sets this tuple's components {@code x}, {@code y} in order
    * from the given array {@code tupleArray}.
    * 
    * @param startIndex the first index to start reading from in the array.
    * @param tupleArray the array containing the new values for this tuple's components. Not modified.
    */
   default void set(int startIndex, double[] tupleArray)
   {
      set(tupleArray[startIndex++], tupleArray[startIndex]);
   }

   /**
    * Sets this tuple's components {@code x}, {@code y} in order
    * from the given array {@code tupleArray}.
    * 
    * @param tupleArray the array containing the new values for this tuple's components. Not modified.
    */
   default void set(float[] tupleArray)
   {
      set(tupleArray[0], tupleArray[1]);
   }

   /**
    * Sets this tuple's components {@code x}, {@code y} in order
    * from the given array {@code tupleArray}.
    * 
    * @param startIndex the first index to start reading from in the array.
    * @param tupleArray the array containing the new values for this tuple's components. Not modified.
    */
   default void set(int startIndex, float[] tupleArray)
   {
      set(tupleArray[startIndex++], tupleArray[startIndex]);
   }

   /**
    * Sets this tuple's components {@code x}, {@code y} in order
    * from the given column vector starting to read from its first row index.
    * 
    * @param matrix the column vector containing the new values for this tuple's components. Not modified.
    */
   default void set(DenseMatrix64F matrix)
   {
      set(matrix.get(0, 0), matrix.get(1, 0));
   }

   /**
    * Sets this tuple's components {@code x}, {@code y} in order
    * from the given column vector starting to read from {@code startRow}.
    * 
    * @param startRow the first row index to start reading in the dense-matrix.
    * @param matrix the column vector containing the new values for this tuple's components. Not modified.
    */
   default void set(int startRow, DenseMatrix64F matrix)
   {
      set(matrix.get(startRow++, 0), matrix.get(startRow, 0));
   }

   /**
    * Sets this tuple's components {@code x}, {@code y} in order
    * from the given matrix starting to read from {@code startRow} at the column index {@code column}.
    * 
    * @param startRow the first row index to start reading in the dense-matrix.
    * @param column the column index to read in the dense-matrix.
    * @param matrix the column vector containing the new values for this tuple's components. Not modified.
    */
   default void set(int startRow, int column, DenseMatrix64F matrix)
   {
      set(matrix.get(startRow++, column), matrix.get(startRow, column));
   }

   /**
    * Sets this tuple to {@code other} and then calls {@link #absolute()}.
    * 
    * @param other the other tuple to copy the values from. Not modified.
    */
   default void setAndAbsolute(Tuple2DReadOnly<?> other)
   {
      set(Math.abs(other.getX()), Math.abs(other.getY()));
   }

   /**
    * Sets this tuple to {@code other} and then calls {@link #negate()}.
    * 
    * @param other the other tuple to copy the values from. Not modified.
    */
   default void setAndNegate(Tuple2DReadOnly<?> other)
   {
      set(-other.getX(), -other.getY());
   }

   /**
    * Sets this tuple to {@code other} and then scales it {@link #scale(double)}.
    * 
    * @param scalar the scale factor to use on this tuple.
    * @param other the other tuple to copy the values from. Not modified.
    */
   default void setAndScale(double scalar, Tuple2DReadOnly<?> other)
   {
      set(scalar * other.getX(), scalar * other.getY());
   }

   /**
    * Sets this tuple to {@code other} and then calls {@link #clipToMax(double)}.
    * 
    * @param max the maximum value for each component of this tuple.
    * @param other the other tuple to copy the values from. Not modified.
    */
   default void setAndClipToMax(double max, Tuple2DReadOnly<?> other)
   {
      set(Math.min(max, other.getX()), Math.min(max, other.getY()));
   }

   /**
    * Sets this tuple to {@code other} and then calls {@link #clipToMin(double)}.
    * 
    * @param min the minimum value for each component of this tuple.
    * @param other the other tuple to copy the values from. Not modified.
    */
   default void setAndClipToMin(double min, Tuple2DReadOnly<?> other)
   {
      set(Math.max(min, other.getX()), Math.max(min, other.getY()));
   }

   /**
    * Sets this tuple to {@code other} and then calls {@link #clipToMinMax(double, double)}.
    * 
    * @param min the minimum value for each component of this tuple.
    * @param max the maximum value for each component of this tuple.
    * @param other the other tuple to copy the values from. Not modified.
    */
   default void setAndClipToMinMax(double min, double max, Tuple2DReadOnly<?> other)
   {
      set(other);
      clipToMinMax(min, max);
   }

   /**
    * Adds the given ({@code x}, {@code y})-tuple to this tuple.
    * <p>
    * this = this + (x, y)
    * </p>
    * 
    * @param x the value to add to the x-component of this tuple.
    * @param y the value to add to the y-component of this tuple.
    */
   default void add(double x, double y)
   {
      set(getX() + x, getY() + y);
   }

   /**
    * Adds the given tuple to this tuple.
    * <p>
    * this = this + other
    * </p>
    * 
    * @param other the other tuple to add to this tuple.
    */
   default void add(Tuple2DReadOnly<?> other)
   {
      add(other.getX(), other.getY());
   }

   /**
    * Sets this tuple to the sum of the two given tuples.
    * <p>
    * this = tuple1 + tuple2
    * </p>
    * 
    * @param tuple1 the first tuple to sum. Not modified.
    * @param tuple2 the second tuple to sum. Not modified.
    */
   default void add(Tuple2DReadOnly<?> tuple1, Tuple2DReadOnly<?> tuple2)
   {
      set(tuple1.getX() + tuple2.getX(), tuple1.getY() + tuple2.getY());
   }

   /**
    * Subtracts the given ({@code x}, {@code y})-tuple to this tuple.
    * <p>
    * this = this - (x, y)
    * </p>
    * 
    * @param x the value to add to the x-component of this tuple.
    * @param y the value to add to the y-component of this tuple.
    */
   default void sub(double x, double y)
   {
      set(getX() - x, getY() - y);
   }

   /**
    * Subtracts the given tuple to this tuple.
    * <p>
    * this = this - other
    * </p>
    * 
    * @param other the other tuple to add to this tuple.
    */
   default void sub(Tuple2DReadOnly<?> tupleReadOnly)
   {
      sub(tupleReadOnly.getX(), tupleReadOnly.getY());
   }

   /**
    * Sets this tuple to the difference of the two given tuples.
    * <p>
    * this = tuple1 - tuple2
    * </p>
    * 
    * @param tuple1 the first tuple. Not modified.
    * @param tuple2 the second to subtract to {@code tuple1}. Not modified.
    */
   default void sub(Tuple2DReadOnly<?> tuple1, Tuple2DReadOnly<?> tuple2)
   {
      set(tuple1.getX() - tuple2.getX(), tuple1.getY() - tuple2.getY());
   }

   /**
    * Scales the components of this tuple by the given {@code scalar}.
    * <p>
    * this = scalar * this
    * </p>
    * 
    * @param scalar the scale factor to use.
    */
   default void scale(double scalar)
   {
      scale(scalar, scalar);
   }

   /**
    * Scales independently each component of this tuple.
    * <pre> 
    * / this.x \ = / scalarX * this.x \
    * \ this.y /   \ scalarY * this.y /
    * </pre>
    * 
    * @param scalarX the scalar factor to use on the x-component of this tuple.
    * @param scalarY the scalar factor to use on the y-component of this tuple.
    */
   default void scale(double scalarX, double scalarY)
   {
      set(scalarX * getX(), scalarY * getY());
   }

   /**
    * Scales this tuple and adds {@code other}.
    * <p>
    * this = scalar * this + other
    * </p>
    * 
    * @param scalar the scale factor to use.
    * @param other the tuple to add to this. Not modified.
    */
   default void scaleAdd(double scalar, Tuple2DReadOnly<?> other)
   {
      scale(scalar);
      add(other);
   }

   /**
    * Sets this tuple to the sum of {@code tuple1} scaled and {@code tuple2}.
    * <p>
    * this = scalar * tuple1 + tuple2
    * </p>
    * 
    * @param scalar the scale factor to use on {@code tuple1}.
    * @param tuple1 the first tuple of the sum. Not modified.
    * @param tuple2 the second tuple of the sum. Not modified.
    */
   default void scaleAdd(double scalar, Tuple2DReadOnly<?> tuple1, Tuple2DReadOnly<?> tuple2)
   {
      setAndScale(scalar, tuple1);
      add(tuple2);
   }

   /**
    * Performs a linear interpolation from this tuple to {@code other} given
    * the percentage {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * this + alpha * other
    * </p>
    * 
    * @param other the other tuple used for the interpolation. Not modified.
    * @param alpha the percentage used for the interpolation.
    * A value of 0 will result in not modifying this tuple, while a value of 1
    * is equivalent to setting this tuple to {@code other}.
    */
   default void interpolate(Tuple2DReadOnly<?> other, double alpha)
   {
      interpolate(this, other, alpha);
   }

   /**
    * Performs a linear interpolation from {@code tuple1} to {@code tuple2} given
    * the percentage {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * tuple1 + alpha * tuple2
    * </p>
    * 
    * @param tuple1 the first tuple used in the interpolation. Not modified.
    * @param tuple2 the second tuple used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation.
    * A value of 0 will result in setting this tuple to {@code tuple1}, while a
    * value of 1 is equivalent to setting this tuple to {@code tuple2}.
    */
   default void interpolate(Tuple2DReadOnly<?> tuple1, Tuple2DReadOnly<?> tuple2, double alpha)
   {
      double x = TupleTools.interpolate(tuple1.getX(), tuple2.getX(), alpha);
      double y = TupleTools.interpolate(tuple1.getY(), tuple2.getY(), alpha);
      set(x, y);
   }
}
