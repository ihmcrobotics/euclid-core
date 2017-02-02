package us.ihmc.geometry.axisAngle.interfaces;

import us.ihmc.geometry.axisAngle.AxisAngleTools;
import us.ihmc.geometry.interfaces.EpsilonComparable;
import us.ihmc.geometry.tuple.RotationVectorConversion;
import us.ihmc.geometry.tuple.interfaces.VectorBasics;

/**
 * Read-only interface for an axis-angle object.
 * <p>
 * An axis-angle is used to represent a 3D orientation by a unitary axis
 * of components (x, y, z) and an angle of rotation usually expressed in radians.
 * </p>
 * 
 * @author Sylvain
 * 
 * @param T the final type of the axis-angle used.
 */
public interface AxisAngleReadOnly<T extends AxisAngleReadOnly<T>> extends EpsilonComparable<T>
{
   /**
    * Returns the angle of this axis-angle, usually expressed in radians.
    * @return the angle.
    */
   double getAngle();

   /**
    * Returns the angle of this axis-angle, usually expressed in radians.
    * @return the angle.
    */
   default float getAngle32()
   {
      return (float) getAngle();
   }
   
   /**
    * Returns the x-component of the unitary axis of this axis-angle.
    * @return the x-component of the unitary axis.
    */
   double getX();

   /**
    * Returns the x-component of the unitary axis of this axis-angle.
    * @return the x-component of the unitary axis.
    */
   default float getX32()
   {
      return (float) getX();
   }
   
   /**
    * Returns the y-component of the unitary axis of this axis-angle.
    * @return the y-component of the unitary axis.
    */
   double getY();

   /**
    * Returns the y-component of the unitary axis of this axis-angle.
    * @return the y-component of the unitary axis.
    */
   default float getY32()
   {
      return (float) getY();
   }
   
   /**
    * Returns the z-component of the unitary axis of this axis-angle.
    * @return the z-component of the unitary axis.
    */
   double getZ();

   /**
    * Returns the z-component of the unitary axis of this axis-angle.
    * @return the z-component of the unitary axis.
    */
   default float getZ32()
   {
      return (float) getZ();
   }
   
   /**
    * Tests if this axis-angle contains a {@link Double#NaN}.
    * 
    * @return {@code true} if this axis-angle contains a {@link Double#NaN}, {@code false} otherwise.
    */
   default boolean containsNaN()
   {
      return AxisAngleTools.containsNaN(this);
   }

   /**
    * Converts and gets the orientation represented by this axis-angle
    * as a rotation vector.
    * See {@link RotationVectorConversion#convertAxisAngleToRotationVector(AxisAngleReadOnly, VectorBasics)}.
    * 
    * @param rotationVectorToPack rotation vector in which the orientation
    *  of this axis-angle is stored. Modified.
    */
   default void getRotationVector(VectorBasics rotationVectorToPack)
   {
      RotationVectorConversion.convertAxisAngleToRotationVector(this, rotationVectorToPack);
   }

   /**
    * Packs the components of this axis-angle in an array starting from its first index.
    * The components are packed in the following order: x, y, z, and angle.
    * 
    * @param axisAngleArrayToPack the array in which this axis-angle is stored. Modified.
    */
   default void get(double[] axisAngleArrayToPack)
   {
      get(axisAngleArrayToPack, 0);
   }

   /**
    * Packs the components of this axis-angle in an array starting from {@code startIndex}.
    * The components are packed in the following order: x, y, z, and angle.
    * 
    * @param axisAngleArrayToPack the array in which this axis-angle is stored. Modified.
    * @param startIndex the index in the array where the first component is stored.
    */
   default void get(double[] axisAngleArrayToPack, int startIndex)
   {
      axisAngleArrayToPack[startIndex++] = getX();
      axisAngleArrayToPack[startIndex++] = getY();
      axisAngleArrayToPack[startIndex++] = getZ();
      axisAngleArrayToPack[startIndex] = getAngle();
   }

   /**
    * Packs the components of this axis-angle in an array starting from its first index.
    * The components are packed in the following order: x, y, z, and angle.
    * 
    * @param axisAngleArrayToPack the array in which this axis-angle is stored. Modified.
    */
   default void get(float[] axisAngleArrayToPack)
   {
      get(axisAngleArrayToPack, 0);
   }

   /**
    * Packs the components of this axis-angle in an array starting from {@code startIndex}.
    * The components are packed in the following order: x, y, z, and angle.
    * 
    * @param axisAngleArrayToPack the array in which this axis-angle is stored. Modified.
    * @param startIndex the index in the array where the first component is stored.
    */
   default void get(float[] axisAngleArrayToPack, int startIndex)
   {
      axisAngleArrayToPack[startIndex++] = getX32();
      axisAngleArrayToPack[startIndex++] = getY32();
      axisAngleArrayToPack[startIndex++] = getZ32();
      axisAngleArrayToPack[startIndex] = getAngle32();
   }

   /**
    * Selects a component of this axis-angle based on {@code index}
    * and returns its value.
    * For {@code index} values of 0, 1, and 2, the corresponding components
    * are x, y, and z, respectively, while a value 3 corresponds to the angle.
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
         return getAngle();
      default:
         throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }
   
   /**
    * Selects a component of this axis-angle based on {@code index}
    * and returns its value.
    * For {@code index} values of 0, 1, and 2, the corresponding components
    * are x, y, and z, respectively, while a value 3 corresponds to the angle.
    * 
    * @param index the index of the component to get.
    * @return the value of the component.
    * @throws IndexOutOfBoundsException if {@code index} &notin; [0, 3].
    */
   default float get32(int index)
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
         return getAngle32();
      default:
         throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   /**
    * Tests on a per component basis, if this axis-angle is exactly equal to {@code other}.
    * A failing test does not necessarily mean that the two axis-angles represent two
    * different orientations.
    * 
    * @param other the other axis-angle to compare against this. Not modified.
    * @return {@code true} if the two axis-angle are exactly equal component-wise, {@code false} otherwise.
    */
   default boolean equals(T other)
   {
      try
      {
         return getX() == other.getX() && getY() == other.getY() && getZ() == other.getZ() && getAngle() == other.getAngle();
      }
      catch (NullPointerException e)
      {
         return false;
      }
   }

   /**
    * Tests on a per component basis, if this axis-angle is equal to {@code other} to an {@code epsilon}.
    * A failing test does not necessarily mean that the two axis-angles represent two
    * different orientations.
    * 
    * @param other the other axis-angle to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two axis-angle are equal component-wise, {@code false} otherwise.
    */
   @Override
   default boolean epsilonEquals(T other, double epsilon)
   {
      double diff;
      
      diff = getX() - other.getX();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;
      
      diff = getY() - other.getY();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;
      
      diff = getZ() - other.getZ();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;
      
      diff = getAngle() - other.getAngle();
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;
      
      return true;
   }
}