package us.ihmc.geometry.axisAngle.interfaces;

import us.ihmc.geometry.axisAngle.AxisAngleConversion;
import us.ihmc.geometry.interfaces.Settable;
import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.geometry.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;

/**
 * Write and read interface for an axis-angle object.
 * <p>
 * An axis-angle is used to represent a 3D orientation by a unitary axis
 * of components (x, y, z) and an angle of rotation usually expressed in radians.
 * </p>
 * 
 * @author Sylvain
 * 
 * @param T the final type of the axis-angle used.
 */
public interface AxisAngleBasics<T extends AxisAngleBasics<T>> extends AxisAngleReadOnly<T>, Settable<T>
{
   /**
    * Sets a new angle to this axis-angle.
    *
    * @param angle the new angle.
    */
   void setAngle(double angle);

   /**
    * Sets a new x-component for the axis of this axis-angle.
    *
    * @param x the new axis x-component.
    */
   void setX(double x);

   /**
    * Sets a new y-component for the axis of this axis-angle.
    *
    * @param y the new axis y-component.
    */
   void setY(double y);

   /**
    * Sets a new z-component for the axis of this axis-angle.
    *
    * @param z the new axis z-component.
    */
   void setZ(double z);

   /**
    * Sets the components of this axis-angle to represent a "zero" rotation.
    * After calling the axis is equal to (1, 0, 0) and the angle to 0.
    */
   @Override
   default void setToZero()
   {
      set(1.0, 0.0, 0.0, 0.0);
   }

   /**
    * Sets the components of this axis-angle to {@link Double#NaN}.
    */
   @Override
   default void setToNaN()
   {
      set(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
   }

   /**
    * Tests if this axis-angle contains a {@link Double#NaN}.
    * 
    * @return {@code true} if this axis-angle contains a {@link Double#NaN}, {@code false} otherwise.
    */
   @Override
   default boolean containsNaN()
   {
      return AxisAngleReadOnly.super.containsNaN();
   }

   /**
    * Negates each component of this axis-angle.
    */
   default void negate()
   {
      set(-getX(), -getY(), -getZ(), -getAngle());
   }

   /**
    * Sets this axis-angle to represent a new rotation of axis ({@code x}, {@code y}, {@code z})
    * and angle of {@code angle}.
    *
    * @param x x-component of the new axis.
    * @param y y-component of the new axis.
    * @param z z-component of the new axis.
    * @param angle the new angle.
    */
   default void set(double x, double y, double z, double angle)
   {
      setAngle(angle);
      setX(x);
      setY(y);
      setZ(z);
   }

   /**
    * Sets the axis and the angle of this axis-angle.
    * 
    * @param axis the new axis. Not modified.
    * @param angle the new angle.
    */
   default void set(Vector3DReadOnly<?> axis, double angle)
   {
      set(axis.getX(), axis.getY(), axis.getZ(), angle);
   }

   /**
    * Sets this axis-angle to the same value as the given axis-angle {@code other}.
    * 
    * @param other the other axis-angle. Not modified.
    */
   default void set(T other)
   {
      set((AxisAngleReadOnly<T>) other);
   }

   /**
    * Sets this axis-angle to the same value as the given axis-angle {@code other}.
    * 
    * @param other the other axis-angle. Not modified.
    */
   default void set(AxisAngleReadOnly<?> other)
   {
      set(other.getX(), other.getY(), other.getZ(), other.getAngle());
   }

   /**
    * Copies the values in the given array into this axis-angle as follows:
    * <ul>
    *    <li> {@code this.setX(axisAngleArray[0]);}
    *    <li> {@code this.setY(axisAngleArray[1]);}
    *    <li> {@code this.setZ(axisAngleArray[2]);}
    *    <li> {@code this.setAngle(axisAngleArray[3]);}
    * </ul>
    * 
    * @param axisAngleArray the array containing the new values for this axis-angle. Not modified.
    */
   default void set(double[] axisAngleArray)
   {
      set(0, axisAngleArray);
   }

   /**
    * Copies the values in the given array into this axis-angle as follows:
    * <ul>
    *    <li> {@code this.setX(axisAngleArray[startIndex + 0]);}
    *    <li> {@code this.setY(axisAngleArray[startIndex + 1]);}
    *    <li> {@code this.setZ(axisAngleArray[startIndex + 2]);}
    *    <li> {@code this.setAngle(axisAngleArray[startIndex + 3]);}
    * </ul>
    * @param startIndex the first index to start reading from in the array.
    * @param axisAngleArray the array containing the new values for this axis-angle. Not modified.
    */
   default void set(int startIndex, double[] axisAngleArray)
   {
      setX(axisAngleArray[startIndex++]);
      setY(axisAngleArray[startIndex++]);
      setZ(axisAngleArray[startIndex++]);
      setAngle(axisAngleArray[startIndex]);
   }

   /**
    * Copies the values in the given array into this axis-angle as follows:
    * <ul>
    *    <li> {@code this.setX(axisAngleArray[0]);}
    *    <li> {@code this.setY(axisAngleArray[1]);}
    *    <li> {@code this.setZ(axisAngleArray[2]);}
    *    <li> {@code this.setAngle(axisAngleArray[3]);}
    * </ul>
    * 
    * @param axisAngleArray the array containing the new values for this axis-angle. Not modified.
    */
   default void set(float[] axisAngleArray)
   {
      set(0, axisAngleArray);
   }

   /**
    * Copies the values in the given array into this axis-angle as follows:
    * <ul>
    *    <li> {@code this.setX(axisAngleArray[startIndex + 0]);}
    *    <li> {@code this.setY(axisAngleArray[startIndex + 1]);}
    *    <li> {@code this.setZ(axisAngleArray[startIndex + 2]);}
    *    <li> {@code this.setAngle(axisAngleArray[startIndex + 3]);}
    * </ul>
    * @param startIndex the first index to start reading from in the array.
    * @param axisAngleArray the array containing the new values for this axis-angle. Not modified.
    */
   default void set(int startIndex, float[] axisAngleArray)
   {
      setX(axisAngleArray[startIndex++]);
      setY(axisAngleArray[startIndex++]);
      setZ(axisAngleArray[startIndex++]);
      setAngle(axisAngleArray[startIndex]);
   }

   /**
    * Sets the components of this axis-angle such that it represents the same
    * orientation as the given {@code quaternion}.
    * See {@link AxisAngleConversion#convertQuaternionToAxisAngle(QuaternionReadOnly, AxisAngleBasics)}.
    * 
    * @param quaternion the quaternion to convert. Not modified.
    */
   default void set(QuaternionReadOnly<?> quaternion)
   {
      AxisAngleConversion.convertQuaternionToAxisAngle(quaternion, this);
   }

   /**
    * Sets the components of this axis-angle such that it represents the same
    * orientation as the given {@code rotationMatrix}.
    * See {@link AxisAngleConversion#convertMatrixToAxisAngle(RotationMatrixReadOnly, AxisAngleBasics)}.
    * 
    * @param rotationMatrix the rotation matrix to convert. Not modified.
    */
   default void set(RotationMatrixReadOnly<?> rotationMatrix)
   {
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, this);
   }

   /**
    * Sets the components of this axis-angle such that it represents the same
    * orientation as the given {@code rotationVector}.
    * See {@link AxisAngleConversion#convertRotationVectorToAxisAngle(Vector3DReadOnly, AxisAngleBasics)}.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    * 
    * @param rotationVector the rotation vector to convert. Not modified.
    */
   default void set(Vector3DReadOnly<?> rotationVector)
   {
      AxisAngleConversion.convertRotationVectorToAxisAngle(rotationVector, this);
   }

   /**
    * Sets the components of this axis-angle such that it represents the same
    * orientation as the given yaw-pitch-roll angles.
    * See {@link AxisAngleConversion#convertYawPitchRollToAxisAngle(double[], AxisAngleBasics)}.
    * 
    * @param yawPitchRoll array containing the yaw, pitch, and roll angles. Not modified.
    */
   default void setYawPitchRoll(double[] yawPitchRoll)
   {
      AxisAngleConversion.convertYawPitchRollToAxisAngle(yawPitchRoll, this);
   }

   /**
    * Sets the components of this axis-angle such that it represents the same
    * orientation as the given yaw-pitch-roll angles.
    * See {@link AxisAngleConversion#convertYawPitchRollToAxisAngle(double, double, double, AxisAngleBasics)}.
    * 
    * @param yaw the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll the angle to rotate about the x-axis.
    */
   default void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      AxisAngleConversion.convertYawPitchRollToAxisAngle(yaw, pitch, roll, this);
   }

   /**
    * Selects a component of this axis-angle based on {@code index}
    * and sets it to {@code value}.
    * <p>
    * For {@code index} values of 0, 1, and 2, the corresponding components
    * are x, y, and z, respectively, while 3 corresponds to the angle.
    * </p>
    * 
    * @param index the index of the component to set.
    * @param value the new value of the selected component.
    * @throws IndexOutOfBoundsException if {@code index} &notin; [0, 3].
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
      case 2:
         setZ(value);
         break;
      case 3:
         setAngle(value);
         break;
      default:
         throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }
}