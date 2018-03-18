package us.ihmc.euclid.tuple4D;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple2D.Vector2D32;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;

/**
 * Class used to represent unit-quaternions which are used to represent 3D orientations.
 * <p>
 * This version of quaternion uses double precision fields to save the value of each component. It
 * is meant for garbage free usage and for situations where heap memory is limited. When memory is
 * not a constraint, the use of {@link Vector2D32} is preferable.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class Quaternion32 implements QuaternionBasics, GeometryObject<Quaternion32>
{
   /** The x-component. */
   private float x;
   /** The y-component. */
   private float y;
   /** The z-component. */
   private float z;
   /** The s-component. */
   private float s;

   /**
    * Creates a new quaternion and initializes it to the neutral quaternion which represents a
    * 'zero' rotation.
    */
   public Quaternion32()
   {
      setToZero();
   }

   /**
    * Creates a new quaternion and initializes it with the given components.
    * <p>
    * The quaternion is immediately normalized.
    * </p>
    *
    * @param x the x-component.
    * @param y the y-component.
    * @param z the z-component.
    * @param s the s-component.
    */
   public Quaternion32(float x, float y, float z, float s)
   {
      set(x, y, z, s);
   }

   /**
    * Creates a new quaternion and initializes its component {@code x}, {@code y}, {@code z},
    * {@code s} in order from the given array.
    * <p>
    * The quaternion is immediately normalized.
    * </p>
    *
    * @param quaternionArray the array containing this quaternion's components. Not modified.
    */
   public Quaternion32(float[] quaternionArray)
   {
      set(quaternionArray);
   }

   /**
    * Creates a new quaternion and initializes such that it represents the same orientation as the
    * given one.
    *
    * @param orientation the orientation used to initialize this quaternion. Not modified.
    */
   public Quaternion32(Orientation3DReadOnly orientation)
   {
      set(orientation);
   }

   /**
    * Creates a new quaternion and initializes such that it represents the same orientation as the
    * given {@code rotationVector}.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param rotationVector the rotation vector to initialize this quaternion. Not modified.
    */
   public Quaternion32(Vector3DReadOnly rotationVector)
   {
      setRotationVector(rotationVector);
   }

   /**
    * Creates and new quaternion and initializes such that it represents the same orientation as the
    * given yaw-pitch-roll {@code yaw}, {@code pitch}, and {@code roll}.
    *
    * @param yaw the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll the angle to rotate about the x-axis.
    */
   public Quaternion32(double yaw, double pitch, double roll)
   {
      setYawPitchRoll(yaw, pitch, roll);
   }

   /**
    * Sets this quaternion to {@code other}.
    *
    * @param other the other quaternion to copy the values from. Not modified.
    */
   @Override
   public void set(Quaternion32 other)
   {
      QuaternionBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setUnsafe(double qx, double qy, double qz, double qs)
   {
      x = (float) qx;
      y = (float) qy;
      z = (float) qz;
      s = (float) qs;
   }

   /** {@inheritDoc} */
   @Override
   public double getX()
   {
      return x;
   }

   /** {@inheritDoc} */
   @Override
   public double getY()
   {
      return y;
   }

   /** {@inheritDoc} */
   @Override
   public double getZ()
   {
      return z;
   }

   /** {@inheritDoc} */
   @Override
   public double getS()
   {
      return s;
   }

   /** {@inheritDoc} */
   @Override
   public float getX32()
   {
      return x;
   }

   /** {@inheritDoc} */
   @Override
   public float getY32()
   {
      return y;
   }

   /** {@inheritDoc} */
   @Override
   public float getZ32()
   {
      return z;
   }

   /** {@inheritDoc} */
   @Override
   public float getS32()
   {
      return s;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method
    * returns {@link #equals(Tuple4DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((Tuple4DReadOnly) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Tests on a per component basis if this quaternion is equal to the given {@code other} to an
    * {@code epsilon}.
    *
    * @param other the other quaternion to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two tuples are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(Quaternion32 other, double epsilon)
   {
      return QuaternionBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests if {@code this} and {@code other} represent the same orientation to an {@code epsilon}.
    * <p>
    * Two quaternions are considered geometrically equal if the magnitude of their difference is
    * less than or equal to {@code epsilon}.
    * </p>
    * <p>
    * Note that two quaternions of opposite sign are considered equal, such that the two quaternions
    * {@code q1 = (x, y, z, s)} and {@code q2 = (-x, -y, -z, -s)} are considered geometrically
    * equal.
    * </p>
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * {@code this.epsilonEquals(other, epsilon)} and vice versa.
    * </p>
    *
    * @param other the other quaternion to compare against this. Not modified.
    * @param epsilon the maximum angle of the difference quaternion can be for the two quaternions
    *           to be considered equal.
    * @return {@code true} if the two quaternions represent the same geometry, {@code false}
    *         otherwise.
    */
   @Override
   public boolean geometricallyEquals(Quaternion32 other, double epsilon)
   {
      return QuaternionBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Provides a {@code String} representation of this quaternion as follows: (x, y, z, s).
    *
    * @return the {@code String} representing this quaternion.
    */
   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getTuple4DString(this);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this quaternion.
    *
    * @return the hash code value for this quaternion.
    */
   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = EuclidHashCodeTools.addToHashCode(bits, x);
      bits = EuclidHashCodeTools.addToHashCode(bits, y);
      bits = EuclidHashCodeTools.addToHashCode(bits, z);
      bits = EuclidHashCodeTools.addToHashCode(bits, s);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }
}
