package us.ihmc.geometry.tuple4D;

import java.io.Serializable;

import us.ihmc.geometry.GeometryBasicsIOTools;
import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.geometry.tuple2D.Vector2D32;
import us.ihmc.geometry.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;

/**
 * Class used to represent unit-quaternions which are used to represent 3D orientations.
 * <p>
 * This version of quaternion uses double precision fields to save the value of each component. It
 * is meant for garbage free usage and for situations where heap memory is limited. When memory is
 * not a constraint, the use of {@link Vector2D32} is preferable.
 * </p>
 *
 * @author Sylvain Bertrand
 *
 */
public class Quaternion32 implements Serializable, QuaternionBasics<Quaternion32>
{
   private static final long serialVersionUID = -3523313039213464150L;

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
    * @param pointArray the array containing this vector's components. Not modified.
    */
   public Quaternion32(float[] quaternionArray)
   {
      set(quaternionArray);
   }

   /**
    * Creates a new quaternion and initializes it to {@code other}
    *
    * @param other the quaternion to copy the components from. Not modified.
    */
   public Quaternion32(QuaternionReadOnly<?> other)
   {
      set(other);
   }

   /**
    * Creates a new quaternion and initializes such that it represents the same orientation as the
    * given {@code rotationMatrix}.
    *
    * @param rotationMatrix the rotation matrix to initialize this quaternion. Not modified.
    */
   public Quaternion32(RotationMatrixReadOnly<?> rotationMatrix)
   {
      set(rotationMatrix);
   }

   /**
    * Creates a new quaternion and initializes such that it represents the same orientation as the
    * given {@code axisAngle}.
    *
    * @param axisAngle the axis-angle to initialize this quaternion. Not modified.
    */
   public Quaternion32(AxisAngleReadOnly axisAngle)
   {
      set(axisAngle);
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
      set(rotationVector);
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
    * returns {@link #equals(Quaternion32)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((Quaternion32) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Provides a {@code String} representation of this quaternion converted to yaw-pitch-roll angles
    * as follows: yaw-pitch-roll: (yaw, pitch, roll).
    *
    * @return
    */
   public String toStringAsYawPitchRoll()
   {
      return "yaw-pitch-roll: (" + getYaw() + ", " + getPitch() + ", " + getRoll() + ")";
   }

   /**
    * Provides a {@code String} representation of this quaternion as follows: (x, y, z, s).
    *
    * @return the {@code String} representing this quaternion.
    */
   @Override
   public String toString()
   {
      return GeometryBasicsIOTools.getTuple4DString(this);
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
      bits = 31L * bits + Float.floatToIntBits(x);
      bits = 31L * bits + Float.floatToIntBits(y);
      bits = 31L * bits + Float.floatToIntBits(z);
      bits = 31L * bits + Float.floatToIntBits(s);
      return (int) (bits ^ bits >> 32);
   }
}
