package us.ihmc.geometry.tuple4D;

import java.io.Serializable;

import us.ihmc.geometry.interfaces.GeometryObject;
import us.ihmc.geometry.tools.EuclidCoreIOTools;
import us.ihmc.geometry.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.geometry.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Tuple4DReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DBasics;

/**
 * Class used to represent generic quaternions.
 * <p>
 * This version of 4D vector uses double precision fields to save the value of each component. It is
 * meant for garbage free usage.
 * </p>
 *
 *
 * @author Sylvain Bertrand
 *
 */
public class Vector4D implements Serializable, Vector4DBasics, GeometryObject<Vector4D>
{
   private static final long serialVersionUID = 3048835798807478377L;

   /** The x-component. */
   private double x;
   /** The y-component. */
   private double y;
   /** The z-component. */
   private double z;
   /** The s-component. */
   private double s;

   /**
    * Creates a new vector and initializes it components to zero.
    */
   public Vector4D()
   {
      setToZero();
   }

   /**
    * Creates a new vector and initializes it with the given components.
    *
    * @param x the x-component.
    * @param y the y-component.
    * @param z the z-component.
    * @param s the s-component.
    */
   public Vector4D(double x, double y, double z, double s)
   {
      set(x, y, z, s);
   }

   /**
    * Creates a new vector and initializes its component {@code x}, {@code y}, {@code z}, {@code s}
    * in order from the given array.
    *
    * @param pointArray the array containing this vector's components. Not modified.
    */
   public Vector4D(double[] vectorArray)
   {
      set(vectorArray);
   }

   /**
    * Creates a new vector and initializes it to {@code other}.
    *
    * @param other the tuple to copy the components from. Not modified.
    */
   public Vector4D(Tuple4DReadOnly other)
   {
      set(other);
   }

   /**
    * Creates a new vector and initializes it to represent the given 3D vector
    * <p>
    * this.xyz = vector3D<br>
    * this.s = 0.0
    * </p>
    *
    * @param vector3D the 3D vector used to set this 4D vector. Not modified.
    */
   public Vector4D(Vector3DReadOnly vector3D)
   {
      set(vector3D);
   }

   /**
    * Creates a new vector and initializes it to represent the given 3D point
    * <p>
    * this.xyz = point3D<br>
    * this.s = 1.0
    * </p>
    *
    * @param point3D the 3D point used to set this 4D vector. Not modified.
    */
   public Vector4D(Point3DReadOnly point3D)
   {
      set(point3D);
   }

   /**
    * Sets this vector to {@code other}.
    *
    * @param other the other vector to copy the values from. Not modified.
    */
   @Override
   public void set(Vector4D other)
   {
      Vector4DBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setX(double x)
   {
      this.x = x;
   }

   /** {@inheritDoc} */
   @Override
   public void setY(double y)
   {
      this.y = y;
   }

   /** {@inheritDoc} */
   @Override
   public void setZ(double z)
   {
      this.z = z;
   }

   /** {@inheritDoc} */
   @Override
   public void setS(double s)
   {
      this.s = s;
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
    * Tests on a per component basis if this vector is equal to the given {@code other} to an
    * {@code epsilon}.
    *
    * @param other the other vector to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two tuples are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(Vector4D other, double epsilon)
   {
      return Vector4DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Provides a {@code String} representation of this 4D vector as follows: (x, y, z, s).
    *
    * @return the {@code String} representing this 4D vector.
    */
   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getTuple4DString(this);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this 4D vector.
    *
    * @return the hash code value for this 4D vector.
    */
   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = 31L * bits + Double.doubleToLongBits(x);
      bits = 31L * bits + Double.doubleToLongBits(y);
      bits = 31L * bits + Double.doubleToLongBits(z);
      bits = 31L * bits + Double.doubleToLongBits(s);
      return (int) (bits ^ bits >> 32);
   }
}
