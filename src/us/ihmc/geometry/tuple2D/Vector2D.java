package us.ihmc.geometry.tuple2D;

import java.io.Serializable;

import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple2D.interfaces.Vector2DBasics;

/**
 * A 2D vector represents a physical quantity with a magnitude and a direction in the XY-plane.
 * For instance, it can be used to represent a 2D velocity, force, or translation from one 2D point to another.
 * <p>
 * This version of 2D vector uses double precision fields to save the value of each component.
 * It is meant for garbage free usage.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class Vector2D implements Serializable, Vector2DBasics<Vector2D>
{
   private static final long serialVersionUID = -1422872858238666884L;

   /** The x-component. */
   private double x;
   /** The y-component. */
   private double y;

   /**
    * Creates a new vector and initializes it components to zero.
    */
   public Vector2D()
   {
      setToZero();
   }

   /**
    * Creates a new vector and initializes it with the given components.
    * 
    * @param x the x-component.
    * @param y the y-component.
    */
   public Vector2D(double x, double y)
   {
      set(x, y);
   }

   /**
    * Creates a new vector and initializes it from the given array.
    * 
    * @param pointArray the array containing this vector's components. Not modified.
    */
   public Vector2D(double[] vectorArray)
   {
      set(vectorArray);
   }

   /**
    * Creates a new vector and initializes it to {@code other}
    * 
    * @param other the tuple to copy the components from. Not modified.
    */
   public Vector2D(Tuple2DReadOnly<?> other)
   {
      set(other);
   }

   /**
    * Sets the x-component of this vector.
    * 
    * @param x the x-component.
    */
   @Override
   public void setX(double x)
   {
      this.x = x;
   }

   /**
    * Sets the y-component of this vector.
    * 
    * @param y the y-component.
    */
   @Override
   public void setY(double y)
   {
      this.y = y;
   }

   /**
    * Returns the value of the x-component of this vector.
    * 
    * @return the x-component's value.
    */
   @Override
   public double getX()
   {
      return x;
   }

   /**
    * Returns the value of the y-component of this vector.
    * 
    * @return the y-component's value.
    */
   @Override
   public double getY()
   {
      return y;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this,
    * in which case the method returns {@link #equals(Vector2D)}, it returns {@code false}
    * otherwise.
    * 
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((Vector2D) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Provides a {@code String} representation of this vector 2D as follows:
    * (x, y).
    * 
    * @return the {@code String} representing this vector 2D.
    */
   @Override
   public String toString()
   {
      return "(" + x + ", " + y + ")";
   }

   /**
    * Calculates and returns a hash code value from the value
    * of each component of this vector 2D.
    * 
    * @return the hash code value for this vector 2D.
    */
   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = 31L * bits + Double.doubleToLongBits(x);
      bits = 31L * bits + Double.doubleToLongBits(y);
      return (int) (bits ^ bits >> 32);
   }
}
