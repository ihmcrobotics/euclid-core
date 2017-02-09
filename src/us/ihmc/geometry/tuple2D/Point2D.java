package us.ihmc.geometry.tuple2D;

import java.io.Serializable;

import us.ihmc.geometry.tuple2D.interfaces.Point2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;

/**
 * A 2D point represents the 2D coordinates of a location on the XY-plane.
 * <p>
 * This version of 2D point uses double precision fields to save the value of each component. It is
 * meant for garbage free usage.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class Point2D implements Serializable, Point2DBasics<Point2D>
{
   private static final long serialVersionUID = -615943325053203074L;

   /** The x-coordinate. */
   private double x;
   /** The y-coordinate. */
   private double y;

   /**
    * Creates a new point and initializes it coordinates to zero.
    */
   public Point2D()
   {
      setToZero();
   }

   /**
    * Creates a new point and initializes it with the given coordinates.
    * 
    * @param x the x-coordinate.
    * @param y the y-coordinate.
    */
   public Point2D(double x, double y)
   {
      set(x, y);
   }

   /**
    * Creates a new point and initializes its component {@code x}, {@code y} in order from the given
    * array.
    * 
    * @param pointArray the array containing this point's coordinates. Not modified.
    */
   public Point2D(double[] pointArray)
   {
      set(pointArray);
   }

   /**
    * Creates a new point and initializes it to {@code other}.
    * 
    * @param other the tuple to copy the coordinates from. Not modified.
    */
   public Point2D(Tuple2DReadOnly<?> other)
   {
      set(other);
   }

   /**
    * Sets the x-coordinate of this point.
    * 
    * @param x the x-coordinate.
    */
   @Override
   public void setX(double x)
   {
      this.x = x;
   }

   /**
    * Sets the y-coordinate of this point.
    * 
    * @param y the y-coordinate.
    */
   @Override
   public void setY(double y)
   {
      this.y = y;
   }

   /**
    * Returns the value of the x-coordinate of this point.
    * 
    * @return the x-coordinate's value.
    */
   @Override
   public double getX()
   {
      return x;
   }

   /**
    * Returns the value of the y-coordinate of this point.
    * 
    * @return the y-coordinate's value.
    */
   @Override
   public double getY()
   {
      return y;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method
    * returns {@link #equals(Point2D)}, it returns {@code false} otherwise.
    * 
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((Point2D) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Provides a {@code String} representation of this point 2D as follows: (x, y).
    * 
    * @return the {@code String} representing this point 2D.
    */
   @Override
   public String toString()
   {
      return "(" + x + ", " + y + ")";
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this point 2D.
    * 
    * @return the hash code value for this point 2D.
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
