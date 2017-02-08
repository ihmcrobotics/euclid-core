package us.ihmc.geometry.tuple2D;

import java.io.Serializable;

import us.ihmc.geometry.tuple2D.interfaces.Point2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;

/**
 * A 2D point represents the 2D coordinates of a location on the XY-plane.
 * <p>
 * This version of 2D point uses double precision fields to save the value of each component.
 * It is meant for garbage free usage.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class Point2D implements Serializable, Point2DBasics<Point2D>
{
   private static final long serialVersionUID = -615943325053203074L;

   private double x, y;

   public Point2D()
   {
      setToZero();
   }

   public Point2D(double x, double y)
   {
      set(x, y);
   }

   public Point2D(double[] pointArray)
   {
      set(pointArray);
   }

   public Point2D(Tuple2DReadOnly<?> other)
   {
      set(other);
   }

   @Override
   public void setX(double x)
   {
      this.x = x;
   }

   @Override
   public void setY(double y)
   {
      this.y = y;
   }

   @Override
   public double getX()
   {
      return x;
   }

   @Override
   public double getY()
   {
      return y;
   }

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

   @Override
   public String toString()
   {
      return "(" + x + ", " + y + ")";
   }

   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = 31L * bits + Double.doubleToLongBits(x);
      bits = 31L * bits + Double.doubleToLongBits(y);
      return (int) (bits ^ bits >> 32);
   }
}
