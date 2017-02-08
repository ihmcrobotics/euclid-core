package us.ihmc.geometry.tuple2D;

import java.io.Serializable;

import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple2D.interfaces.Vector2DBasics;

public class Vector2D implements Serializable, Vector2DBasics<Vector2D>
{
   private static final long serialVersionUID = -1422872858238666884L;

   private double x, y;

   public Vector2D()
   {
      setToZero();
   }

   public Vector2D(double x, double y)
   {
      set(x, y);
   }

   public Vector2D(double[] vectorArray)
   {
      set(vectorArray);
   }

   public Vector2D(Tuple2DReadOnly<?> other)
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
         return equals((Vector2D) object);
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
