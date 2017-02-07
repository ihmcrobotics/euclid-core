package us.ihmc.geometry.tuple2D;

import java.io.Serializable;

import us.ihmc.geometry.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;

public abstract class Tuple2D<T extends Tuple2D<T>> implements Serializable, Tuple2DBasics<T>
{
   private final static long serialVersionUID = -1825643561716102256L;

   private double x, y;

   public Tuple2D()
   {
      setToZero();
   }

   public Tuple2D(double x, double y)
   {
      set(x, y);
   }

   public Tuple2D(double[] tupleArray)
   {
      set(tupleArray);
   }

   public Tuple2D(Tuple2DReadOnly<?> other)
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

   @SuppressWarnings("unchecked")
   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((T) object);
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
