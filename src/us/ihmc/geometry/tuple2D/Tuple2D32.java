package us.ihmc.geometry.tuple2D;

import java.io.Serializable;

import us.ihmc.geometry.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;

public abstract class Tuple2D32<T extends Tuple2D32<T>> implements Serializable, Tuple2DBasics<T>
{
   private static final long serialVersionUID = 8630707913840584158L;

   private float x, y;

   public Tuple2D32()
   {
      setToZero();
   }

   public Tuple2D32(float x, float y)
   {
      set(x, y);
   }

   public Tuple2D32(float[] tupleArray)
   {
      set(tupleArray);
   }

   public Tuple2D32(Tuple2DReadOnly<?> other)
   {
      set(other);
   }

   @Override
   public void setX(double x)
   {
      this.x = (float) x;
   }

   @Override
   public void setY(double y)
   {
      this.y = (float) y;
   }

   public void setX(float x)
   {
      this.x = x;
   }

   public void setY(float y)
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

   public float getX32()
   {
      return x;
   }

   public float getY32()
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
      bits = 31L * bits + Float.floatToIntBits(x);
      bits = 31L * bits + Float.floatToIntBits(y);
      return (int) (bits ^ bits >> 32);
   }
}
