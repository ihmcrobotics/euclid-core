package us.ihmc.geometry.tuple2D;

import java.io.Serializable;

import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple2D.interfaces.Vector2DBasics;

public class Vector2D32 implements Serializable, Vector2DBasics<Vector2D32>
{
   private static final long serialVersionUID = 6380132073713315352L;

   private float x, y;

   public Vector2D32()
   {
      setToZero();
   }

   public Vector2D32(float x, float y)
   {
      set(x, y);
   }

   public Vector2D32(float[] vectorArray)
   {
      set(vectorArray);
   }

   public Vector2D32(Tuple2DReadOnly<?> other)
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

   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((Vector2D32) object);
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
