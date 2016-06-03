package us.ihmc.geometry.axisAngle;

import java.io.Serializable;

import us.ihmc.geometry.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.geometry.interfaces.EpsilonComparable;
import us.ihmc.geometry.interfaces.Settable;
import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.geometry.tuple.RotationVectorConversion;
import us.ihmc.geometry.tuple.interfaces.VectorBasics;
import us.ihmc.geometry.tuple.interfaces.VectorReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;

public class AxisAngle32 implements Serializable, AxisAngleBasics, EpsilonComparable<AxisAngle32>, Settable<AxisAngle32>
{
   private static final long serialVersionUID = 3750855120438442145L;

   private float x, y, z;
   private float angle;

   public AxisAngle32()
   {
      setToZero();
   }

   public AxisAngle32(AxisAngleReadOnly other)
   {
      set(other);
   }

   public AxisAngle32(float x, float y, float z, float angle)
   {
      set(x, y, z, angle);
   }

   public AxisAngle32(float[] axisAngleArray)
   {
      set(axisAngleArray);
   }

   public AxisAngle32(VectorReadOnly axis, float angle)
   {
      set(axis, angle);
   }

   public AxisAngle32(QuaternionReadOnly quaternion)
   {
      set(quaternion);
   }

   public AxisAngle32(RotationMatrixReadOnly rotationMatrix)
   {
      set(rotationMatrix);
   }

   public AxisAngle32(VectorReadOnly rotationVector)
   {
      set(rotationVector);
   }

   @Override
   public boolean containsNaN()
   {
      return AxisAngleTools.containsNaN(this);
   }

   public void negate()
   {
      x = -x;
      y = -y;
      z = -z;
      angle = -angle;
   }

   @Override
   public void set(AxisAngle32 other)
   {
      set((AxisAngleReadOnly) other);
   }

   @Override
   public void setToNaN()
   {
      x = Float.NaN;
      y = Float.NaN;
      z = Float.NaN;
      angle = Float.NaN;
   }

   @Override
   public void setToZero()
   {
      x = 1.0f;
      y = 0.0f;
      z = 0.0f;
      angle = 0.0f;
   }

   public final void set(AxisAngleReadOnly other)
   {
      x = (float) other.getX();
      y = (float) other.getY();
      z = (float) other.getZ();
      angle = (float) other.getAngle();
   }

   @Override
   public final void set(double x, double y, double z, double angle)
   {
      this.x = (float) x;
      this.y = (float) y;
      this.z = (float) z;
      this.angle = (float) angle;
   }

   public final void set(float x, float y, float z, float angle)
   {
      this.x = x;
      this.y = y;
      this.z = z;
      this.angle = angle;
   }

   public final void set(float[] axisAngleArray)
   {
      set(axisAngleArray, 0);
   }

   public final void set(float[] axisAngleArray, int startIndex)
   {
      x = axisAngleArray[startIndex++];
      y = axisAngleArray[startIndex++];
      z = axisAngleArray[startIndex++];
      angle = axisAngleArray[startIndex];
   }

   public final void set(QuaternionReadOnly quaternion)
   {
      AxisAngleConversion.convertQuaternionToAxisAngle(quaternion, this);
   }

   public final void set(RotationMatrixReadOnly rotationMatrix)
   {
      AxisAngleConversion.convertMatrixToAxisAngle(rotationMatrix, this);
   }

   public final void set(VectorReadOnly rotationVector)
   {
      AxisAngleConversion.convertRotationVectorToAxisAngle(rotationVector, this);
   }

   public final void set(VectorReadOnly axis, float angle)
   {
      x = (float) axis.getX();
      y = (float) axis.getY();
      z = (float) axis.getZ();
      this.angle = angle;
   }

   public void set(int index, float value)
   {
      switch (index)
      {
      case 0:
         x = value;
         break;
      case 1:
         y = value;
         break;
      case 2:
         z = value;
         break;
      case 3:
         angle = value;
         break;
      default:
         throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   @Override
   public final void setX(double x)
   {
      this.x = (float) x;
   }

   public final void setX(float x)
   {
      this.x = x;
   }

   @Override
   public final void setY(double y)
   {
      this.y = (float) y;
   }

   public final void setY(float y)
   {
      this.y = y;
   }

   @Override
   public final void setZ(double z)
   {
      this.z = (float) z;
   }

   public final void setZ(float z)
   {
      this.z = z;
   }

   @Override
   public final void setAngle(double angle)
   {
      this.angle = (float) angle;
   }

   public final void setAngle(float angle)
   {
      this.angle = angle;
   }

   public final void getRotationVector(VectorBasics rotationVectorToPack)
   {
      RotationVectorConversion.convertAxisAngleToRotationVector(this, rotationVectorToPack);
   }

   public final void get(float[] axisAngleArrayToPack)
   {
      get(axisAngleArrayToPack, 0);
   }

   public final void get(float[] axisAngleArrayToPack, int startIndex)
   {
      axisAngleArrayToPack[startIndex++] = x;
      axisAngleArrayToPack[startIndex++] = y;
      axisAngleArrayToPack[startIndex++] = z;
      axisAngleArrayToPack[startIndex] = angle;
   }

   public float get(int index)
   {
      switch (index)
      {
      case 0:
         return x;
      case 1:
         return y;
      case 2:
         return z;
      case 3:
         return angle;
      default:
         throw new IndexOutOfBoundsException(Integer.toString(index));
      }
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
   public double getZ()
   {
      return z;
   }

   @Override
   public final double getAngle()
   {
      return angle;
   }

   public final float getAngle32()
   {
      return angle;
   }

   public final float getX32()
   {
      return x;
   }

   public final float getY32()
   {
      return y;
   }

   public final float getZ32()
   {
      return z;
   }

   @Override
   public boolean epsilonEquals(AxisAngle32 other, double epsilon)
   {
      double diff;

      diff = x - other.x;
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      diff = y - other.y;
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      diff = z - other.z;
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      diff = angle - other.angle;
      if (Double.isNaN(diff) || Math.abs(diff) > epsilon)
         return false;

      return true;
   }

   public boolean equals(AxisAngle32 other)
   {
      try
      {
         return x == other.x && y == other.y && z == other.z && angle == other.angle;
      }
      catch (NullPointerException e)
      {
         return false;
      }
   }

   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((AxisAngle32) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return "(" + x + ", " + y + ", " + z + ", " + angle + ")";
   }

   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = 31L * bits + Float.floatToIntBits(x);
      bits = 31L * bits + Float.floatToIntBits(y);
      bits = 31L * bits + Float.floatToIntBits(z);
      bits = 31L * bits + Float.floatToIntBits(angle);
      return (int) (bits ^ bits >> 32);
   }
}
