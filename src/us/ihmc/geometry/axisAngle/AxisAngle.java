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

public class AxisAngle implements Serializable, AxisAngleBasics, EpsilonComparable<AxisAngle>, Settable<AxisAngle>
{
   private static final long serialVersionUID = -7238256250079419416L;

   private double x, y, z;
   private double angle;

   public AxisAngle()
   {
      setToZero();
   }

   public AxisAngle(AxisAngleReadOnly other)
   {
      set(other);
   }

   public AxisAngle(double x, double y, double z, double angle)
   {
      set(x, y, z, angle);
   }

   public AxisAngle(double[] axisAngleArray)
   {
      set(axisAngleArray);
   }

   public AxisAngle(VectorReadOnly axis, double angle)
   {
      set(axis, angle);
   }

   public AxisAngle(QuaternionReadOnly quaternion)
   {
      set(quaternion);
   }

   public AxisAngle(RotationMatrixReadOnly rotationMatrix)
   {
      set(rotationMatrix);
   }

   public AxisAngle(VectorReadOnly rotationVector)
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
   public void set(AxisAngle other)
   {
      set((AxisAngleReadOnly) other); 
   }

   @Override
   public void setToNaN()
   {
      x = Double.NaN;
      y = Double.NaN;
      z = Double.NaN;
      angle = Double.NaN;
   }

   @Override
   public void setToZero()
   {
      x = 1.0;
      y = 0.0;
      z = 0.0;
      angle = 0.0;
   }

   public final void set(AxisAngleReadOnly other)
   {
      x = other.getX();
      y = other.getY();
      z = other.getZ();
      angle = other.getAngle();
   }

   @Override
   public final void set(double x, double y, double z, double angle)
   {
      this.x = x;
      this.y = y;
      this.z = z;
      this.angle = angle;
   }

   public final void set(double[] axisAngleArray)
   {
      set(axisAngleArray, 0);
   }

   public final void set(double[] axisAngleArray, int startIndex)
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

   public final void set(VectorReadOnly axis, double angle)
   {
      x = axis.getX();
      y = axis.getY();
      z = axis.getZ();
      this.angle = angle;
   }

   public void set(int index, double value)
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
      this.x = x;
   }

   @Override
   public final void setY(double y)
   {
      this.y = y;
   }

   @Override
   public final void setZ(double z)
   {
      this.z = z;
   }

   @Override
   public final void setAngle(double angle)
   {
      this.angle = angle;
   }

   public final void getRotationVector(VectorBasics rotationVectorToPack)
   {
      RotationVectorConversion.convertAxisAngleToRotationVector(this, rotationVectorToPack);
   }

   public final void get(double[] axisAngleArrayToPack)
   {
      get(axisAngleArrayToPack, 0);
   }

   public final void get(double[] axisAngleArrayToPack, int startIndex)
   {
      axisAngleArrayToPack[startIndex++] = x;
      axisAngleArrayToPack[startIndex++] = y;
      axisAngleArrayToPack[startIndex++] = z;
      axisAngleArrayToPack[startIndex] = angle;
   }

   public double get(int index)
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

   @Override
   public boolean epsilonEquals(AxisAngle other, double epsilon)
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

   public boolean equals(AxisAngle other)
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
         return equals((AxisAngle) object);
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
      bits = 31L * bits + Double.doubleToLongBits(x);
      bits = 31L * bits + Double.doubleToLongBits(y);
      bits = 31L * bits + Double.doubleToLongBits(z);
      bits = 31L * bits + Double.doubleToLongBits(angle);
      return (int) (bits ^ bits >> 32);
   }
}
