package us.ihmc.geometry.tuple4D;

import java.io.Serializable;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.geometry.interfaces.GeometryObject;
import us.ihmc.geometry.transform.interfaces.Transform;
import us.ihmc.geometry.tuple3D.Tuple3DTools;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Tuple4DReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DReadOnly;

public class Vector4D implements Serializable, Vector4DBasics, GeometryObject<Vector4D>
{
   private static final long serialVersionUID = 3048835798807478377L;

   private double x, y, z, s;

   public Vector4D()
   {
      setToZero();
   }

   public Vector4D(double x, double y, double z, double s)
   {
      set(x, y, z, s);
   }

   public Vector4D(double[] vectorArray)
   {
      set(vectorArray);
   }

   public Vector4D(QuaternionReadOnly quaternion)
   {
      set(quaternion);
   }

   public Vector4D(Tuple4DReadOnly other)
   {
      set(other);
   }

   @Override
   public void set(Vector4D other)
   {
      x = other.x;
      y = other.y;
      z = other.z;
      s = other.s;
   }
   
   @Override
   public void set(double x, double y, double z, double s)
   {
      this.x = x;
      this.y = y;
      this.z = z;
      this.s = s;
   }

   public final void set(double[] vectorArray)
   {
      x = vectorArray[0];
      y = vectorArray[1];
      z = vectorArray[2];
      s = vectorArray[3];
   }

   public final void set(double[] vectorArray, int startIndex)
   {
      x = vectorArray[startIndex++];
      y = vectorArray[startIndex++];
      z = vectorArray[startIndex++];
      s = vectorArray[startIndex];
   }

   public void set(Tuple4DReadOnly other)
   {
      x = other.getX();
      y = other.getY();
      z = other.getZ();
      s = other.getS();
   }

   public void set(Tuple3DReadOnly tuple)
   {
      x = tuple.getX();
      y = tuple.getY();
      z = tuple.getZ();
      s = 0.0;
   }

   public final void set(DenseMatrix64F matrix)
   {
      x = matrix.get(0, 0);
      y = matrix.get(1, 0);
      z = matrix.get(2, 0);
      s = matrix.get(3, 0);
   }

   public final void set(DenseMatrix64F matrix, int startRow)
   {
      x = matrix.get(startRow++, 0);
      y = matrix.get(startRow++, 0);
      z = matrix.get(startRow++, 0);
      s = matrix.get(startRow, 0);
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
         s = value;
         break;
      default:
         throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   @Override
   public void setToNaN()
   {
      x = Double.NaN;
      y = Double.NaN;
      z = Double.NaN;
      s = Double.NaN;
   }

   @Override
   public void setToZero()
   {
      x = 0.0;
      y = 0.0;
      z = 0.0;
      s = 0.0;
   }

   @Override
   public final void negate()
   {
      x = -x;
      y = -y;
      z = -z;
      s = -s;
   }

   public final void absolute()
   {
      x = Math.abs(x);
      y = Math.abs(y);
      z = Math.abs(z);
      s = Math.abs(s);
   }

   public void normalize()
   {
      if (containsNaN())
         return;
      scale(1.0 / length());
   }

   public double dot(Vector4DReadOnly other)
   {
      return Tuple4DTools.dot(this, other);
   }

   public double lengthSquared()
   {
      return x * x + y * y + z * z + s * s;
   }

   public double length()
   {
      return Math.sqrt(lengthSquared());
   }

   public final void scale(double scalar)
   {
      x *= scalar;
      y *= scalar;
      z *= scalar;
      s *= scalar;
   }

   public void scale(double scaleX, double scaleY, double scaleZ, double scaleS)
   {
      x *= scaleX;
      y *= scaleY;
      z *= scaleZ;
      s *= scaleS;
   }

   public void interpolate(Tuple4DReadOnly tuple1, double alpha)
   {
      interpolate(this,  tuple1, alpha);
   }

   public void interpolate(Tuple4DReadOnly tuple1, Tuple4DReadOnly tuple2, double alpha)
   {
      x = Tuple3DTools.interpolate(tuple1.getX(), tuple2.getX(), alpha);
      y = Tuple3DTools.interpolate(tuple1.getY(), tuple2.getY(), alpha);
      z = Tuple3DTools.interpolate(tuple1.getZ(), tuple2.getZ(), alpha);
      s = Tuple3DTools.interpolate(tuple1.getS(), tuple2.getS(), alpha);
   }

   public void add(Tuple4DReadOnly other)
   {
      x += other.getX();
      y += other.getY();
      z += other.getZ();
      s += other.getS();
   }

   public void sub(Tuple4DReadOnly other)
   {
      x -= other.getX();
      y -= other.getY();
      z -= other.getZ();
      s -= other.getS();
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
   public void setZ(double z)
   {
      this.z = z;
   }

   @Override
   public void setS(double s)
   {
      this.s = s;
   }

   public final void get(double[] tupleArray)
   {
      tupleArray[0] = x;
      tupleArray[1] = y;
      tupleArray[2] = z;
      tupleArray[3] = s;
   }

   public final void get(double[] tupleArray, int startIndex)
   {
      tupleArray[startIndex++] = x;
      tupleArray[startIndex++] = y;
      tupleArray[startIndex++] = z;
      tupleArray[startIndex] = s;
   }

   public final void get(DenseMatrix64F tupleMatrixToPack)
   {
      tupleMatrixToPack.set(0, 0, x);
      tupleMatrixToPack.set(1, 0, y);
      tupleMatrixToPack.set(2, 0, z);
      tupleMatrixToPack.set(3, 0, s);
   }

   public final void get(DenseMatrix64F tupleMatrixToPack, int startRow)
   {
      get(tupleMatrixToPack, startRow, 0);
   }

   public final void get(DenseMatrix64F tupleMatrixToPack, int startRow, int column)
   {
      tupleMatrixToPack.set(startRow++, column, x);
      tupleMatrixToPack.set(startRow++, column, y);
      tupleMatrixToPack.set(startRow++, column, z);
      tupleMatrixToPack.set(startRow, column, s);
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
         return s;
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
   public double getS()
   {
      return s;
   }

   @Override
   public boolean containsNaN()
   {
      return Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(z) || Double.isNaN(s);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      transform.transform(this);
   }

   @Override
   public boolean epsilonEquals(Vector4D other, double epsilon)
   {
      return Tuple4DTools.epsilonEquals(this, other, epsilon);
   }

   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((Vector4D) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   public boolean equals(Vector4D other)
   {
      try
      {
         return x == other.x && y == other.y && z == other.z && s == other.s;
      }
      catch (NullPointerException e)
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return Tuple4DTools.toString(this);
   }

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
