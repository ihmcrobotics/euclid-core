package us.ihmc.geometry.tuple4D;

import java.io.Serializable;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.geometry.axisAngle.AxisAngleConversion;
import us.ihmc.geometry.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.geometry.interfaces.GeometryObject;
import us.ihmc.geometry.matrix.Matrix3D;
import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.geometry.transform.interfaces.Transform;
import us.ihmc.geometry.tuple.RotationVectorConversion;
import us.ihmc.geometry.tuple.interfaces.TupleBasics;
import us.ihmc.geometry.tuple.interfaces.TupleReadOnly;
import us.ihmc.geometry.tuple.interfaces.VectorBasics;
import us.ihmc.geometry.tuple.interfaces.VectorReadOnly;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Tuple4DReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DReadOnly;
import us.ihmc.geometry.yawPitchRoll.YawPitchRollConversion;

public class Quaternion implements Serializable, QuaternionBasics, GeometryObject<Quaternion>
{
   private static final long serialVersionUID = -3523313039213464150L;

   private double x, y, z, s;

   public Quaternion()
   {
      setToZero();
   }

   public Quaternion(double x, double y, double z, double s)
   {
      set(x, y, z, s);
   }

   public Quaternion(double[] quaternionArray)
   {
      set(quaternionArray);
   }

   public Quaternion(QuaternionReadOnly other)
   {
      set(other);
   }

   public Quaternion(RotationMatrixReadOnly rotationMatrix)
   {
      set(rotationMatrix);
   }

   public Quaternion(AxisAngleReadOnly<?> axisAngle)
   {
      set(axisAngle);
   }

   public Quaternion(VectorReadOnly rotationVector)
   {
      set(rotationVector);
   }

   public void conjugate()
   {
      x = -x;
      y = -y;
      z = -z;
   }

   @Override
   public void negate()
   {
      x = -x;
      y = -y;
      z = -z;
      s = -s;
   }

   public void normalize()
   {
      QuaternionTools.normalize(this);
   }

   public void normalizeAndLimitToPiMinusPi()
   {
      QuaternionTools.normalizeAndLimitToPiMinusPi(this);
   }

   public boolean isNormalized(double epsilon)
   {
      double normSquared = normSquared();
      return !Double.isNaN(normSquared) && Math.abs(normSquared - 1.0) < epsilon;
   }

   public double norm()
   {
      return QuaternionTools.norm(this);
   }

   public double normSquared()
   {
      return QuaternionTools.normSquared(this);
   }

   @Override
   public void setToZero()
   {
      x = 0.0;
      y = 0.0;
      z = 0.0;
      s = 1.0;
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
   public boolean containsNaN()
   {
      return Tuple4DTools.containsNaN(this);
   }

   public void interpolate(QuaternionReadOnly q1, double alpha)
   {
      QuaternionTools.interpolate(this, q1, alpha, this);
   }

   public void interpolate(QuaternionReadOnly q1, QuaternionReadOnly q2, double alpha)
   {
      QuaternionTools.interpolate(q1, q2, alpha, this);
   }

   /**
   * Sets the value of this quaternion to the quaternion inverse of itself.
   */
   public void inverse()
   {
      conjugate();
      normalize();
   }

   public double dot(QuaternionReadOnly other)
   {
      return Tuple4DTools.dot(this, other);
   }

   public void difference(QuaternionReadOnly q1, QuaternionReadOnly q2)
   {
      QuaternionTools.multiplyConjugateLeft(q1, q2, this);
   }

   @Override
   public void set(Quaternion other)
   {
      set((Tuple4DReadOnly) other);
   }

   @Override
   public void set(Tuple4DReadOnly other)
   {
      x = other.getX();
      y = other.getY();
      z = other.getZ();
      s = other.getS();
      normalizeAndLimitToPiMinusPi();
   }

   public void set(AxisAngleReadOnly<?> axisAngle)
   {
      QuaternionConversion.convertAxisAngleToQuaternion(axisAngle, this);
   }

   public void set(RotationMatrixReadOnly rotationMatrix)
   {
      QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, this);
   }

   public void set(VectorReadOnly rotationVector)
   {
      QuaternionConversion.convertRotationVectorToQuaternion(rotationVector, this);
   }

   public void setYawPitchRoll(double[] yawPitchRoll)
   {
      QuaternionConversion.convertYawPitchRollToQuaternion(yawPitchRoll, this);
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      QuaternionConversion.convertYawPitchRollToQuaternion(yaw, pitch, roll, this);
   }

   @Override
   public void set(double qx, double qy, double qz, double qs)
   {
      setUnsafe(qx, qy, qz, qs);
      normalizeAndLimitToPiMinusPi();
   }

   @Override
   public void setUnsafe(double qx, double qy, double qz, double qs)
   {
      x = qx;
      y = qy;
      z = qz;
      s = qs;
   }

   public void setAndConjugate(QuaternionReadOnly other)
   {
      set(other);
      conjugate();
   }

   public void setAndNegate(QuaternionReadOnly other)
   {
      set(other);
      negate();
   }

   public void setAndInverse(QuaternionReadOnly other)
   {
      set(other);
      inverse();
   }

   public void set(double[] quaternionArray)
   {
      x = quaternionArray[0];
      y = quaternionArray[1];
      z = quaternionArray[2];
      s = quaternionArray[3];
      normalizeAndLimitToPiMinusPi();
   }

   public void set(double[] quaternionArray, int startIndex)
   {
      x = quaternionArray[startIndex++];
      y = quaternionArray[startIndex++];
      z = quaternionArray[startIndex++];
      s = quaternionArray[startIndex];
      normalizeAndLimitToPiMinusPi();
   }

   public void set(DenseMatrix64F matrix)
   {
      x = matrix.get(0, 0);
      y = matrix.get(1, 0);
      z = matrix.get(2, 0);
      s = matrix.get(3, 0);
      normalizeAndLimitToPiMinusPi();
   }

   public void set(DenseMatrix64F matrix, int startRow)
   {
      set(matrix, startRow, 0);
   }

   public void set(DenseMatrix64F matrix, int startRow, int column)
   {
      x = matrix.get(startRow++, column);
      y = matrix.get(startRow++, column);
      z = matrix.get(startRow++, column);
      s = matrix.get(startRow, column);
      normalizeAndLimitToPiMinusPi();
   }

   public void multiply(QuaternionReadOnly other)
   {
      QuaternionTools.multiply(this, other, this);
   }

   public void multiply(QuaternionReadOnly q1, QuaternionReadOnly q2)
   {
      QuaternionTools.multiply(q1, q2, this);
   }

   public void multiplyConjugateOther(QuaternionReadOnly other)
   {
      QuaternionTools.multiplyConjugateRight(this, other, this);
   }

   public void multiplyConjugateThis(QuaternionReadOnly other)
   {
      QuaternionTools.multiplyConjugateLeft(this, other, this);
   }

   public void preMultiply(QuaternionReadOnly other)
   {
      QuaternionTools.multiply(other, this, this);
   }

   public void preMultiplyConjugateOther(QuaternionReadOnly other)
   {
      QuaternionTools.multiplyConjugateLeft(other, this, this);
   }

   public void preMultiplyConjugateThis(QuaternionReadOnly other)
   {
      QuaternionTools.multiplyConjugateRight(other, this, this);
   }

   public void multiply(RotationMatrixReadOnly matrix)
   {
      QuaternionTools.multiply(this, matrix, this);
   }

   public void preMultiply(RotationMatrixReadOnly matrix)
   {
      QuaternionTools.multiply(matrix, this, this);
   }

   public void transform(TupleBasics tupleToTransform)
   {
      QuaternionTools.transform(this, tupleToTransform);
   }

   public void transform(TupleReadOnly tupleOriginal, TupleBasics tupleTransformed)
   {
      QuaternionTools.transform(this, tupleOriginal, tupleTransformed);
   }

   public void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      QuaternionTools.transform(this, tupleOriginal, tupleTransformed, true);
   }

   public void transform(Tuple2DBasics tupleToTransform, boolean checkIfTransformInXYPlane)
   {
      QuaternionTools.transform(this, tupleToTransform, checkIfTransformInXYPlane);
   }

   public void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      QuaternionTools.transform(this, tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   public void transform(QuaternionBasics quaternionToTransform)
   {
      transform(quaternionToTransform, quaternionToTransform);
   }

   public void transform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      QuaternionTools.transform(this, quaternionOriginal, quaternionTransformed);
   }

   public void transform(Vector4DBasics vectorToTransform)
   {
      transform(vectorToTransform, vectorToTransform);
   }

   public void transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      QuaternionTools.transform(this, vectorOriginal, vectorTransformed);
   }

   public void transform(Matrix3D matrixToTransform)
   {
      transform(matrixToTransform, matrixToTransform);
   }

   public void transform(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
   {
      QuaternionTools.transform(this, matrixOriginal, matrixTransformed);
   }

   public void transform(RotationMatrix matrixToTransform)
   {
      transform(matrixToTransform, matrixToTransform);
   }

   public void transform(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
   {
      QuaternionTools.transform(this, matrixOriginal, matrixTransformed);
   }

   public void inverseTransform(TupleBasics tupleToTransform)
   {
      QuaternionTools.inverseTransform(this, tupleToTransform);
   }

   public void inverseTransform(TupleReadOnly tupleOriginal, TupleBasics tupleTransformed)
   {
      QuaternionTools.inverseTransform(this, tupleOriginal, tupleTransformed);
   }

   public void inverseTransform(Tuple2DBasics tupleToTransform)
   {
      QuaternionTools.inverseTransform(this, tupleToTransform, true);
   }

   public void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      QuaternionTools.inverseTransform(this, tupleOriginal, tupleTransformed, true);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      transform.transform(this);
   }

   public double getAngle()
   {
      normalizeAndLimitToPiMinusPi();
      double sinHalfTheta = Math.sqrt(x * x + y * y + z * z);
      return 2.0 * Math.atan2(sinHalfTheta, s);
   }

   public void get(double[] quaternionArray)
   {
      quaternionArray[0] = x;
      quaternionArray[1] = y;
      quaternionArray[2] = z;
      quaternionArray[3] = s;
   }

   public void get(double[] quaternionArray, int startIndex)
   {
      quaternionArray[startIndex++] = x;
      quaternionArray[startIndex++] = y;
      quaternionArray[startIndex++] = z;
      quaternionArray[startIndex] = s;
   }

   public void get(DenseMatrix64F quaternionMatrixToPack)
   {
      quaternionMatrixToPack.set(0, 0, x);
      quaternionMatrixToPack.set(1, 0, y);
      quaternionMatrixToPack.set(2, 0, z);
      quaternionMatrixToPack.set(3, 0, s);
   }

   public void get(DenseMatrix64F quaternionMatrixToPack, int startRow)
   {
      get(quaternionMatrixToPack, startRow, 0);
   }

   public void get(DenseMatrix64F quaternionMatrixToPack, int startRow, int column)
   {
      quaternionMatrixToPack.set(startRow++, column, x);
      quaternionMatrixToPack.set(startRow++, column, y);
      quaternionMatrixToPack.set(startRow++, column, z);
      quaternionMatrixToPack.set(startRow, column, s);
   }

   public void get(QuaternionBasics quaternionToPack)
   {
      quaternionToPack.set(x, y, z, s);
   }

   public void get(VectorBasics rotationVectorToPack)
   {
      RotationVectorConversion.convertQuaternionToRotationVector(this, rotationVectorToPack);
   }

   public void get(AxisAngleBasics<?> axisAngleToPack)
   {
      AxisAngleConversion.convertQuaternionToAxisAngle(this, axisAngleToPack);
   }

   public void getYawPitchRoll(double[] yawPitchRollToPack)
   {
      YawPitchRollConversion.convertQuaternionToYawPitchRoll(this, yawPitchRollToPack);
   }

   public double getYaw()
   {
      return YawPitchRollConversion.computeYaw(this);
   }

   public double getPitch()
   {
      return YawPitchRollConversion.computePitch(this);
   }

   public double getRoll()
   {
      return YawPitchRollConversion.computeRoll(this);
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
   public double getS()
   {
      return s;
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
   public boolean epsilonEquals(Quaternion other, double epsilon)
   {
      return Tuple4DTools.epsilonEquals(this, other, epsilon);
   }

   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((Quaternion) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   public boolean equals(Quaternion other)
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
   
   public String toStringAsYawPitchRoll()
   {
      return "yaw-pitch-roll: (" + getYaw() + ", " + getPitch() + ", " + getRoll() + ")";
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
