package us.ihmc.geometry.matrix;

import java.io.Serializable;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.geometry.axisAngle.AxisAngleConversion;
import us.ihmc.geometry.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.geometry.interfaces.GeometryObject;
import us.ihmc.geometry.matrix.interfaces.Matrix3DBasics;
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
import us.ihmc.geometry.tuple4D.QuaternionConversion;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DReadOnly;
import us.ihmc.geometry.yawPitchRoll.YawPitchRollConversion;

public class RotationMatrix implements Serializable, Matrix3DBasics, RotationMatrixReadOnly, GeometryObject<RotationMatrix>
{
   private static final long serialVersionUID = 2802307840830134164L;

   /** The 9 coefficients of this matrix. */
   private double m00, m01, m02, m10, m11, m12, m20, m21, m22;

   public RotationMatrix()
   {
      setIdentity();
   }

   public RotationMatrix(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   public RotationMatrix(double[] rotationMatrixArray)
   {
      set(rotationMatrixArray);
   }

   public RotationMatrix(Matrix3DReadOnly rotationMatrix)
   {
      set(rotationMatrix);
   }

   public RotationMatrix(RotationMatrixReadOnly other)
   {
      set(other);
   }

   public RotationMatrix(AxisAngleReadOnly axisAngle)
   {
      set(axisAngle);
   }

   public RotationMatrix(QuaternionReadOnly quaternion)
   {
      set(quaternion);
   }

   public RotationMatrix(VectorReadOnly rotationVector)
   {
      set(rotationVector);
   }

   @Override
   public void setToZero()
   {
      setIdentity();
   }

   /**
    * @return the determinant of this matrix.
    */
   public double determinant()
   {
      return Matrix3DFeatures.determinant(this);
   }

   public void checkIfMatrixProper()
   {
      Matrix3DFeatures.checkIfRotationMatrix(this);
   }

   /**
    * Verify if this is a rotation matrix.
    * 
    * @return whether this is a rotation matrix or not.
    */
   public boolean isRotationMatrix()
   {
      return Matrix3DFeatures.isRotationMatrix(this);
   }

   @Override
   public void normalize()
   {
      Matrix3DTools.normalize(this);
   }

   /**
    * Sets this matrix to identity:
    * <pre>
    *     | 1  0  0 |
    * m = | 0  1  0 |
    *     | 0  0  1 |
    */
   public void setIdentity()
   {
      m01 = m02 = m12 = 0.0;
      m00 = m11 = m22 = 1.0;
      m10 = m20 = m21 = 0.0;
   }

   /**
    * Sets this matrix to contain only {@linkplain Double#NaN}:
    * <pre>
    *     | NaN  NaN  NaN |
    * m = | NaN  NaN  NaN |
    *     | NaN  NaN  NaN |
    */
   @Override
   public void setToNaN()
   {
      m00 = m01 = m02 = Double.NaN;
      m10 = m11 = m12 = Double.NaN;
      m20 = m21 = m22 = Double.NaN;
   }

   @Override
   public boolean containsNaN()
   {
      return Matrix3DFeatures.containsNaN(this);
   }

   /**
    * Sets the 9 coefficients of this rotation matrix without performing any checks on the data provided.
    * <p>
    * Only for internal usage.
    * @param m00 first matrix element in the first row.
    * @param m01 second matrix element in the first row.
    * @param m02 third matrix element in the first row.
    * @param m10 first matrix element in the second row.
    * @param m11 second matrix element in the second row.
    * @param m12 third matrix element in the second row.
    * @param m20 first matrix element in the third row.
    * @param m21 second matrix element in the third row.
    * @param m22 third matrix element in the third row.
    */
   public void setUnsafe(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      this.m00 = m00;
      this.m01 = m01;
      this.m02 = m02;

      this.m10 = m10;
      this.m11 = m11;
      this.m12 = m12;

      this.m20 = m20;
      this.m21 = m21;
      this.m22 = m22;
   }

   @Override
   public void set(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      checkIfMatrixProper();
   }

   public void setAndNormalize(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      normalize();
   }

   public final void set(double[] matrixArray)
   {
      Matrix3DBasicsTools.setMatrixFromArray(matrixArray, this);
   }

   @Override
   public final void set(Matrix3DReadOnly other)
   {
      Matrix3DBasicsTools.setMatrixFromOther(other, this);
   }

   public final void set(DenseMatrix64F matrix)
   {
      Matrix3DBasicsTools.setMatrixFromDenseMatrix(matrix, this);
   }

   public final void set(DenseMatrix64F matrix, int startRow, int startColumn)
   {
      Matrix3DBasicsTools.setMatrixFromDenseMatrix(matrix, startRow, startColumn, this);
   }

   @Override
   public void set(RotationMatrix other)
   {
      set((RotationMatrixReadOnly) other);
   }

   public void set(RotationMatrixReadOnly other)
   {
      m00 = other.getM00();
      m01 = other.getM01();
      m02 = other.getM02();
      m10 = other.getM10();
      m11 = other.getM11();
      m12 = other.getM12();
      m20 = other.getM20();
      m21 = other.getM21();
      m22 = other.getM22();
   }

   public final void setAndNormalize(Matrix3DReadOnly matrix)
   {
      m00 = matrix.getM00();
      m01 = matrix.getM01();
      m02 = matrix.getM02();
      m10 = matrix.getM10();
      m11 = matrix.getM11();
      m12 = matrix.getM12();
      m20 = matrix.getM20();
      m21 = matrix.getM21();
      m22 = matrix.getM22();
      normalize();
   }

   public void setAndNormalize(RotationMatrixReadOnly other)
   {
      set(other);
      normalize();
   }

   public void setAndInvert(Matrix3DReadOnly matrix)
   {
      setAndTranspose(matrix);
   }

   public void setAndInvert(RotationMatrixReadOnly other)
   {
      setAndTranspose(other);
   }
   
   public void setAndTranspose(Matrix3DReadOnly matrix)
   {
      set(matrix);
      transpose();
   }

   public void setAndTranspose(RotationMatrixReadOnly other)
   {
      set(other);
      transpose();
   }

   public void set(AxisAngleReadOnly axisAngle)
   {
      RotationMatrixConversion.convertAxisAngleToMatrix(axisAngle, this);
   }

   public void set(QuaternionReadOnly quaternion)
   {
      RotationMatrixConversion.convertQuaternionToMatrix(quaternion, this);
   }

   public void set(VectorReadOnly rotationVector)
   {
      RotationMatrixConversion.convertRotationVectorToMatrix(rotationVector, this);
   }

   public void setToPitchMatrix(double pitch)
   {
      RotationMatrixConversion.computePitchMatrix(pitch, this);
   }

   public void setToRollMatrix(double roll)
   {
      RotationMatrixConversion.computeRollMatrix(roll, this);
   }

   public void setToYawMatrix(double yaw)
   {
      RotationMatrixConversion.computeYawMatrix(yaw, this);
   }

   public void setYawPitchRoll(double[] yawPitchRoll)
   {
      setYawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      RotationMatrixConversion.convertYawPitchRollToMatrix(yaw, pitch, roll, this);
   }

   public void setEuler(VectorReadOnly eulerAngles)
   {
      setYawPitchRoll(eulerAngles.getZ(), eulerAngles.getY(), eulerAngles.getX());
   }

   public void setEuler(double rotX, double rotY, double rotZ)
   {
      setYawPitchRoll(rotZ, rotY, rotX);
   }

   public void invert()
   {
      transpose();
   }

   public void transpose()
   {
      double temp;

      temp = m10;
      m10 = m01;
      m01 = temp;

      temp = m20;
      m20 = m02;
      m02 = temp;

      temp = m21;
      m21 = m12;
      m12 = temp;
   }

   public void multiply(RotationMatrixReadOnly other)
   {
      RotationMatrixTools.multiply(this, other, this);
   }

   public void multiplyTransposeThis(RotationMatrixReadOnly other)
   {
      RotationMatrixTools.multiplyTransposeLeft(this, other, this);
   }

   public void multiplyTransposeOther(RotationMatrixReadOnly other)
   {
      RotationMatrixTools.multiplyTransposeRight(this, other, this);
   }

   public void multiplyTransposeBoth(RotationMatrixReadOnly other)
   {
      RotationMatrixTools.multiplyTransposeBoth(this, other, this);
   }

   public void preMultiply(RotationMatrixReadOnly other)
   {
      RotationMatrixTools.multiply(other, this, this);
   }

   public void preMultiplyTransposeThis(RotationMatrixReadOnly other)
   {
      RotationMatrixTools.multiplyTransposeRight(other, this, this);
   }

   public void preMultiplyTransposeOther(RotationMatrixReadOnly other)
   {
      RotationMatrixTools.multiplyTransposeLeft(other, this, this);
   }

   public void preMultiplyTransposeBoth(RotationMatrixReadOnly other)
   {
      RotationMatrixTools.multiplyTransposeBoth(other, this, this);
   }

   public void transform(TupleBasics tupleToTransform)
   {
      RotationMatrixTools.transform(this, tupleToTransform, tupleToTransform);
   }

   public void transform(TupleReadOnly tupleOriginal, TupleBasics tupleTransformed)
   {
      RotationMatrixTools.transform(this, tupleOriginal, tupleTransformed);
   }

   public void transform(Tuple2DBasics tupleToTransform)
   {
      RotationMatrixTools.transform(this, tupleToTransform, tupleToTransform, true);
   }

   public void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      RotationMatrixTools.transform(this, tupleOriginal, tupleTransformed, true);
   }

   public void transform(Tuple2DBasics tupleToTransform, boolean checkIfRotationInXYPlane)
   {
      RotationMatrixTools.transform(this, tupleToTransform, tupleToTransform, checkIfRotationInXYPlane);
   }

   public void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfRotationInXYPlane)
   {
      RotationMatrixTools.transform(this, tupleOriginal, tupleTransformed, checkIfRotationInXYPlane);
   }

   public void transform(QuaternionBasics quaternionToTransform)
   {
      RotationMatrixTools.transform(this, quaternionToTransform, quaternionToTransform);
   }
   
   public void transform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      RotationMatrixTools.transform(this, quaternionOriginal, quaternionTransformed);
   }

   public void transform(Vector4DBasics vectorToTransform)
   {
      RotationMatrixTools.transform(this, vectorToTransform, vectorToTransform);
   }
   
   public void transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      RotationMatrixTools.transform(this, vectorOriginal, vectorTransformed);
   }

   public void transform(RotationMatrix matrixToTransform)
   {
      RotationMatrixTools.transform(this, matrixToTransform, matrixToTransform);
   }

   public void transform(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
   {
      RotationMatrixTools.transform(this, matrixOriginal, matrixTransformed);
   }
   
   public void transform(Matrix3D matrixToTransform)
   {
      RotationMatrixTools.transform(this, matrixToTransform, matrixToTransform);
   }
   
   public void transform(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
   {
      RotationMatrixTools.transform(this, matrixOriginal, matrixTransformed);
   }

   public void inverseTransform(TupleBasics tupleToTransform)
   {
      RotationMatrixTools.inverseTransform(this, tupleToTransform, tupleToTransform);
   }

   public void inverseTransform(TupleReadOnly tupleOriginal, TupleBasics tupleTransformed)
   {
      RotationMatrixTools.inverseTransform(this, tupleOriginal, tupleTransformed);
   }

   public void inverseTransform(Tuple2DBasics tupleToTransform)
   {
      RotationMatrixTools.inverseTransform(this, tupleToTransform, tupleToTransform, true);
   }

   public void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      RotationMatrixTools.inverseTransform(this, tupleOriginal, tupleTransformed, true);
   }

   public void inverseTransform(Vector4DBasics vectorToTransform)
   {
      RotationMatrixTools.inverseTransform(this, vectorToTransform, vectorToTransform);
   }

   public void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      RotationMatrixTools.inverseTransform(this, vectorOriginal, vectorTransformed);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      transform.transform(this);
      normalize();
   }

   public void get(AxisAngleBasics axisAngleToPack)
   {
      AxisAngleConversion.convertMatrixToAxisAngle(this, axisAngleToPack);
   }

   public void get(QuaternionBasics quaternionToPack)
   {
      QuaternionConversion.convertMatrixToQuaternion(this, quaternionToPack);
   }

   public void get(VectorBasics rotationVectorToPack)
   {
      RotationVectorConversion.convertMatrixToRotationVector(this, rotationVectorToPack);
   }

   public void get(RotationMatrix rotationMatrixToPack)
   {
      rotationMatrixToPack.set(this);
   }

   /**
    * Computes the RPY angles from the rotation matrix for rotations about the
    * X, Y, and Z axes respectively. Note that this method is here for the
    * purpose of unit testing the method setEuler. This particular solution is
    * only valid for -pi/2 < vector.y < pi/2 and for vector.y != 0.
    * 
    */
   public void getEuler(TupleBasics eulerAnglesToPack)
   {
      YawPitchRollConversion.convertMatrixToYawPitchRoll(this, eulerAnglesToPack);
   }

   public void getYawPitchRoll(double[] yawPitchRollToPack)
   {
      YawPitchRollConversion.convertMatrixToYawPitchRoll(this, yawPitchRollToPack);
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

   public final void get(double[] matrixArrayToPack)
   {
      Matrix3DReadOnlyTools.getMatrixAsArray(this, matrixArrayToPack);
   }

   public final void get(double[] matrixArrayToPack, int startIndex)
   {
      Matrix3DReadOnlyTools.getMatrixAsArray(this, matrixArrayToPack, startIndex);
   }

   public final void get(DenseMatrix64F matrixToPack)
   {
      Matrix3DReadOnlyTools.getMatrixAsDenseMatrix(this, matrixToPack);
   }

   public final void get(DenseMatrix64F matrixToPack, int startRow, int startColumn)
   {
      Matrix3DReadOnlyTools.getMatrixAsDenseMatrix(this, matrixToPack, startRow, startColumn);
   }

   public final void getColumn(int column, double columnArrayToPack[])
   {
      Matrix3DReadOnlyTools.getMatrixColumn(this, column, columnArrayToPack);
   }

   public final void getColumn(int column, TupleBasics columnToPack)
   {
      Matrix3DReadOnlyTools.getMatrixColumn(this, column, columnToPack);
   }

   @Override
   public final double getElement(int row, int column)
   {
      return Matrix3DReadOnlyTools.getMatrixElement(this, row, column);
   }

   public final void getRow(int row, double rowArrayToPack[])
   {
      Matrix3DReadOnlyTools.getMatrixRow(this, row, rowArrayToPack);
   }

   public final void getRow(int row, TupleBasics rowVectorToPack)
   {
      Matrix3DReadOnlyTools.getMatrixRow(this, row, rowVectorToPack);
   }

   @Override
   public double getM00()
   {
      return m00;
   }

   @Override
   public double getM01()
   {
      return m01;
   }

   @Override
   public double getM02()
   {
      return m02;
   }

   @Override
   public double getM10()
   {
      return m10;
   }

   @Override
   public double getM11()
   {
      return m11;
   }

   @Override
   public double getM12()
   {
      return m12;
   }

   @Override
   public double getM20()
   {
      return m20;
   }

   @Override
   public double getM21()
   {
      return m21;
   }

   @Override
   public double getM22()
   {
      return m22;
   }

   @Override
   public boolean epsilonEquals(RotationMatrix other, double epsilon)
   {
      return Matrix3DFeatures.epsilonEquals(this, other, epsilon);
   }

   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((RotationMatrix) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   public boolean equals(RotationMatrix other)
   {
      return Matrix3DFeatures.equals(this, other);
   }

   @Override
   public String toString()
   {
      return Matrix3DReadOnlyTools.toString(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = 31L * bits + Double.doubleToLongBits(m00);
      bits = 31L * bits + Double.doubleToLongBits(m01);
      bits = 31L * bits + Double.doubleToLongBits(m02);
      bits = 31L * bits + Double.doubleToLongBits(m10);
      bits = 31L * bits + Double.doubleToLongBits(m11);
      bits = 31L * bits + Double.doubleToLongBits(m12);
      bits = 31L * bits + Double.doubleToLongBits(m20);
      bits = 31L * bits + Double.doubleToLongBits(m21);
      bits = 31L * bits + Double.doubleToLongBits(m22);
      return (int) (bits ^ bits >> 32);
   }
}
