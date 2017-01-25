package us.ihmc.geometry.matrix;

import java.io.Serializable;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.geometry.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.geometry.exceptions.NotARotationScaleMatrixException;
import us.ihmc.geometry.interfaces.EpsilonComparable;
import us.ihmc.geometry.interfaces.Settable;
import us.ihmc.geometry.matrix.interfaces.Matrix3DBasics;
import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationScaleMatrixReadOnly;
import us.ihmc.geometry.tuple.Vector;
import us.ihmc.geometry.tuple.interfaces.TupleBasics;
import us.ihmc.geometry.tuple.interfaces.TupleReadOnly;
import us.ihmc.geometry.tuple.interfaces.VectorBasics;
import us.ihmc.geometry.tuple.interfaces.VectorReadOnly;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DReadOnly;

public class RotationScaleMatrix implements Serializable, Matrix3DBasics, RotationScaleMatrixReadOnly, EpsilonComparable<RotationScaleMatrix>, Settable<RotationScaleMatrix>
{
   private static final long serialVersionUID = 5012534518639484244L;

   private final RotationMatrix rotationMatrix = new RotationMatrix();
   private final Vector scale = new Vector(1.0, 1.0, 1.0);

   /**
    * Set to identity
    */
   public RotationScaleMatrix()
   {
      setIdentity();
   }

   public RotationScaleMatrix(RotationScaleMatrix rotationScaleMatrix)
   {
      set(rotationScaleMatrix);
   }

   public RotationScaleMatrix(Matrix3DReadOnly rotationScaleMatrix)
   {
      set(rotationScaleMatrix);
   }

   public RotationScaleMatrix(DenseMatrix64F rotationScaleMatrix)
   {
      set(rotationScaleMatrix);
   }

   public RotationScaleMatrix(double[] rotationScaleMatrixArray)
   {
      set(rotationScaleMatrixArray);
   }

   public RotationScaleMatrix(AxisAngleReadOnly<?> axisAngle, double scale)
   {
      set(axisAngle, scale);
   }

   public RotationScaleMatrix(AxisAngleReadOnly<?> axisAngle, double scalex, double scaley, double scalez)
   {
      set(axisAngle, scalex, scaley, scalez);
   }

   public RotationScaleMatrix(AxisAngleReadOnly<?> axisAngle, TupleReadOnly scales)
   {
      set(axisAngle, scales);
   }

   public RotationScaleMatrix(DenseMatrix64F matrix, double scale)
   {
      set(matrix, scale);
   }

   public RotationScaleMatrix(DenseMatrix64F matrix, double scalex, double scaley, double scalez)
   {
      set(matrix, scalex, scaley, scalez);
   }

   public RotationScaleMatrix(DenseMatrix64F matrix, TupleReadOnly scales)
   {
      set(matrix, scales);
   }

   public RotationScaleMatrix(QuaternionReadOnly quaternion, double scale)
   {
      set(quaternion, scale);
   }

   public RotationScaleMatrix(QuaternionReadOnly quaternion, double scalex, double scaley, double scalez)
   {
      set(quaternion, scalex, scaley, scalez);
   }

   public RotationScaleMatrix(QuaternionReadOnly quaternion, TupleReadOnly scales)
   {
      set(quaternion, scales);
   }

   public RotationScaleMatrix(RotationMatrixReadOnly rotationMatrix, double scale)
   {
      set(rotationMatrix, scale);
   }

   public RotationScaleMatrix(RotationMatrixReadOnly rotationMatrix, double scalex, double scaley, double scalez)
   {
      set(rotationMatrix, scalex, scaley, scalez);
   }

   public RotationScaleMatrix(RotationMatrixReadOnly rotationMatrix, TupleReadOnly scales)
   {
      set(rotationMatrix, scales);
   }

   public void checkIfMatrixProper()
   {
      if (!rotationMatrix.isRotationMatrix())
         throw new NotARotationScaleMatrixException(this);
   }

   /**
    * Orthonormalization of the rotation matrix using Gram-Schmidt method.
    */
   public void normalizeRotationMatrix()
   {
      rotationMatrix.normalize();
   }

   public void resetScale()
   {
      scale.set(1.0, 1.0, 1.0);
   }

   @Override
   public void setToZero()
   {
      setIdentity();
   }

   @Override
   public void setToNaN()
   {
      rotationMatrix.setToNaN();
      scale.setToNaN();
   }

   public void setIdentity()
   {
      rotationMatrix.setIdentity();
      resetScale();
   }

   public void setRotationToZero()
   {
      rotationMatrix.setToZero();
   }

   @Override
   public boolean containsNaN()
   {
      return rotationMatrix.containsNaN() || scale.containsNaN();
   }

   @Override
   public void set(RotationScaleMatrix other)
   {
      rotationMatrix.set(other.rotationMatrix);
      scale.set(other.scale);
   }

   public void set(RotationScaleMatrixReadOnly other)
   {
      rotationMatrix.set(other.getRotationMatrix());
      scale.set(other.getScale());
   }

   public void set(RotationMatrixReadOnly rotationMatrix)
   {
      this.rotationMatrix.set(rotationMatrix);
      resetScale();
   }

   public void set(double[] rotationScaleMatrixArray)
   {
      Matrix3DBasicsTools.setMatrixFromArray(rotationScaleMatrixArray, this);
   }

   @Override
   public void set(Matrix3DReadOnly rotationScaleMatrix)
   {
      Matrix3DBasicsTools.setMatrixFromOther(rotationScaleMatrix, this);
   }

   public void set(DenseMatrix64F rotationScaleMatrix)
   {
      Matrix3DBasicsTools.setMatrixFromDenseMatrix(rotationScaleMatrix, this);
   }

   public void set(DenseMatrix64F rotationScaleMatrix, int startRow, int startColumn)
   {
      Matrix3DBasicsTools.setMatrixFromDenseMatrix(rotationScaleMatrix, startRow, startColumn, this);
   }

   @Override
   public void set(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      if (Matrix3DFeatures.determinant(m00, m01, m02, m10, m11, m12, m20, m21, m22) <= 0.0)
         throw new NotARotationScaleMatrixException(m00, m01, m02, m10, m11, m12, m20, m21, m22);

      scale.setX(Math.sqrt(m00 * m00 + m10 * m10 + m20 * m20));
      scale.setY(Math.sqrt(m01 * m01 + m11 * m11 + m21 * m21));
      scale.setZ(Math.sqrt(m02 * m02 + m12 * m12 + m22 * m22));

      if (scale.getX() == 0.0 || scale.getY() == 0.0 || scale.getZ() == 0.0)
         throw new NotARotationScaleMatrixException(m00, m01, m02, m10, m11, m12, m20, m21, m22);

      double rot00 = m00 / scale.getX();
      double rot01 = m01 / scale.getY();
      double rot02 = m02 / scale.getZ();
      double rot10 = m10 / scale.getX();
      double rot11 = m11 / scale.getY();
      double rot12 = m12 / scale.getZ();
      double rot20 = m20 / scale.getX();
      double rot21 = m21 / scale.getY();
      double rot22 = m22 / scale.getZ();

      if (!Matrix3DFeatures.isRotationMatrix(rot00, rot01, rot02, rot10, rot11, rot12, rot20, rot21, rot22))
         throw new NotARotationScaleMatrixException(m00, m01, m02, m10, m11, m12, m20, m21, m22);

      rotationMatrix.setUnsafe(rot00, rot01, rot02, rot10, rot11, rot12, rot20, rot21, rot22);
   }

   public void set(AxisAngleReadOnly<?> axisAngle, double scale)
   {
      set(axisAngle, scale, scale, scale);
   }

   public void set(AxisAngleReadOnly<?> axisAngle, double scalex, double scaley, double scalez)
   {
      rotationMatrix.set(axisAngle);
      setScale(scalex, scaley, scalez);
   }

   public void set(AxisAngleReadOnly<?> axisAngle, TupleReadOnly scales)
   {
      set(axisAngle, scales.getX(), scales.getY(), scales.getZ());
   }

   public void set(DenseMatrix64F matrix, double scale)
   {
      set(matrix, scale, scale, scale);
   }

   public void set(DenseMatrix64F rotationMatrix, double scalex, double scaley, double scalez)
   {
      set(rotationMatrix);
      setScale(scalex, scaley, scalez);
   }

   public void set(DenseMatrix64F rotationMatrix, TupleReadOnly scales)
   {
      set(rotationMatrix, scales.getX(), scales.getY(), scales.getZ());
   }

   public void set(QuaternionReadOnly quaternion, double scale)
   {
      set(quaternion, scale, scale, scale);
   }

   public void set(QuaternionReadOnly quaternion, double scalex, double scaley, double scalez)
   {
      rotationMatrix.set(quaternion);
      setScale(scalex, scaley, scalez);
   }

   public void set(QuaternionReadOnly quaternion, TupleReadOnly scales)
   {
      set(quaternion, scales.getX(), scales.getY(), scales.getZ());
   }

   public void set(Matrix3DReadOnly rotationMatrix, double scale)
   {
      set(rotationMatrix, scale, scale, scale);
   }

   public void set(Matrix3DReadOnly rotationMatrix, double scalex, double scaley, double scalez)
   {
      this.rotationMatrix.set(rotationMatrix);
      setScale(scalex, scaley, scalez);
   }

   public void set(Matrix3DReadOnly rotationMatrix, TupleReadOnly scales)
   {
      set(rotationMatrix, scales.getX(), scales.getY(), scales.getZ());
   }

   public void set(RotationMatrixReadOnly rotationMatrix, double scale)
   {
      set(rotationMatrix, scale, scale, scale);
   }

   public void set(RotationMatrixReadOnly rotationMatrix, double scalex, double scaley, double scalez)
   {
      this.rotationMatrix.set(rotationMatrix);
      setScale(scalex, scaley, scalez);
   }

   public void set(RotationMatrixReadOnly rotationMatrix, TupleReadOnly scales)
   {
      set(rotationMatrix, scales.getX(), scales.getY(), scales.getZ());
   }

   public void setRotation(DenseMatrix64F rotationMatrix)
   {
      this.rotationMatrix.set(rotationMatrix);
   }

   public void setRotation(double[] rotationMatrixArray)
   {
      rotationMatrix.set(rotationMatrixArray);
   }

   public void setRotation(AxisAngleReadOnly<?> axisAngle)
   {
      rotationMatrix.set(axisAngle);
   }

   public void setRotation(QuaternionReadOnly quaternion)
   {
      rotationMatrix.set(quaternion);
   }

   public void setRotation(Matrix3DReadOnly rotationMatrix)
   {
      this.rotationMatrix.set(rotationMatrix);
   }

   public void setRotation(RotationMatrixReadOnly rotationMatrix)
   {
      this.rotationMatrix.set(rotationMatrix);
   }

   public void setRotation(VectorReadOnly rotationVector)
   {
      rotationMatrix.set(rotationVector);
   }

   public void setScale(double scale)
   {
      setScale(scale, scale, scale);
   }

   public void setScale(double x, double y, double z)
   {
      if (x <= 0.0 || y <= 0.0 || z <= 0.0)
         throw new RuntimeException("Mirroring or zero scale is not handled, scale values: " + x + ", " + y + ", " + z + ".");

      scale.set(x, y, z);
   }

   public void setScale(TupleReadOnly scales)
   {
      setScale(scales.getX(), scales.getY(), scales.getZ());
   }

   public void setToPitchMatrix(double pitch)
   {
      rotationMatrix.setToPitchMatrix(pitch);
      resetScale();
   }

   public void setToRollMatrix(double roll)
   {
      rotationMatrix.setToRollMatrix(roll);
      resetScale();
   }

   public void setToYawMatrix(double yaw)
   {
      rotationMatrix.setToYawMatrix(yaw);
      resetScale();
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      rotationMatrix.setYawPitchRoll(yaw, pitch, roll);
      resetScale();
   }

   public void setEuler(VectorReadOnly eulerAngles)
   {
      rotationMatrix.setEuler(eulerAngles);
      resetScale();
   }

   public void setEuler(double rotX, double rotY, double rotZ)
   {
      rotationMatrix.setEuler(rotX, rotY, rotZ);
      resetScale();
   }

   public void preMultiply(RotationMatrixReadOnly rotationMatrix)
   {
      this.rotationMatrix.preMultiply(rotationMatrix);
   }

   public void preMultiplyTransposeOther(RotationMatrixReadOnly rotationMatrix)
   {
      this.rotationMatrix.preMultiplyTransposeOther(rotationMatrix);
   }

   public void transform(TupleBasics tupleToTransform)
   {
      RotationScaleMatrixTools.transform(this, tupleToTransform, tupleToTransform);
   }

   public void transform(TupleReadOnly tupleOriginal, TupleBasics tupleTransformed)
   {
      RotationScaleMatrixTools.transform(this, tupleOriginal, tupleTransformed);
   }

   public void transform(Tuple2DBasics tupleToTransform)
   {
      RotationScaleMatrixTools.transform(this, tupleToTransform, tupleToTransform, true);
   }

   public void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      RotationScaleMatrixTools.transform(this, tupleOriginal, tupleTransformed, true);
   }

   public void transform(Tuple2DBasics tupleToTransform, boolean checkIfTransformInXYPlane)
   {
      RotationScaleMatrixTools.transform(this, tupleToTransform, tupleToTransform, checkIfTransformInXYPlane);
   }

   public void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      RotationScaleMatrixTools.transform(this, tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   public void transform(QuaternionBasics quaternionToTransform)
   {
      RotationScaleMatrixTools.transform(this, quaternionToTransform, quaternionToTransform);
   }
   
   public void transform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      RotationScaleMatrixTools.transform(this, quaternionOriginal, quaternionTransformed);
   }

   public void transform(Vector4DBasics vectorToTransform)
   {
      RotationScaleMatrixTools.transform(this, vectorToTransform, vectorToTransform);
   }

   public void transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      RotationScaleMatrixTools.transform(this, vectorOriginal, vectorTransformed);
   }

   public void transform(RotationMatrix matrixToTransform)
   {
      RotationScaleMatrixTools.transform(this, matrixToTransform, matrixToTransform);
   }

   public void transform(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
   {
      RotationScaleMatrixTools.transform(this, matrixOriginal, matrixTransformed);
   }

   public void transform(Matrix3D matrixToTransform)
   {
      RotationScaleMatrixTools.transform(this, matrixToTransform, matrixToTransform);
   }

   public void transform(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
   {
      RotationScaleMatrixTools.transform(this, matrixOriginal, matrixTransformed);
   }

   public void inverseTransform(TupleBasics tupleToTransform)
   {
      RotationScaleMatrixTools.inverseTransform(this, tupleToTransform, tupleToTransform);
   }

   public void inverseTransform(TupleReadOnly tupleOriginal, TupleBasics tupleTransformed)
   {
      RotationScaleMatrixTools.inverseTransform(this, tupleOriginal, tupleTransformed);
   }

   public void inverseTransform(Tuple2DBasics tupleToTransform)
   {
      RotationScaleMatrixTools.inverseTransform(this, tupleToTransform, tupleToTransform, true);
   }

   public void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      RotationScaleMatrixTools.inverseTransform(this, tupleOriginal, tupleTransformed, true);
   }

   public void inverseTransform(Vector4DBasics vectorToTransform)
   {
      RotationScaleMatrixTools.inverseTransform(this, vectorToTransform, vectorToTransform);
   }

   public void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      RotationScaleMatrixTools.inverseTransform(this, vectorOriginal, vectorTransformed);
   }

   /**
    * Computes the RPY angles from the rotation matrix for rotations about the
    * X, Y, and Z axes respectively. Note that this method is here for the
    * purpose of unit testing the method setEuler. This particular solution is
    * only valid for -pi/2 < vector.y < pi/2 and for vector.y != 0.
    * 
    */
   public void getRotationEuler(TupleBasics eulerAnglesToPack)
   {
      rotationMatrix.getEuler(eulerAnglesToPack);
   }

   public double getMaxScale()
   {
      return Matrix3DTools.max(scale.getX(), scale.getY(), scale.getZ());
   }

   public void getRotation(RotationMatrix rotationMatrixToPack)
   {
      rotationMatrixToPack.set(rotationMatrix);
   }

   public void getRotation(double[] rotationMatrixArrayToPack)
   {
      rotationMatrix.get(rotationMatrixArrayToPack);
   }

   public void getRotation(DenseMatrix64F rotationMatrixToPack)
   {
      rotationMatrix.get(rotationMatrixToPack);
   }

   public void getRotation(QuaternionBasics quaternionToPack)
   {
      rotationMatrix.get(quaternionToPack);
   }

   public void getRotation(AxisAngleBasics<?> axisAngleToPack)
   {
      rotationMatrix.get(axisAngleToPack);
   }

   public void getRotation(VectorBasics rotationVectorToPack)
   {
      rotationMatrix.get(rotationVectorToPack);
   }

   public void getRotationYawPitchRoll(double[] yawPitchRollToPack)
   {
      rotationMatrix.getYawPitchRoll(yawPitchRollToPack);
   }

   public double getRotationYaw()
   {
      return rotationMatrix.getYaw();
   }

   public double getRotationPitch()
   {
      return rotationMatrix.getPitch();
   }

   public double getRotationRoll()
   {
      return rotationMatrix.getRoll();
   }

   public void getScale(TupleBasics scaleToPack)
   {
      scaleToPack.set(scale);
   }

   @Override
   public RotationMatrixReadOnly getRotationMatrix()
   {
      return rotationMatrix;
   }

   @Override
   public TupleReadOnly getScale()
   {
      return scale;
   }

   public void get(double[] rotationScaleMatrixArrayToPack)
   {
      Matrix3DReadOnlyTools.getMatrixAsArray(this, rotationScaleMatrixArrayToPack);
   }

   public void get(double[] rotationScaleMatrixArrayToPack, int startIndex)
   {
      Matrix3DReadOnlyTools.getMatrixAsArray(this, rotationScaleMatrixArrayToPack, startIndex);
   }

   public void get(DenseMatrix64F rotationScaleMatrixToPack)
   {
      Matrix3DReadOnlyTools.getMatrixAsDenseMatrix(this, rotationScaleMatrixToPack);
   }

   public void get(DenseMatrix64F rotationScaleMatrixToPack, int startRow, int startColumn)
   {
      Matrix3DReadOnlyTools.getMatrixAsDenseMatrix(this, rotationScaleMatrixToPack, startRow, startColumn);
   }

   public void getColumn(int column, double columnArrayToPack[])
   {
      Matrix3DReadOnlyTools.getMatrixColumn(this, column, columnArrayToPack);
   }

   public void getColumn(int column, TupleBasics columnToPack)
   {
      Matrix3DReadOnlyTools.getMatrixColumn(this, column, columnToPack);
   }

   @Override
   public double getElement(int row, int column)
   {
      return Matrix3DReadOnlyTools.getMatrixElement(this, row, column);
   }

   public void getRow(int row, double rowArrayToPack[])
   {
      Matrix3DReadOnlyTools.getMatrixRow(this, row, rowArrayToPack);
   }

   public void getRow(int row, TupleBasics rowVectorToPack)
   {
      Matrix3DReadOnlyTools.getMatrixRow(this, row, rowVectorToPack);
   }

   @Override
   public double getM00()
   {
      return rotationMatrix.getM00() * scale.getX();
   }

   @Override
   public double getM01()
   {
      return rotationMatrix.getM01() * scale.getY();
   }

   @Override
   public double getM02()
   {
      return rotationMatrix.getM02() * scale.getZ();
   }

   @Override
   public double getM10()
   {
      return rotationMatrix.getM10() * scale.getX();
   }

   @Override
   public double getM11()
   {
      return rotationMatrix.getM11() * scale.getY();
   }

   @Override
   public double getM12()
   {
      return rotationMatrix.getM12() * scale.getZ();
   }

   @Override
   public double getM20()
   {
      return rotationMatrix.getM20() * scale.getX();
   }

   @Override
   public double getM21()
   {
      return rotationMatrix.getM21() * scale.getY();
   }

   @Override
   public double getM22()
   {
      return rotationMatrix.getM22() * scale.getZ();
   }

   @Override
   public double getScaleX()
   {
      return scale.getX();
   }

   @Override
   public double getScaleY()
   {
      return scale.getY();
   }

   @Override
   public double getScaleZ()
   {
      return scale.getZ();
   }

   @Override
   public boolean epsilonEquals(RotationScaleMatrix other, double epsilon)
   {
      return rotationMatrix.epsilonEquals(other.rotationMatrix, epsilon) && scale.epsilonEquals(other.scale, epsilon);
   }
   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((RotationScaleMatrix) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   public boolean equals(RotationScaleMatrix other)
   {
      if (other == null)
         return false;
      else
         return rotationMatrix.equals(other.rotationMatrix) && scale.equals(other.scale);
   }

   @Override
   public String toString()
   {
      return Matrix3DReadOnlyTools.toString(getM00(), getM01(), getM02(), getM10(), getM11(), getM12(), getM20(), getM21(), getM22());
   }

   @Override
   public int hashCode()
   {
      long bits = 31L * rotationMatrix.hashCode() + scale.hashCode();
      return (int) (bits ^ bits >> 32);
   }
}