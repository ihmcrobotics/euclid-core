package us.ihmc.geometry.transform;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.geometry.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.geometry.interfaces.EpsilonComparable;
import us.ihmc.geometry.interfaces.Settable;
import us.ihmc.geometry.matrix.Matrix3D;
import us.ihmc.geometry.matrix.Matrix3DReadOnlyTools;
import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.matrix.RotationScaleMatrix;
import us.ihmc.geometry.matrix.interfaces.Matrix3DBasics;
import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationScaleMatrixReadOnly;
import us.ihmc.geometry.transform.interfaces.Transform;
import us.ihmc.geometry.tuple.Vector;
import us.ihmc.geometry.tuple.interfaces.PointBasics;
import us.ihmc.geometry.tuple.interfaces.PointReadOnly;
import us.ihmc.geometry.tuple.interfaces.TupleBasics;
import us.ihmc.geometry.tuple.interfaces.TupleReadOnly;
import us.ihmc.geometry.tuple.interfaces.VectorBasics;
import us.ihmc.geometry.tuple.interfaces.VectorReadOnly;
import us.ihmc.geometry.tuple2D.interfaces.Point2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DReadOnly;

public class AffineTransform implements Transform, EpsilonComparable<AffineTransform>, Settable<AffineTransform>
{
   private final RotationScaleMatrix rotationScaleMatrix = new RotationScaleMatrix();
   private final Vector translationVector = new Vector();

   public AffineTransform()
   {
   }

   public AffineTransform(AffineTransform other)
   {
      set(other);
   }

   public AffineTransform(RigidBodyTransform rigidBodyTransform)
   {
      set(rigidBodyTransform);
   }

   public AffineTransform(RotationScaleMatrixReadOnly<?> rotationScaleMatrix, TupleReadOnly translation)
   {
      set(rotationScaleMatrix, translation);
   }

   @Override
   public void setToZero()
   {
      setIdentity();
   }

   @Override
   public void setToNaN()
   {
      setRotationToNaN();
      setTranslationToNaN();
   }

   public void setRotationToNaN()
   {
      rotationScaleMatrix.setToNaN();
   }

   public void setTranslationToNaN()
   {
      translationVector.setToNaN();
   }

   @Override
   public boolean containsNaN()
   {
      return rotationScaleMatrix.containsNaN() || translationVector.containsNaN();
   }

   public void resetRotation()
   {
      rotationScaleMatrix.setRotationToZero();
   }

   public void resetScale()
   {
      rotationScaleMatrix.resetScale();
   }

   public void resetTranslation()
   {
      translationVector.setToZero();
   }

   public void setIdentity()
   {
      rotationScaleMatrix.setIdentity();
      translationVector.setToZero();
   }

   public void set(double m00, double m01, double m02, double m03, double m10, double m11, double m12, double m13, double m20, double m21, double m22,
                   double m23)
   {
      rotationScaleMatrix.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      translationVector.set(m03, m13, m23);
   }

   @Override
   public void set(AffineTransform other)
   {
      rotationScaleMatrix.set(other.rotationScaleMatrix);
      translationVector.set(other.translationVector);
   }

   public void set(RigidBodyTransform rigidBodyTransform)
   {
      rigidBodyTransform.get(rotationScaleMatrix, translationVector);
   }

   public void set(DenseMatrix64F affineTransformMatrix)
   {
      rotationScaleMatrix.set(affineTransformMatrix);
      translationVector.set(affineTransformMatrix, 0, 3);
   }

   public void set(DenseMatrix64F affineTransformMatrix, int startRow, int startColumn)
   {
      rotationScaleMatrix.set(affineTransformMatrix, startRow, startColumn);
      translationVector.set(affineTransformMatrix, startRow, startColumn + 3);
   }

   public void set(double[] transformArray)
   {
      double m00 = transformArray[0];
      double m01 = transformArray[1];
      double m02 = transformArray[2];
      double m03 = transformArray[3];
      double m10 = transformArray[4];
      double m11 = transformArray[5];
      double m12 = transformArray[6];
      double m13 = transformArray[7];
      double m20 = transformArray[8];
      double m21 = transformArray[9];
      double m22 = transformArray[10];
      double m23 = transformArray[11];

      rotationScaleMatrix.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      translationVector.set(m03, m13, m23);
   }

   public void set(Matrix3DReadOnly<?> rotationScaleMatrix, TupleReadOnly translation)
   {
      this.rotationScaleMatrix.set(rotationScaleMatrix);
      translationVector.set(translation);
   }

   public void set(RotationScaleMatrixReadOnly<?> rotationScaleMatrix, TupleReadOnly translation)
   {
      this.rotationScaleMatrix.set(rotationScaleMatrix);
      translationVector.set(translation);
   }

   public void set(Matrix3DReadOnly<?> rotationMatrix, double scale, TupleReadOnly translation)
   {
      rotationScaleMatrix.set(rotationMatrix, scale);
      translationVector.set(translation);
   }

   public void set(Matrix3DReadOnly<?> rotationMatrix, double scalex, double scaley, double scalez, TupleReadOnly translation)
   {
      rotationScaleMatrix.set(rotationMatrix, scalex, scaley, scalez);
      translationVector.set(translation);
   }

   public void set(Matrix3DReadOnly<?> rotationMatrix, TupleReadOnly scales, TupleReadOnly translation)
   {
      rotationScaleMatrix.set(rotationMatrix, scales);
      translationVector.set(translation);
   }

   public void set(RotationMatrixReadOnly<?> rotationMatrix, double scale, TupleReadOnly translation)
   {
      rotationScaleMatrix.set(rotationMatrix, scale);
      translationVector.set(translation);
   }

   public void set(RotationMatrixReadOnly<?> rotationMatrix, double scalex, double scaley, double scalez, TupleReadOnly translation)
   {
      rotationScaleMatrix.set(rotationMatrix, scalex, scaley, scalez);
      translationVector.set(translation);
   }

   public void set(RotationMatrixReadOnly<?> rotationMatrix, TupleReadOnly scales, TupleReadOnly translation)
   {
      rotationScaleMatrix.set(rotationMatrix, scales);
      translationVector.set(translation);
   }

   public void set(AxisAngleReadOnly<?> axisAngle, double scale, TupleReadOnly translation)
   {
      rotationScaleMatrix.set(axisAngle, scale);
      translationVector.set(translation);
   }

   public void set(AxisAngleReadOnly<?> axisAngle, double scalex, double scaley, double scalez, TupleReadOnly translation)
   {
      rotationScaleMatrix.set(axisAngle, scalex, scaley, scalez);
      translationVector.set(translation);
   }

   public void set(AxisAngleReadOnly<?> axisAngle, TupleReadOnly scales, TupleReadOnly translation)
   {
      rotationScaleMatrix.set(axisAngle, scales);
      translationVector.set(translation);
   }

   public void set(QuaternionReadOnly quaternion, double scale, TupleReadOnly translation)
   {
      rotationScaleMatrix.set(quaternion, scale);
      translationVector.set(translation);
   }

   public void set(QuaternionReadOnly quaternion, double scalex, double scaley, double scalez, TupleReadOnly translation)
   {
      rotationScaleMatrix.set(quaternion, scalex, scaley, scalez);
      translationVector.set(translation);
   }

   public void set(QuaternionReadOnly quaternion, TupleReadOnly scales, TupleReadOnly translation)
   {
      rotationScaleMatrix.set(quaternion, scales);
      translationVector.set(translation);
   }

   public void setRotation(AxisAngleReadOnly<?> axisAngle)
   {
      rotationScaleMatrix.setRotation(axisAngle);
   }

   public void setRotation(VectorReadOnly rotationVector)
   {
      rotationScaleMatrix.setRotation(rotationVector);
   }

   public void setRotation(DenseMatrix64F matrix)
   {
      rotationScaleMatrix.setRotation(matrix);
   }

   public void setRotation(QuaternionReadOnly quaternion)
   {
      rotationScaleMatrix.setRotation(quaternion);
   }

   public void setRotation(Matrix3DReadOnly<?> rotationMatrix)
   {
      rotationScaleMatrix.setRotation(rotationMatrix);
   }

   public void setRotation(RotationMatrix rotationMatrix)
   {
      rotationScaleMatrix.setRotation(rotationMatrix);
   }

   public void setRotationYaw(double yaw)
   {
      rotationScaleMatrix.setToYawMatrix(yaw);
   }

   public void setRotationPitch(double pitch)
   {
      rotationScaleMatrix.setToPitchMatrix(pitch);
   }

   public void setRotationRoll(double roll)
   {
      rotationScaleMatrix.setToRollMatrix(roll);
   }

   public void setRotationYawPitchRoll(double yaw, double pitch, double roll)
   {
      rotationScaleMatrix.setYawPitchRoll(yaw, pitch, roll);
   }

   public void setRotationEuler(VectorReadOnly eulerAngles)
   {
      rotationScaleMatrix.setEuler(eulerAngles);
   }

   public void setRotationEuler(double rotX, double rotY, double rotZ)
   {
      rotationScaleMatrix.setEuler(rotX, rotY, rotZ);
   }

   public void setScale(double scale)
   {
      rotationScaleMatrix.setScale(scale);
   }

   public void setScale(double scalex, double scaley, double scalez)
   {
      rotationScaleMatrix.setScale(scalex, scaley, scalez);
   }

   public void setScale(TupleReadOnly scales)
   {
      rotationScaleMatrix.setScale(scales);
   }

   public void setTranslation(double x, double y, double z)
   {
      translationVector.set(x, y, z);
   }

   public void setTranslation(TupleReadOnly translation)
   {
      translationVector.set(translation);
   }

   public void preMultiply(RigidBodyTransform other)
   {
      other.transform(translationVector);
      translationVector.add(other.getTranslationVector());
      rotationScaleMatrix.preMultiply(other.getRotationMatrix());
   }

   @Override
   public void transform(PointReadOnly pointOriginal, PointBasics pointTransformed)
   {
      rotationScaleMatrix.transform(pointOriginal, pointTransformed);
      pointTransformed.add(translationVector);
   }

   @Override
   public void transform(VectorReadOnly vectorOriginal, VectorBasics vectorTransformed)
   {
      rotationScaleMatrix.transform(vectorOriginal, vectorTransformed);
   }

   @Override
   public void transform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      rotationScaleMatrix.transform(quaternionOriginal, quaternionTransformed);
   }

   @Override
   public void transform(Vector4DReadOnly vector4DOriginal, Vector4DBasics vector4DTransformed)
   {
      rotationScaleMatrix.transform(vector4DOriginal, vector4DTransformed);
   }

   @Override
   public void transform(RotationMatrixReadOnly<?> matrixOriginal, RotationMatrix matrixTranformed)
   {
      rotationScaleMatrix.transform(matrixOriginal, matrixTranformed);
   }

   @Override
   public void transform(Matrix3DReadOnly<?> matrixOriginal, Matrix3D matrixTransformed)
   {
      rotationScaleMatrix.transform(matrixOriginal, matrixTransformed);
   }

   @Override
   public void transform(Point2DBasics pointOriginal, Point2DBasics pointTransformed, boolean checkIfTransformInXYPlane)
   {
      rotationScaleMatrix.transform(pointOriginal, pointTransformed, checkIfTransformInXYPlane);
      pointTransformed.add(translationVector.getX(), translationVector.getY());
   }

   @Override
   public void transform(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed, boolean checkIfTransformInXYPlane)
   {
      rotationScaleMatrix.transform(vectorOriginal, vectorTransformed, checkIfTransformInXYPlane);
   }

   public void inverseTransform(PointBasics pointToTransform)
   {
      inverseTransform(pointToTransform, pointToTransform);
   }

   public void inverseTransform(PointReadOnly pointOriginal, PointBasics pointTransformed)
   {
      pointTransformed.set(pointOriginal);
      pointTransformed.sub(translationVector);
      rotationScaleMatrix.inverseTransform(pointTransformed);
   }

   public void inverseTransform(VectorBasics vectorToTransform)
   {
      inverseTransform(vectorToTransform, vectorToTransform);
   }

   public void inverseTransform(VectorReadOnly vectorOriginal, VectorBasics vectorTransformed)
   {
      rotationScaleMatrix.inverseTransform(vectorOriginal, vectorTransformed);
   }

   public void inverseTransform(Point2DBasics pointToTransform)
   {
      inverseTransform(pointToTransform, pointToTransform);
   }

   public void inverseTransform(Point2DBasics pointOriginal, Point2DBasics pointTransformed)
   {
      pointTransformed.set(pointOriginal);
      pointTransformed.sub(translationVector.getX(), translationVector.getY());
      rotationScaleMatrix.inverseTransform(pointTransformed);
   }

   public void inverseTransform(Vector2DBasics vectorToTransform)
   {
      inverseTransform(vectorToTransform, vectorToTransform);
   }

   public void inverseTransform(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed)
   {
      rotationScaleMatrix.inverseTransform(vectorOriginal, vectorTransformed);
   }

   public void getRigidBodyTransform(RigidBodyTransform rigidBodyTransformToPack)
   {
      rigidBodyTransformToPack.setRotation(rotationScaleMatrix.getRotationMatrix());
      rigidBodyTransformToPack.setTranslation(translationVector);
   }

   public void get(DenseMatrix64F matrixToPack)
   {
      rotationScaleMatrix.get(matrixToPack);
      translationVector.get(matrixToPack, 0, 3);
      matrixToPack.set(3, 0, 0.0);
      matrixToPack.set(3, 1, 0.0);
      matrixToPack.set(3, 2, 0.0);
      matrixToPack.set(3, 3, 1.0);
   }

   public void get(DenseMatrix64F matrixToPack, int startRow, int startColumn)
   {
      rotationScaleMatrix.get(startRow, startColumn, matrixToPack);
      translationVector.get(matrixToPack, startRow, startColumn + 3);
      startRow += 3;
      matrixToPack.set(startRow, startColumn++, 0.0);
      matrixToPack.set(startRow, startColumn++, 0.0);
      matrixToPack.set(startRow, startColumn++, 0.0);
      matrixToPack.set(startRow, startColumn, 1.0);
   }

   public void get(Matrix3DBasics<?> rotationScaleMarixToPack, TupleBasics translationToPack)
   {
      rotationScaleMarixToPack.set(rotationScaleMatrix);
      translationToPack.set(translationVector);
   }

   public void get(RotationScaleMatrix rotationScaleMarixToPack, TupleBasics translationToPack)
   {
      rotationScaleMarixToPack.set(rotationScaleMatrix);
      translationToPack.set(translationVector);
   }

   public void get(double[] transformArrayToPack)
   {
      transformArrayToPack[0] = getM00();
      transformArrayToPack[1] = getM01();
      transformArrayToPack[2] = getM02();
      transformArrayToPack[3] = getM03();
      transformArrayToPack[4] = getM10();
      transformArrayToPack[5] = getM11();
      transformArrayToPack[6] = getM12();
      transformArrayToPack[7] = getM13();
      transformArrayToPack[8] = getM20();
      transformArrayToPack[9] = getM21();
      transformArrayToPack[10] = getM22();
      transformArrayToPack[11] = getM23();
      transformArrayToPack[12] = getM30();
      transformArrayToPack[13] = getM31();
      transformArrayToPack[14] = getM32();
      transformArrayToPack[15] = getM33();
   }

   public RotationMatrixReadOnly<?> getRotationMatrix()
   {
      return rotationScaleMatrix.getRotationMatrix();
   }

   public void getRotation(Matrix3DBasics<?> rotationMatrixToPack)
   {
      rotationMatrixToPack.set(rotationScaleMatrix.getRotationMatrix());
   }

   public void getRotation(RotationMatrix rotationMatrixToPack)
   {
      rotationScaleMatrix.getRotation(rotationMatrixToPack);
   }

   public void getRotation(DenseMatrix64F rotationMatrixToPack)
   {
      rotationScaleMatrix.getRotation(rotationMatrixToPack);
   }

   public void getRotation(double[] rotationMatrixArrayToPack)
   {
      rotationScaleMatrix.getRotation(rotationMatrixArrayToPack);
   }

   public void getRotation(QuaternionBasics quaternionToPack)
   {
      rotationScaleMatrix.getRotation(quaternionToPack);
   }

   public void getRotation(AxisAngleBasics<?> axisAngleToPack)
   {
      rotationScaleMatrix.getRotation(axisAngleToPack);
   }

   public void getRotation(VectorBasics rotationVectorToPack)
   {
      rotationScaleMatrix.getRotation(rotationVectorToPack);
   }

   public RotationScaleMatrixReadOnly<?> getRotationScaleMatrix()
   {
      return rotationScaleMatrix;
   }

   public void getRotationScale(Matrix3DBasics<?> rotationMatrixToPack)
   {
      rotationMatrixToPack.set(rotationScaleMatrix);
   }

   public void getRotationScale(RotationScaleMatrix rotationScaleMatrixToPack)
   {
      rotationScaleMatrixToPack.set(rotationScaleMatrix);
   }

   public void getRotationScale(DenseMatrix64F rotationScaleMatrixToPack)
   {
      rotationScaleMatrix.get(rotationScaleMatrixToPack);
   }

   public VectorReadOnly getTranslationVector()
   {
      return translationVector;
   }

   public void getTranslation(TupleBasics translationToPack)
   {
      translationToPack.set(translationVector);
   }

   public double getElement(int row, int column)
   {
      if (row < 3)
      {
         if (column < 3)
         {
            return rotationScaleMatrix.getElement(row, column);
         }
         else if (column < 4)
         {
            return translationVector.get(row);
         }
         else
         {
            throw Matrix3DReadOnlyTools.columnOutOfBoundsException(3, column);
         }
      }
      else if (row < 4)
      {
         if (column < 3)
         {
            return 0.0;
         }
         else if (column < 4)
         {
            return 1.0;
         }
         else
         {
            throw Matrix3DReadOnlyTools.columnOutOfBoundsException(3, column);
         }
      }
      else
      {
         throw Matrix3DReadOnlyTools.rowOutOfBoundsException(3, row);
      }
   }

   public double getM00()
   {
      return rotationScaleMatrix.getM00();
   }

   public double getM01()
   {
      return rotationScaleMatrix.getM01();
   }

   public double getM02()
   {
      return rotationScaleMatrix.getM02();
   }

   public double getM03()
   {
      return translationVector.getX();
   }

   public double getM10()
   {
      return rotationScaleMatrix.getM10();
   }

   public double getM11()
   {
      return rotationScaleMatrix.getM11();
   }

   public double getM12()
   {
      return rotationScaleMatrix.getM12();
   }

   public double getM13()
   {
      return translationVector.getY();
   }

   public double getM20()
   {
      return rotationScaleMatrix.getM20();
   }

   public double getM21()
   {
      return rotationScaleMatrix.getM21();
   }

   public double getM22()
   {
      return rotationScaleMatrix.getM22();
   }

   public double getM23()
   {
      return translationVector.getZ();
   }

   public double getM30()
   {
      return 0.0;
   }

   public double getM31()
   {
      return 0.0;
   }

   public double getM32()
   {
      return 0.0;
   }

   public double getM33()
   {
      return 1.0;
   }

   @Override
   public boolean epsilonEquals(AffineTransform other, double epsilon)
   {
      return rotationScaleMatrix.epsilonEquals(other.rotationScaleMatrix, epsilon) && translationVector.epsilonEquals(other.translationVector, epsilon);
   }

   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((AffineTransform) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   public boolean equals(AffineTransform other)
   {
      if (other == null)
         return false;
      else
         return rotationScaleMatrix.equals(other.rotationScaleMatrix) && translationVector.equals(other.translationVector);
   }

   @Override
   public String toString()
   {
      return TransformTools.toString(rotationScaleMatrix, translationVector);
   }
}
