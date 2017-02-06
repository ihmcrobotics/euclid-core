package us.ihmc.geometry.transform;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.geometry.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.geometry.interfaces.EpsilonComparable;
import us.ihmc.geometry.interfaces.Settable;
import us.ihmc.geometry.matrix.Matrix3D;
import us.ihmc.geometry.matrix.Matrix3DReadOnlyTools;
import us.ihmc.geometry.matrix.Matrix3DTools;
import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.matrix.RotationScaleMatrix;
import us.ihmc.geometry.matrix.interfaces.Matrix3DBasics;
import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.geometry.transform.interfaces.Transform;
import us.ihmc.geometry.tuple2D.interfaces.Point2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.geometry.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.geometry.tuple3D.Vector;
import us.ihmc.geometry.tuple3D.interfaces.Point3DBasics;
import us.ihmc.geometry.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.geometry.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.geometry.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DReadOnly;

public class RigidBodyTransform implements Transform, EpsilonComparable<RigidBodyTransform>, Settable<RigidBodyTransform>
{
   private final RotationMatrix rotationMatrix = new RotationMatrix();
   private final Vector translationVector = new Vector();

   public RigidBodyTransform()
   {
   }

   public RigidBodyTransform(RigidBodyTransform other)
   {
      set(other);
   }

   public RigidBodyTransform(QuaternionBasedTransform quaternionBasedTransform)
   {
      set(quaternionBasedTransform);
   }

   public RigidBodyTransform(DenseMatrix64F matrix)
   {
      set(matrix);
   }

   public RigidBodyTransform(RotationMatrix rotationMatrix, Tuple3DReadOnly translation)
   {
      set(rotationMatrix, translation);
   }

   public RigidBodyTransform(QuaternionReadOnly quaternion, Tuple3DReadOnly translation)
   {
      set(quaternion, translation);
   }

   public RigidBodyTransform(AxisAngleReadOnly<?> axisAngle, Tuple3DReadOnly translation)
   {
      set(axisAngle, translation);
   }

   public RigidBodyTransform(double[] transformArray)
   {
      set(transformArray);
   }

   public RigidBodyTransform(double m00, double m01, double m02, double m03, double m10, double m11, double m12, double m13, double m20, double m21, double m22, double m23)
   {
      set(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23);
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
      translationVector.setToNaN();
   }

   @Override
   public boolean containsNaN()
   {
      return rotationMatrix.containsNaN() || translationVector.containsNaN();
   }

   public void resetRotation()
   {
      rotationMatrix.setIdentity();
   }

   public void resetTranslation()
   {
      translationVector.setToZero();
   }

   public void normalizeRotationPart()
   {
      rotationMatrix.normalize();
   }

   public double determinantRotationPart()
   {
      return rotationMatrix.determinant();
   }

   public void set(double m00, double m01, double m02, double m03, double m10, double m11, double m12, double m13, double m20, double m21, double m22, double m23)
   {
      rotationMatrix.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      translationVector.set(m03, m13, m23);
   }

   public void setUnsafe(double m00, double m01, double m02, double m03, double m10, double m11, double m12, double m13, double m20, double m21, double m22, double m23)
   {
      rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      translationVector.set(m03, m13, m23);
   }

   @Override
   public void set(RigidBodyTransform other)
   {
      rotationMatrix.set(other.rotationMatrix);
      translationVector.set(other.translationVector);
   }

   public void setAndInvert(RigidBodyTransform other)
   {
      set(other);
      invert();
   }

   public void set(QuaternionBasedTransform quaternionBasedTransform)
   {
      rotationMatrix.set(quaternionBasedTransform.getQuaternion());
      translationVector.set(quaternionBasedTransform.getTranslationVector());
   }

   public void set(DenseMatrix64F matrix)
   {
      rotationMatrix.set(matrix);
      translationVector.set(matrix, 0, 3);
   }

   public void set(DenseMatrix64F matrix, int startRow, int startColumn)
   {
      rotationMatrix.set(matrix, startRow, startColumn);
      translationVector.set(matrix, startRow, startColumn + 3);
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

      rotationMatrix.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      translationVector.set(m03, m13, m23);
   }

   public void set(Matrix3DReadOnly<?> rotationMatrix, Tuple3DReadOnly translation)
   {
      this.rotationMatrix.set(rotationMatrix);
      translationVector.set(translation);
   }

   public void set(RotationMatrix rotationMatrix, Tuple3DReadOnly translation)
   {
      this.rotationMatrix.set(rotationMatrix);
      translationVector.set(translation);
   }

   public void set(RotationScaleMatrix rotationScaleMatrix, Tuple3DReadOnly translation)
   {
      rotationScaleMatrix.getRotation(rotationMatrix);
      translationVector.set(translation);
   }

   public void set(AxisAngleReadOnly<?> axisAngle, Tuple3DReadOnly translation)
   {
      rotationMatrix.set(axisAngle);
      translationVector.set(translation);
   }

   public void set(QuaternionReadOnly quaternion, Tuple3DReadOnly translation)
   {
      rotationMatrix.set(quaternion);
      translationVector.set(translation);
   }

   public void setIdentity()
   {
      rotationMatrix.setIdentity();
      translationVector.setToZero();
   }

   public void setRotation(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      rotationMatrix.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   public void setRotationUnsafe(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      rotationMatrix.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   public void setRotation(AxisAngleReadOnly<?> axisAngle)
   {
      rotationMatrix.set(axisAngle);
   }

   public void setRotation(DenseMatrix64F matrix)
   {
      rotationMatrix.set(matrix);
   }

   public void setRotation(QuaternionReadOnly quaternion)
   {
      rotationMatrix.set(quaternion);
   }

   public void setRotation(Matrix3DReadOnly<?> rotationMatrix)
   {
      this.rotationMatrix.set(rotationMatrix);
   }

   public void setRotationYaw(double yaw)
   {
      rotationMatrix.setToYawMatrix(yaw);
   }

   public void setRotationPitch(double pitch)
   {
      rotationMatrix.setToPitchMatrix(pitch);
   }

   public void setRotationRoll(double roll)
   {
      rotationMatrix.setToRollMatrix(roll);
   }

   public void setRotationYawPitchRoll(double yaw, double pitch, double roll)
   {
      rotationMatrix.setYawPitchRoll(yaw, pitch, roll);
   }

   public void setRotationEuler(Vector3DReadOnly eulerAngles)
   {
      rotationMatrix.setEuler(eulerAngles);
   }

   public void setRotationEuler(double rotX, double rotY, double rotZ)
   {
      rotationMatrix.setEuler(rotX, rotY, rotZ);
   }

   public void setRotationAndZeroTranslation(AxisAngleReadOnly<?> axisAngle)
   {
      setRotation(axisAngle);
      translationVector.setToZero();
   }

   public void setRotationAndZeroTranslation(DenseMatrix64F matrix)
   {
      setRotation(matrix);
      translationVector.setToZero();
   }

   public void setRotationAndZeroTranslation(QuaternionReadOnly quaternion)
   {
      setRotation(quaternion);
      translationVector.setToZero();
   }

   public void setRotationAndZeroTranslation(Matrix3DReadOnly<?> rotationMatrix)
   {
      setRotation(rotationMatrix);
      translationVector.setToZero();
   }

   public void setRotationYawAndZeroTranslation(double yaw)
   {
      setRotationYaw(yaw);
      translationVector.setToZero();
   }

   public void setRotationPitchAndZeroTranslation(double pitch)
   {
      setRotationPitch(pitch);
      translationVector.setToZero();
   }

   public void setRotationRollAndZeroTranslation(double roll)
   {
      setRotationRoll(roll);
      translationVector.setToZero();
   }

   public void setRotationYawPitchRollAndZeroTranslation(double yaw, double pitch, double roll)
   {
      setRotationYawPitchRoll(yaw, pitch, roll);
      translationVector.setToZero();
   }

   public void setRotationEulerAndZeroTranslation(Vector3DReadOnly eulerAngles)
   {
      setRotationEuler(eulerAngles);
      translationVector.setToZero();
   }

   public void setTranslation(Tuple3DReadOnly translation)
   {
      translationVector.set(translation);
   }

   public void setTranslation(double x, double y, double z)
   {
      translationVector.set(x, y, z);
   }

   public void setTranslationAndIdentityRotation(Tuple3DReadOnly translation)
   {
      setTranslation(translation);
      rotationMatrix.setIdentity();
   }

   public RotationMatrixReadOnly<?> getRotationMatrix()
   {
      return rotationMatrix;
   }

   public void getRotation(Matrix3DBasics<?> rotationMatrixToPack)
   {
      rotationMatrixToPack.set(rotationMatrix);
   }

   public void getRotation(RotationMatrix rotationMatrixToPack)
   {
      rotationMatrixToPack.set(rotationMatrix);
   }

   public void getRotation(RotationScaleMatrix rotationMatrixToPack)
   {
      rotationMatrixToPack.set(rotationMatrix);
   }

   public void getRotation(DenseMatrix64F matrixToPack)
   {
      rotationMatrix.get(matrixToPack);
   }

   public void getRotation(QuaternionBasics quaternionToPack)
   {
      rotationMatrix.get(quaternionToPack);
   }

   public void getRotation(AxisAngleBasics<?> axisAngleToPack)
   {
      axisAngleToPack.set(rotationMatrix);
   }

   public Vector3DReadOnly getTranslationVector()
   {
      return translationVector;
   }

   public void getTranslation(Tuple3DBasics translationToPack)
   {
      translationToPack.set(translationVector);
   }

   public void get(DenseMatrix64F matrixToPack)
   {
      rotationMatrix.get(matrixToPack);
      translationVector.get(matrixToPack, 0, 3);
      matrixToPack.set(3, 0, 0.0);
      matrixToPack.set(3, 1, 0.0);
      matrixToPack.set(3, 2, 0.0);
      matrixToPack.set(3, 3, 1.0);
   }

   public void get(DenseMatrix64F matrixToPack, int startRow, int startColumn)
   {
      rotationMatrix.get(startRow, startColumn, matrixToPack);
      translationVector.get(matrixToPack, startRow, startColumn + 3);
      startRow += 3;
      matrixToPack.set(startRow, startColumn++, 0.0);
      matrixToPack.set(startRow, startColumn++, 0.0);
      matrixToPack.set(startRow, startColumn++, 0.0);
      matrixToPack.set(startRow, startColumn, 1.0);
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

   public void get(QuaternionBasics quaternionToPack, Tuple3DBasics translationToPack)
   {
      rotationMatrix.get(quaternionToPack);
      translationToPack.set(translationVector);
   }

   public void get(Matrix3DBasics<?> rotationMarixToPack, Tuple3DBasics translationToPack)
   {
      rotationMarixToPack.set(rotationMatrix);
      translationToPack.set(translationVector);
   }

   public void get(RotationMatrix rotationMarixToPack, Tuple3DBasics translationToPack)
   {
      rotationMarixToPack.set(rotationMatrix);
      translationToPack.set(translationVector);
   }

   public void get(RotationScaleMatrix rotationMarixToPack, Tuple3DBasics translationToPack)
   {
      rotationMarixToPack.set(rotationMatrix);
      translationToPack.set(translationVector);
   }

   public void invert()
   {
      rotationMatrix.invert();
      rotationMatrix.transform(translationVector);
      translationVector.negate();
   }

   public void multiply(RigidBodyTransform other)
   {
      Matrix3DTools.addTransform(rotationMatrix, other.translationVector, translationVector);
      rotationMatrix.multiply(other.rotationMatrix);
   }

   public void preMultiply(RigidBodyTransform other)
   {
      other.rotationMatrix.transform(translationVector);
      translationVector.add(other.translationVector);
      rotationMatrix.preMultiply(other.rotationMatrix);
   }

   @Override
   public void transform(Point3DReadOnly pointOriginal, Point3DBasics pointTransformed)
   {
      rotationMatrix.transform(pointOriginal, pointTransformed);
      pointTransformed.add(translationVector);
   }

   @Override
   public void transform(Vector3DReadOnly vectorOriginal, Vector3DBasics vectorTransformed)
   {
      rotationMatrix.transform(vectorOriginal, vectorTransformed);
   }

   @Override
   public void transform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      rotationMatrix.transform(quaternionOriginal, quaternionTransformed);
   }

   @Override
   public void transform(Vector4DReadOnly vector4DOriginal, Vector4DBasics vector4DTransformed)
   {
      rotationMatrix.transform(vector4DOriginal, vector4DTransformed);
   }

   @Override
   public void transform(Point2DReadOnly point2DOriginal, Point2DBasics point2DTransformed, boolean checkIfTransformInXYPlane)
   {
      rotationMatrix.transform(point2DOriginal, point2DTransformed, checkIfTransformInXYPlane);
      point2DTransformed.add(translationVector.getX(), translationVector.getY());
   }

   @Override
   public void transform(Vector2DReadOnly vector2DOriginal, Vector2DBasics vector2DTransformed, boolean checkIfTransformInXYPlane)
   {
      rotationMatrix.transform(vector2DOriginal, vector2DTransformed, checkIfTransformInXYPlane);
   }

   @Override
   public void transform(RotationMatrixReadOnly<?> matrixOriginal, RotationMatrix matrixTransformed)
   {
      rotationMatrix.transform(matrixOriginal, matrixTransformed);
   }
   
   @Override
   public void transform(Matrix3DReadOnly<?> matrixOriginal, Matrix3D matrixTransformed)
   {
      rotationMatrix.transform(matrixOriginal, matrixTransformed);
   }

   @Override
   public void inverseTransform(Point3DReadOnly pointOriginal, Point3DBasics pointTransformed)
   {
      pointTransformed.set(pointOriginal);
      pointTransformed.sub(translationVector);
      rotationMatrix.inverseTransform(pointTransformed);
   }

   @Override
   public void inverseTransform(Vector3DReadOnly vectorOriginal, Vector3DBasics vectorTransformed)
   {
      rotationMatrix.inverseTransform(vectorOriginal, vectorTransformed);
   }

   @Override
   public void inverseTransform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      rotationMatrix.inverseTransform(quaternionOriginal, quaternionTransformed);
   }

   @Override
   public void inverseTransform(Vector4DReadOnly vector4DOriginal, Vector4DBasics vector4DTransformed)
   {
      rotationMatrix.inverseTransform(vector4DOriginal, vector4DTransformed);
   }

   @Override
   public void inverseTransform(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed, boolean checkIfTransformInXYPlane)
   {
      pointTransformed.set(pointOriginal);
      pointTransformed.sub(translationVector.getX(), translationVector.getY());
      rotationMatrix.inverseTransform(pointTransformed, checkIfTransformInXYPlane);
   }

   @Override
   public void inverseTransform(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed, boolean checkIfTransformInXYPlane)
   {
      rotationMatrix.inverseTransform(vectorOriginal, vectorTransformed, checkIfTransformInXYPlane);
   }

   @Override
   public void inverseTransform(RotationMatrixReadOnly<?> matrixOriginal, RotationMatrix matrixTransformed)
   {
      rotationMatrix.inverseTransform(matrixOriginal, matrixTransformed);
   }

   @Override
   public void inverseTransform(Matrix3DReadOnly<?> matrixOriginal, Matrix3D matrixTransformed)
   {
      rotationMatrix.inverseTransform(matrixOriginal, matrixTransformed);
   }

   public double getElement(int row, int column)
   {
      if (row < 3)
      {
         if (column < 3)
         {
            return rotationMatrix.getElement(row, column);
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
      return rotationMatrix.getM00();
   }

   public double getM01()
   {
      return rotationMatrix.getM01();
   }

   public double getM02()
   {
      return rotationMatrix.getM02();
   }

   public double getM03()
   {
      return translationVector.getX();
   }

   public double getM10()
   {
      return rotationMatrix.getM10();
   }

   public double getM11()
   {
      return rotationMatrix.getM11();
   }

   public double getM12()
   {
      return rotationMatrix.getM12();
   }

   public double getM13()
   {
      return translationVector.getY();
   }

   public double getM20()
   {
      return rotationMatrix.getM20();
   }

   public double getM21()
   {
      return rotationMatrix.getM21();
   }

   public double getM22()
   {
      return rotationMatrix.getM22();
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
   public boolean epsilonEquals(RigidBodyTransform other, double epsilon)
   {
      return rotationMatrix.epsilonEquals(other.rotationMatrix, epsilon) && translationVector.epsilonEquals(other.translationVector, epsilon);
   }

   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((RigidBodyTransform) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   public boolean equals(RigidBodyTransform other)
   {
      if (other == null)
         return false;
      else
         return rotationMatrix.equals(other.rotationMatrix) && translationVector.equals(other.translationVector);
   }

   @Override
   public String toString()
   {
      return TransformTools.toString(rotationMatrix, translationVector);
   }
}
