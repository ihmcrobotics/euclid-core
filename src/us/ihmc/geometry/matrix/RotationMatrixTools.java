package us.ihmc.geometry.matrix;

import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.geometry.tuple.interfaces.TupleBasics;
import us.ihmc.geometry.tuple.interfaces.TupleReadOnly;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple4D.QuaternionTools;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DReadOnly;

public abstract class RotationMatrixTools
{
   public static void multiply(RotationMatrixReadOnly<?> m1, RotationMatrixReadOnly<?> m2, RotationMatrix matrixToPack)
   {
      double m00 = m1.getM00() * m2.getM00() + m1.getM01() * m2.getM10() + m1.getM02() * m2.getM20();
      double m01 = m1.getM00() * m2.getM01() + m1.getM01() * m2.getM11() + m1.getM02() * m2.getM21();
      double m02 = m1.getM00() * m2.getM02() + m1.getM01() * m2.getM12() + m1.getM02() * m2.getM22();
      double m10 = m1.getM10() * m2.getM00() + m1.getM11() * m2.getM10() + m1.getM12() * m2.getM20();
      double m11 = m1.getM10() * m2.getM01() + m1.getM11() * m2.getM11() + m1.getM12() * m2.getM21();
      double m12 = m1.getM10() * m2.getM02() + m1.getM11() * m2.getM12() + m1.getM12() * m2.getM22();
      double m20 = m1.getM20() * m2.getM00() + m1.getM21() * m2.getM10() + m1.getM22() * m2.getM20();
      double m21 = m1.getM20() * m2.getM01() + m1.getM21() * m2.getM11() + m1.getM22() * m2.getM21();
      double m22 = m1.getM20() * m2.getM02() + m1.getM21() * m2.getM12() + m1.getM22() * m2.getM22();
      matrixToPack.setAndNormalize(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   public static void multiplyTransposeBoth(RotationMatrixReadOnly<?> m1, RotationMatrixReadOnly<?> m2, RotationMatrix matrixToPack)
   {
      double m00 = m1.getM00() * m2.getM00() + m1.getM10() * m2.getM01() + m1.getM20() * m2.getM02();
      double m01 = m1.getM00() * m2.getM10() + m1.getM10() * m2.getM11() + m1.getM20() * m2.getM12();
      double m02 = m1.getM00() * m2.getM20() + m1.getM10() * m2.getM21() + m1.getM20() * m2.getM22();
      double m10 = m1.getM01() * m2.getM00() + m1.getM11() * m2.getM01() + m1.getM21() * m2.getM02();
      double m11 = m1.getM01() * m2.getM10() + m1.getM11() * m2.getM11() + m1.getM21() * m2.getM12();
      double m12 = m1.getM01() * m2.getM20() + m1.getM11() * m2.getM21() + m1.getM21() * m2.getM22();
      double m20 = m1.getM02() * m2.getM00() + m1.getM12() * m2.getM01() + m1.getM22() * m2.getM02();
      double m21 = m1.getM02() * m2.getM10() + m1.getM12() * m2.getM11() + m1.getM22() * m2.getM12();
      double m22 = m1.getM02() * m2.getM20() + m1.getM12() * m2.getM21() + m1.getM22() * m2.getM22();
      matrixToPack.setAndNormalize(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   public static void multiplyTransposeLeft(RotationMatrixReadOnly<?> m1, RotationMatrixReadOnly<?> m2, RotationMatrix matrixToPack)
   {
      double m00 = m1.getM00() * m2.getM00() + m1.getM10() * m2.getM10() + m1.getM20() * m2.getM20();
      double m01 = m1.getM00() * m2.getM01() + m1.getM10() * m2.getM11() + m1.getM20() * m2.getM21();
      double m02 = m1.getM00() * m2.getM02() + m1.getM10() * m2.getM12() + m1.getM20() * m2.getM22();
      double m10 = m1.getM01() * m2.getM00() + m1.getM11() * m2.getM10() + m1.getM21() * m2.getM20();
      double m11 = m1.getM01() * m2.getM01() + m1.getM11() * m2.getM11() + m1.getM21() * m2.getM21();
      double m12 = m1.getM01() * m2.getM02() + m1.getM11() * m2.getM12() + m1.getM21() * m2.getM22();
      double m20 = m1.getM02() * m2.getM00() + m1.getM12() * m2.getM10() + m1.getM22() * m2.getM20();
      double m21 = m1.getM02() * m2.getM01() + m1.getM12() * m2.getM11() + m1.getM22() * m2.getM21();
      double m22 = m1.getM02() * m2.getM02() + m1.getM12() * m2.getM12() + m1.getM22() * m2.getM22();
      matrixToPack.setAndNormalize(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   public static void multiplyTransposeRight(RotationMatrixReadOnly<?> m1, RotationMatrixReadOnly<?> m2, RotationMatrix matrixToPack)
   {
      double m00 = m1.getM00() * m2.getM00() + m1.getM01() * m2.getM01() + m1.getM02() * m2.getM02();
      double m01 = m1.getM00() * m2.getM10() + m1.getM01() * m2.getM11() + m1.getM02() * m2.getM12();
      double m02 = m1.getM00() * m2.getM20() + m1.getM01() * m2.getM21() + m1.getM02() * m2.getM22();
      double m10 = m1.getM10() * m2.getM00() + m1.getM11() * m2.getM01() + m1.getM12() * m2.getM02();
      double m11 = m1.getM10() * m2.getM10() + m1.getM11() * m2.getM11() + m1.getM12() * m2.getM12();
      double m12 = m1.getM10() * m2.getM20() + m1.getM11() * m2.getM21() + m1.getM12() * m2.getM22();
      double m20 = m1.getM20() * m2.getM00() + m1.getM21() * m2.getM01() + m1.getM22() * m2.getM02();
      double m21 = m1.getM20() * m2.getM10() + m1.getM21() * m2.getM11() + m1.getM22() * m2.getM12();
      double m22 = m1.getM20() * m2.getM20() + m1.getM21() * m2.getM21() + m1.getM22() * m2.getM22();
      matrixToPack.setAndNormalize(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   public static void transform(RotationMatrixReadOnly<?> rotationMatrix, TupleReadOnly tupleOriginal, TupleBasics tupleTransformed)
   {
      rotationMatrix.normalize();
      Matrix3DTools.transform(rotationMatrix, tupleOriginal, tupleTransformed);
   }

   public static void addTransform(RotationMatrixReadOnly<?> rotationMatrix, TupleReadOnly tupleOriginal, TupleBasics tupleTransformed)
   {
      rotationMatrix.normalize();
      Matrix3DTools.addTransform(rotationMatrix, tupleOriginal, tupleTransformed);
   }

   public static void transform(RotationMatrixReadOnly<?> rotationMatrix, Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      rotationMatrix.normalize();
      Matrix3DTools.transform(rotationMatrix, tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   public static void transform(RotationMatrixReadOnly<?> rotationMatrix, QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      rotationMatrix.normalize();
      QuaternionTools.multiply(rotationMatrix, quaternionOriginal, quaternionTransformed);
   }

   public static void transform(RotationMatrixReadOnly<?> rotationMatrix, Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      rotationMatrix.normalize();
      Matrix3DTools.transform(rotationMatrix, vectorOriginal, vectorTransformed);
   }

   public static void transform(RotationMatrixReadOnly<?> rotationMatrix, RotationMatrixReadOnly<?> matrixOriginal, RotationMatrix matrixTransformed)
   {
      rotationMatrix.normalize();
      multiply(rotationMatrix, matrixOriginal, matrixTransformed);
   }

   public static void transform(RotationMatrixReadOnly<?> rotationMatrix, Matrix3DReadOnly<?> matrixOriginal, Matrix3D matrixTransformed)
   {
      rotationMatrix.normalize();
      Matrix3DTools.multiply(rotationMatrix, matrixOriginal, matrixTransformed);
      Matrix3DTools.multiplyTransposeRight(matrixTransformed, rotationMatrix, matrixTransformed);
   }

   public static void inverseTransform(RotationMatrixReadOnly<?> rotationMatrix, TupleReadOnly tupleOriginal, TupleBasics tupleTransformed)
   {
      rotationMatrix.normalize();
      double x = rotationMatrix.getM00() * tupleOriginal.getX() + rotationMatrix.getM10() * tupleOriginal.getY() + rotationMatrix.getM20() * tupleOriginal.getZ();
      double y = rotationMatrix.getM01() * tupleOriginal.getX() + rotationMatrix.getM11() * tupleOriginal.getY() + rotationMatrix.getM21() * tupleOriginal.getZ();
      double z = rotationMatrix.getM02() * tupleOriginal.getX() + rotationMatrix.getM12() * tupleOriginal.getY() + rotationMatrix.getM22() * tupleOriginal.getZ();
      tupleTransformed.set(x, y, z);
   }

   public static void inverseTransform(RotationMatrixReadOnly<?> rotationMatrix, Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed,
         boolean checkIfTransformInXYPlane)
   {
      rotationMatrix.normalize();

      if (checkIfTransformInXYPlane)
         rotationMatrix.checkIfMatrix2D();

      double x = rotationMatrix.getM00() * tupleOriginal.getX() + rotationMatrix.getM10() * tupleOriginal.getY();
      double y = rotationMatrix.getM01() * tupleOriginal.getX() + rotationMatrix.getM11() * tupleOriginal.getY();
      tupleTransformed.set(x, y);
   }

   public static void inverseTransform(RotationMatrixReadOnly<?> rotationMatrix, Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      rotationMatrix.normalize();
      double x = rotationMatrix.getM00() * vectorOriginal.getX() + rotationMatrix.getM10() * vectorOriginal.getY() + rotationMatrix.getM20() * vectorOriginal.getZ();
      double y = rotationMatrix.getM01() * vectorOriginal.getX() + rotationMatrix.getM11() * vectorOriginal.getY() + rotationMatrix.getM21() * vectorOriginal.getZ();
      double z = rotationMatrix.getM02() * vectorOriginal.getX() + rotationMatrix.getM12() * vectorOriginal.getY() + rotationMatrix.getM22() * vectorOriginal.getZ();
      vectorTransformed.set(x, y, z, vectorOriginal.getS());
   }
}
