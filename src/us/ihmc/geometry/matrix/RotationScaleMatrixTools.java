package us.ihmc.geometry.matrix;

import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationScaleMatrixReadOnly;
import us.ihmc.geometry.tuple.interfaces.TupleBasics;
import us.ihmc.geometry.tuple.interfaces.TupleReadOnly;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DReadOnly;

public abstract class RotationScaleMatrixTools
{
   /**
    * Performs:
    * <pre>
    *     | x  0  0 |
    * m = | 0  y  0 | * m
    *     | 0  0  z |
    * </pre>
    * 
    * @param x
    * @param y
    * @param z
    * @param matrixToScale
    */
   public static void preScaleMatrix(double x, double y, double z, Matrix3D matrixToScale)
   {
      preScaleMatrix(x, y, z, matrixToScale, matrixToScale);
   }

   public static void preScaleMatrix(double x, double y, double z, Matrix3DReadOnly matrixOriginal, Matrix3D matrixScaled)
   {
      double m00 = x * matrixOriginal.getM00();
      double m01 = x * matrixOriginal.getM01();
      double m02 = x * matrixOriginal.getM02();
      double m10 = y * matrixOriginal.getM10();
      double m11 = y * matrixOriginal.getM11();
      double m12 = y * matrixOriginal.getM12();
      double m20 = z * matrixOriginal.getM20();
      double m21 = z * matrixOriginal.getM21();
      double m22 = z * matrixOriginal.getM22();
      matrixScaled.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Performs:
    * <pre>
    *         | x  0  0 |
    * m = m * | 0  y  0 |
    *         | 0  0  z |
    * </pre>
    * 
    * @param x
    * @param y
    * @param z
    * @param matrixToScale
    */
   public static void postScaleMatrix(double x, double y, double z, Matrix3D matrixToScale)
   {
      postScaleMatrix(x, y, z, matrixToScale, matrixToScale);
   }

   public static void postScaleMatrix(double x, double y, double z, Matrix3DReadOnly matrixOriginal, Matrix3D matrixScaled)
   {
      double m00 = x * matrixOriginal.getM00();
      double m01 = y * matrixOriginal.getM01();
      double m02 = z * matrixOriginal.getM02();
      double m10 = x * matrixOriginal.getM10();
      double m11 = y * matrixOriginal.getM11();
      double m12 = z * matrixOriginal.getM12();
      double m20 = x * matrixOriginal.getM20();
      double m21 = y * matrixOriginal.getM21();
      double m22 = z * matrixOriginal.getM22();
      matrixScaled.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   public static void transform(RotationScaleMatrixReadOnly rotationScaleMatrix, TupleReadOnly tupleOriginal, TupleBasics tupleTransformed)
   {
      tupleTransformed.setX(tupleOriginal.getX() * rotationScaleMatrix.getScaleX());
      tupleTransformed.setY(tupleOriginal.getY() * rotationScaleMatrix.getScaleY());
      tupleTransformed.setZ(tupleOriginal.getZ() * rotationScaleMatrix.getScaleZ());

      RotationMatrixReadOnly rotationMatrix = rotationScaleMatrix.getRotationMatrix();
      RotationMatrixTools.transform(rotationMatrix, tupleTransformed, tupleTransformed);
   }

   public static void transform(RotationScaleMatrixReadOnly rotationScaleMatrix, Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfRotationInXYPlane)
   {
      tupleTransformed.setX(rotationScaleMatrix.getScaleX() * tupleOriginal.getX());
      tupleTransformed.setY(rotationScaleMatrix.getScaleY() * tupleOriginal.getY());

      RotationMatrixReadOnly rotationMatrix = rotationScaleMatrix.getRotationMatrix();
      RotationMatrixTools.transform(rotationMatrix, tupleTransformed, tupleTransformed, checkIfRotationInXYPlane);
   }

   public static void transform(RotationScaleMatrixReadOnly rotationScaleMatrix, QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      RotationMatrixReadOnly rotationMatrix = rotationScaleMatrix.getRotationMatrix();
      RotationMatrixTools.transform(rotationMatrix, quaternionOriginal, quaternionTransformed);
   }

   public static void transform(RotationScaleMatrixReadOnly rotationScaleMatrix, Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      vectorTransformed.setX(vectorOriginal.getX() * rotationScaleMatrix.getScaleX());
      vectorTransformed.setY(vectorOriginal.getY() * rotationScaleMatrix.getScaleY());
      vectorTransformed.setZ(vectorOriginal.getZ() * rotationScaleMatrix.getScaleZ());
      vectorTransformed.setS(vectorOriginal.getS());

      RotationMatrixReadOnly rotationMatrix = rotationScaleMatrix.getRotationMatrix();
      RotationMatrixTools.transform(rotationMatrix, vectorTransformed, vectorTransformed);
   }

   public static void transform(RotationScaleMatrixReadOnly rotationScaleMatrix, RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
   {
      RotationMatrixReadOnly rotationMatrix = rotationScaleMatrix.getRotationMatrix();
      RotationMatrixTools.transform(rotationMatrix, matrixOriginal, matrixTransformed);
   }

   public static void transform(RotationScaleMatrixReadOnly rotationScaleMatrix, Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
   {
      double x = rotationScaleMatrix.getScaleX();
      double y = rotationScaleMatrix.getScaleY();
      double z = rotationScaleMatrix.getScaleZ();
      preScaleMatrix(x, y, z, matrixOriginal, matrixTransformed);
      postScaleMatrix(1.0 / x, 1.0 / y, 1.0 / z, matrixTransformed);
      RotationMatrixReadOnly rotationMatrix = rotationScaleMatrix.getRotationMatrix();
      RotationMatrixTools.transform(rotationMatrix, matrixTransformed, matrixTransformed);
   }

   public static void inverseTransform(RotationScaleMatrixReadOnly rotationScaleMatrix, TupleReadOnly tupleOriginal, TupleBasics tupleTransformed)
   {
      RotationMatrixReadOnly rotationMatrix = rotationScaleMatrix.getRotationMatrix();
      RotationMatrixTools.inverseTransform(rotationMatrix, tupleOriginal, tupleTransformed);

      tupleTransformed.setX(tupleTransformed.getX() / rotationScaleMatrix.getScaleX());
      tupleTransformed.setY(tupleTransformed.getY() / rotationScaleMatrix.getScaleY());
      tupleTransformed.setZ(tupleTransformed.getZ() / rotationScaleMatrix.getScaleZ());
   }

   public static void inverseTransform(RotationScaleMatrixReadOnly rotationScaleMatrix, Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfRotationInXYPlane)
   {
      RotationMatrixReadOnly rotationMatrix = rotationScaleMatrix.getRotationMatrix();
      RotationMatrixTools.inverseTransform(rotationMatrix, tupleOriginal, tupleTransformed, checkIfRotationInXYPlane);

      tupleTransformed.setX(tupleTransformed.getX() / rotationScaleMatrix.getScaleX());
      tupleTransformed.setY(tupleTransformed.getY() / rotationScaleMatrix.getScaleY());
   }

   public static void inverseTransform(RotationScaleMatrixReadOnly rotationScaleMatrix, Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      RotationMatrixReadOnly rotationMatrix = rotationScaleMatrix.getRotationMatrix();
      RotationMatrixTools.inverseTransform(rotationMatrix, vectorOriginal, vectorTransformed);

      vectorTransformed.setX(vectorTransformed.getX() / rotationScaleMatrix.getScaleX());
      vectorTransformed.setY(vectorTransformed.getY() / rotationScaleMatrix.getScaleY());
      vectorTransformed.setZ(vectorTransformed.getZ() / rotationScaleMatrix.getScaleZ());
   }
}
