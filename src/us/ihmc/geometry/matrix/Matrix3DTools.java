package us.ihmc.geometry.matrix;

import us.ihmc.geometry.exceptions.NotAMatrix2DException;
import us.ihmc.geometry.exceptions.SingularMatrixException;
import us.ihmc.geometry.matrix.interfaces.Matrix3DBasics;
import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationScaleMatrixReadOnly;
import us.ihmc.geometry.tuple.interfaces.TupleBasics;
import us.ihmc.geometry.tuple.interfaces.TupleReadOnly;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DReadOnly;

public abstract class Matrix3DTools
{
   private static final double EPS_NORM = 2.107342e-08;
   static final double EPS_INVERT = 1.0e-16;

   /**
    * Performs an in-place inversion of the given matrix such that: m = m<sup>-1</sup>.
    * <p>
    * <b> This method is meant for internal use only.</b>
    * <p>
    * 
    * @param matrixToInvert the matrix to invert, not null, modified.
    * @return true if the inversion succeeds, false if the matrix is singular.
    */
   public static boolean invert(Matrix3D matrixToInvert)
   {
      return invert(matrixToInvert, matrixToInvert);
   }

   /**
    * Computes the inverse of <code>matrix</code> and stores the result in <code>inverseToPack</code>.
    * The matrices can be the same object.
    * <p>
    * <b> This method is meant for internal use only.</b>
    * <p>
    * 
    * @param matrix the matrix to compute the inverse of, not null, not modified.
    * @param inverseToPack the result to pack, not null, modified.
    * @return true if the inversion succeeds, false if the matrix is singular.
    */
   public static boolean invert(Matrix3DReadOnly matrix, Matrix3D inverseToPack)
   {
      double det = Matrix3DFeatures.determinant(matrix);

      if (Math.abs(det) >= EPS_INVERT)
      {
         det = 1.0 / det;
         double m00 =  (matrix.getM11() * matrix.getM22() - matrix.getM21() * matrix.getM12()) * det;
         double m01 = -(matrix.getM01() * matrix.getM22() - matrix.getM21() * matrix.getM02()) * det;
         double m02 =  (matrix.getM01() * matrix.getM12() - matrix.getM11() * matrix.getM02()) * det;
         double m10 = -(matrix.getM10() * matrix.getM22() - matrix.getM20() * matrix.getM12()) * det;
         double m11 =  (matrix.getM00() * matrix.getM22() - matrix.getM20() * matrix.getM02()) * det;
         double m12 = -(matrix.getM00() * matrix.getM12() - matrix.getM10() * matrix.getM02()) * det;
         double m20 =  (matrix.getM10() * matrix.getM21() - matrix.getM20() * matrix.getM11()) * det;
         double m21 = -(matrix.getM00() * matrix.getM21() - matrix.getM20() * matrix.getM01()) * det;
         double m22 =  (matrix.getM00() * matrix.getM11() - matrix.getM10() * matrix.getM01()) * det;
         inverseToPack.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
         return true;
      }

      return false;
   }

   /**
    * Performs the multiplication: <code>m1</code> * <code>m2</code> and stores the result in <code>matrixToPack</code>.
    * <p>
    * All the matrices can be the same object.
    * <p>
    * Before the multiplication is performed, this calls {@linkplain Matrix3DReadOnly#checkIfMatrixProper()} on both <code>m1</code> and <code>m2</code>.
    * 
    * @param m1 the first matrix, not null, not modified.
    * @param m2 the second matrix, not null, not modified.
    * @param matrixToPack the result of the multiplication, not null, modified.
    */
   public static void multiply(Matrix3DReadOnly m1, Matrix3DReadOnly m2, Matrix3D matrixToPack)
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
      matrixToPack.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Performs the multiplication: <code>m1</code><sup>T</sup> * <code>m2</code><sup>T</sup> and stores the result in <code>matrixToPack</code>.
    * <p>
    * All the matrices can be the same object.
    * <p>
    * Before the multiplication is performed, this calls {@linkplain Matrix3DReadOnly#checkIfMatrixProper()} on both <code>m1</code> and <code>m2</code>.
    * 
    * @param m1 the first matrix, not null, not modified.
    * @param m2 the second matrix, not null, not modified.
    * @param matrixToPack the result of the multiplication, not null, modified.
    */
   public static void multiplyTransposeBoth(Matrix3DReadOnly m1, Matrix3DReadOnly m2, Matrix3D matrixToPack)
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
      matrixToPack.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   public static void multiplyInvertBoth(Matrix3DReadOnly m1, Matrix3DReadOnly m2, Matrix3D matrixToPack)
   {
      multiply(m2, m1, matrixToPack);
      boolean success = invert(matrixToPack);
      if (!success)
         throw new SingularMatrixException(matrixToPack);
   }

   /**
    * Performs the multiplication: <code>m1</code><sup>T</sup> * <code>m2</code> and stores the result in <code>matrixToPack</code>.
    * <p>
    * All the matrices can be the same object.
    * <p>
    * Before the multiplication is performed, this calls {@linkplain Matrix3DReadOnly#checkIfMatrixProper()} on both <code>m1</code> and <code>m2</code>.
    * 
    * @param m1 the first matrix, not null, not modified.
    * @param m2 the second matrix, not null, not modified.
    * @param matrixToPack the result of the multiplication, not null, modified.
    */
   public static void multiplyTransposeLeft(Matrix3DReadOnly m1, Matrix3DReadOnly m2, Matrix3D matrixToPack)
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
      matrixToPack.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   public static void multiplyInvertLeft(Matrix3DReadOnly m1, Matrix3DReadOnly m2, Matrix3D matrixToPack)
   {
      double det = Matrix3DFeatures.determinant(m1);
      if (Math.abs(det) < EPS_INVERT)
         throw new SingularMatrixException(m1);

      det = 1.0 / det;
      double invM00 =  (m1.getM11() * m1.getM22() - m1.getM21() * m1.getM12()) * det;
      double invM01 = -(m1.getM01() * m1.getM22() - m1.getM21() * m1.getM02()) * det;
      double invM02 =  (m1.getM01() * m1.getM12() - m1.getM11() * m1.getM02()) * det;
      double invM10 = -(m1.getM10() * m1.getM22() - m1.getM20() * m1.getM12()) * det;
      double invM11 =  (m1.getM00() * m1.getM22() - m1.getM20() * m1.getM02()) * det;
      double invM12 = -(m1.getM00() * m1.getM12() - m1.getM10() * m1.getM02()) * det;
      double invM20 =  (m1.getM10() * m1.getM21() - m1.getM20() * m1.getM11()) * det;
      double invM21 = -(m1.getM00() * m1.getM21() - m1.getM20() * m1.getM01()) * det;
      double invM22 =  (m1.getM00() * m1.getM11() - m1.getM10() * m1.getM01()) * det;

      double m00 = invM00 * m2.getM00() + invM01 * m2.getM10() + invM02 * m2.getM20();
      double m01 = invM00 * m2.getM01() + invM01 * m2.getM11() + invM02 * m2.getM21();
      double m02 = invM00 * m2.getM02() + invM01 * m2.getM12() + invM02 * m2.getM22();
      double m10 = invM10 * m2.getM00() + invM11 * m2.getM10() + invM12 * m2.getM20();
      double m11 = invM10 * m2.getM01() + invM11 * m2.getM11() + invM12 * m2.getM21();
      double m12 = invM10 * m2.getM02() + invM11 * m2.getM12() + invM12 * m2.getM22();
      double m20 = invM20 * m2.getM00() + invM21 * m2.getM10() + invM22 * m2.getM20();
      double m21 = invM20 * m2.getM01() + invM21 * m2.getM11() + invM22 * m2.getM21();
      double m22 = invM20 * m2.getM02() + invM21 * m2.getM12() + invM22 * m2.getM22();
      matrixToPack.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   public static void multiplyInvertLeftUnsafe(RotationMatrixReadOnly m1, Matrix3DReadOnly m2, Matrix3D matrixToPack)
   {
      multiplyTransposeLeft(m1, m2, matrixToPack);
   }

   public static void multiplyInvertLeft(RotationScaleMatrixReadOnly m1, Matrix3DReadOnly m2, Matrix3D matrixToPack)
   {
      multiplyTransposeLeft(m1.getRotationMatrix(), m2, matrixToPack);
      RotationScaleMatrixTools.preScaleMatrix(1.0 / m1.getScaleX(), 1.0 / m1.getScaleY(), 1.0 / m1.getScaleZ(), matrixToPack);
   }

   /**
    * Performs the multiplication: <code>m1</code> * <code>m2</code><sup>T</sup> and stores the result in <code>matrixToPack</code>.
    * <p>
    * All the matrices can be the same object.
    * <p>
    * Before the multiplication is performed, this calls {@linkplain Matrix3DReadOnly#checkIfMatrixProper()} on both <code>m1</code> and <code>m2</code>.
    * 
    * @param m1 the first matrix, not null, not modified.
    * @param m2 the second matrix, not null, not modified.
    * @param matrixToPack the result of the multiplication, not null, modified.
    */
   public static void multiplyTransposeRight(Matrix3DReadOnly m1, Matrix3DReadOnly m2, Matrix3D matrixToPack)
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
      matrixToPack.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   public static void multiplyInvertRight(Matrix3DReadOnly m1, Matrix3DReadOnly m2, Matrix3D matrixToPack)
   {
      double det = Matrix3DFeatures.determinant(m2);
      if (Math.abs(det) < EPS_INVERT)
         throw new SingularMatrixException(m2);

      det = 1.0 / det;
      double invM00 =  (m2.getM11() * m2.getM22() - m2.getM21() * m2.getM12()) * det;
      double invM01 = -(m2.getM01() * m2.getM22() - m2.getM21() * m2.getM02()) * det;
      double invM02 =  (m2.getM01() * m2.getM12() - m2.getM11() * m2.getM02()) * det;
      double invM10 = -(m2.getM10() * m2.getM22() - m2.getM20() * m2.getM12()) * det;
      double invM11 =  (m2.getM00() * m2.getM22() - m2.getM20() * m2.getM02()) * det;
      double invM12 = -(m2.getM00() * m2.getM12() - m2.getM10() * m2.getM02()) * det;
      double invM20 =  (m2.getM10() * m2.getM21() - m2.getM20() * m2.getM11()) * det;
      double invM21 = -(m2.getM00() * m2.getM21() - m2.getM20() * m2.getM01()) * det;
      double invM22 =  (m2.getM00() * m2.getM11() - m2.getM10() * m2.getM01()) * det;

      double m00 = m1.getM00() * invM00 + m1.getM01() * invM10 + m1.getM02() * invM20;
      double m01 = m1.getM00() * invM01 + m1.getM01() * invM11 + m1.getM02() * invM21;
      double m02 = m1.getM00() * invM02 + m1.getM01() * invM12 + m1.getM02() * invM22;
      double m10 = m1.getM10() * invM00 + m1.getM11() * invM10 + m1.getM12() * invM20;
      double m11 = m1.getM10() * invM01 + m1.getM11() * invM11 + m1.getM12() * invM21;
      double m12 = m1.getM10() * invM02 + m1.getM11() * invM12 + m1.getM12() * invM22;
      double m20 = m1.getM20() * invM00 + m1.getM21() * invM10 + m1.getM22() * invM20;
      double m21 = m1.getM20() * invM01 + m1.getM21() * invM11 + m1.getM22() * invM21;
      double m22 = m1.getM20() * invM02 + m1.getM21() * invM12 + m1.getM22() * invM22;
      matrixToPack.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   public static void multiplyInvertRight(Matrix3DReadOnly m1, RotationMatrixReadOnly m2, Matrix3D matrixToPack)
   {
      multiplyTransposeRight(m1, m2, matrixToPack);
   }
   
   public static void multiplyInvertRight(Matrix3DReadOnly m1, RotationScaleMatrixReadOnly m2, Matrix3D matrixToPack)
   {
      RotationScaleMatrixTools.postScaleMatrix(1.0 / m2.getScaleX(), 1.0 / m2.getScaleY(), 1.0 / m2.getScaleZ(), m1, matrixToPack);
      multiplyTransposeRight(matrixToPack, m2.getRotationMatrix(), matrixToPack);
   }

   /**
    * Orthonormalization of the rotation matrix using Gram-Schmidt method.
    * 
    * @param matrixToNormalize the matrix to normalize, not null, modified.
    */
   public static void normalize(Matrix3DBasics matrixToNormalize)
   {
      double m00 = matrixToNormalize.getM00();
      double m01 = matrixToNormalize.getM01();
      double m02 = matrixToNormalize.getM02();
      double m10 = matrixToNormalize.getM10();
      double m11 = matrixToNormalize.getM11();
      double m12 = matrixToNormalize.getM12();
      double m20 = matrixToNormalize.getM20();
      double m21 = matrixToNormalize.getM21();
      double m22 = matrixToNormalize.getM22();

      double xdoty = m00 * m01 + m10 * m11 + m20 * m21;
      double xdotx = m00 * m00 + m10 * m10 + m20 * m20;
      double tmp = xdoty / xdotx;

      m01 -= tmp * m00;
      m11 -= tmp * m10;
      m21 -= tmp * m20;

      double zdoty = m02 * m01 + m12 * m11 + m22 * m21;
      double zdotx = m02 * m00 + m12 * m10 + m22 * m20;
      double ydoty = m01 * m01 + m11 * m11 + m21 * m21;

      tmp = zdotx / xdotx;
      double tmp1 = zdoty / ydoty;

      m02 -= tmp * m00 + tmp1 * m01;
      m12 -= tmp * m10 + tmp1 * m11;
      m22 -= tmp * m20 + tmp1 * m21;

      // Compute orthogonalized vector magnitudes and normalize
      double invMagX = m00 * m00 + m10 * m10 + m20 * m20;
      double invMagY = m01 * m01 + m11 * m11 + m21 * m21;
      double invMagZ = m02 * m02 + m12 * m12 + m22 * m22;

      if (Math.abs(1.0 - invMagX) < EPS_NORM)
         invMagX = 2.0 / (1.0 + invMagX);
      else
         invMagX = 1.0 / Math.sqrt(invMagX);

      if (Math.abs(1.0 - invMagY) < EPS_NORM)
         invMagY = 2.0 / (1.0 + invMagY);
      else
         invMagY = 1.0 / Math.sqrt(invMagY);

      if (Math.abs(1.0 - invMagZ) < EPS_NORM)
         invMagZ = 2.0 / (1.0 + invMagZ);
      else
         invMagZ = 1.0 / Math.sqrt(invMagZ);

      m00 *= invMagX;
      m01 *= invMagY;
      m02 *= invMagZ;
      m10 *= invMagX;
      m11 *= invMagY;
      m12 *= invMagZ;
      m20 *= invMagX;
      m21 *= invMagY;
      m22 *= invMagZ;
      matrixToNormalize.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Performs a transformation of <code>tupleOriginal</code> using the given matrix and stores the result in <code>tupleTransformed</code>:
    * <p>
    * <code>tupleTransformed</code> = <code>matrix</code> * <code>tupleOriginal</code>.
    * <p>
    * Before the transformation is performed, this calls {@linkplain Matrix3DReadOnly#checkIfMatrixProper()} on the given matrix.
    * <p>
    * Both tuples can be the same instance to perform in-place transformation.
    * 
    * @param matrix the matrix used to transform <code>tupleOriginal</code>, not null, not modified.
    * @param tupleOriginal the original tuple to use for the transformation, not null, not modified.
    * @param tupleTransformed the tuple used to store the result of the transformation, not null, modified.
    */
   public static void transform(Matrix3DReadOnly matrix, TupleReadOnly tupleOriginal, TupleBasics tupleTransformed)
   {
      double x = matrix.getM00() * tupleOriginal.getX() + matrix.getM01() * tupleOriginal.getY() + matrix.getM02() * tupleOriginal.getZ();
      double y = matrix.getM10() * tupleOriginal.getX() + matrix.getM11() * tupleOriginal.getY() + matrix.getM12() * tupleOriginal.getZ();
      double z = matrix.getM20() * tupleOriginal.getX() + matrix.getM21() * tupleOriginal.getY() + matrix.getM22() * tupleOriginal.getZ();
      tupleTransformed.set(x, y, z);
   }

   /**
    * Performs a transformation of <code>tupleOriginal</code> using the given matrix and add the result to <code>tupleTransformed</code>:
    * <p>
    * <code>tupleTransformed</code> = <code>tupleTransformed</code> + <code>matrix</code> * <code>tupleOriginal</code>.
    * <p>
    * Before the transformation is performed, this calls {@linkplain Matrix3DReadOnly#checkIfMatrixProper()} on the given matrix.
    * 
    * @param matrix the matrix used to transform <code>tupleOriginal</code>, not null, not modified.
    * @param tupleOriginal the original tuple to use for the transformation, not null, not modified.
    * @param tupleTransformed the tuple to which the result of the transformation is added to, not null, modified.
    */
   public static void addTransform(Matrix3DReadOnly matrix, TupleReadOnly tupleOriginal, TupleBasics tupleTransformed)
   {
      double x = matrix.getM00() * tupleOriginal.getX() + matrix.getM01() * tupleOriginal.getY() + matrix.getM02() * tupleOriginal.getZ();
      double y = matrix.getM10() * tupleOriginal.getX() + matrix.getM11() * tupleOriginal.getY() + matrix.getM12() * tupleOriginal.getZ();
      double z = matrix.getM20() * tupleOriginal.getX() + matrix.getM21() * tupleOriginal.getY() + matrix.getM22() * tupleOriginal.getZ();
      tupleTransformed.add(x, y, z);
   }

   /**
    * Performs a transformation of <code>tupleOriginal</code> using the given matrix and stores the result in <code>tupleTransformed</code>:
    * <p>
    * <code>tupleTransformed</code> = <code>matrix</code> * <code>tupleOriginal</code>.
    * <p>
    * Before the transformation is performed, this calls {@linkplain Matrix3DReadOnly#checkIfMatrixProper()} on the given matrix.
    * <p>
    * Before the transformation is performed, if <code>checkIfTransformInXYPlane</code> equals true, this verify that the matrix is a 2D transformation matrix
    * using {@link Matrix3DFeatures#checkIfMatrix2D(Matrix3DReadOnly)}.
    * <p>
    * Both tuples can be the same instance to perform in-place transformation.
    * 
    * @param matrix the matrix used to transform <code>tupleOriginal</code>, not null, not modified.
    * @param tupleOriginal the original tuple to use for the transformation, not null, not modified.
    * @param tupleTransformed the tuple used to stored the result of the transformation, not null, modified.
    * @param checkIfTransformInXYPlane whether {@link Matrix3DFeatures#checkIfMatrix2D(Matrix3DReadOnly)} needs to be called on the matrix.
    * @throws NotAMatrix2DException if the matrix is not a 2D matrix and <code>checkIfTransformInXYPlane</code> is true.
    */
   public static void transform(Matrix3DReadOnly matrix, Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      if (checkIfTransformInXYPlane)
         Matrix3DFeatures.checkIfMatrix2D(matrix);
      double x = matrix.getM00() * tupleOriginal.getX() + matrix.getM01() * tupleOriginal.getY();
      double y = matrix.getM10() * tupleOriginal.getX() + matrix.getM11() * tupleOriginal.getY();
      tupleTransformed.set(x, y);
   }

   public static void transform(Matrix3DReadOnly matrix, Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      double x = matrix.getM00() * vectorOriginal.getX() + matrix.getM01() * vectorOriginal.getY() + matrix.getM02() * vectorOriginal.getZ();
      double y = matrix.getM10() * vectorOriginal.getX() + matrix.getM11() * vectorOriginal.getY() + matrix.getM12() * vectorOriginal.getZ();
      double z = matrix.getM20() * vectorOriginal.getX() + matrix.getM21() * vectorOriginal.getY() + matrix.getM22() * vectorOriginal.getZ();
      vectorTransformed.set(x, y, z, vectorOriginal.getS());
   }

   /**
    * Performs a transformation of <code>matrixOriginal</code> using <code>matrix</code> and stores the result in <code>matrixTransformed</code>:
    * <p>
    * <code>matrixTransformed</code> = <code>matrix</code> * <code>matrixOriginal</code> * <code>matrix</code><sup>T</sup>.
    * <p>
    * <b> This is different from concatenating orientations.</b>
    * <p>
    * Before the transformation is performed, this calls {@linkplain Matrix3DReadOnly#checkIfMatrixProper()} on <code>matrixOriginal</code> and <code>matrix</code>.
    * <p>
    * <code>matrixOriginal</code> and <code>matrixTransformed</code> can be the same instance to perform in-place transformation.
    * 
    * @param matrix the matrix used to transform <code>matrixOriginal</code>, not null, not modified.
    * @param matrixOriginal the original matrix to use for the transformation, not null, not modified.
    * @param matrixTransformed the matrix used to stored the result of the transformation, not null, modified.
    */
   public static void transform(Matrix3DReadOnly matrix, Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
   {
      multiply(matrix, matrixOriginal, matrixTransformed);
      multiplyInvertRight(matrixTransformed, matrix, matrixTransformed);
   }

   public static void inverseTransform(Matrix3DReadOnly matrix, TupleReadOnly tupleOriginal, TupleBasics tupleTransformed)
   {
      double det = Matrix3DFeatures.determinant(matrix);
      if (Math.abs(det) < EPS_INVERT)
         throw new SingularMatrixException(matrix);

      det = 1.0 / det;
      double invM00 =  (matrix.getM11() * matrix.getM22() - matrix.getM21() * matrix.getM12()) * det;
      double invM01 = -(matrix.getM01() * matrix.getM22() - matrix.getM21() * matrix.getM02()) * det;
      double invM02 =  (matrix.getM01() * matrix.getM12() - matrix.getM11() * matrix.getM02()) * det;
      double invM10 = -(matrix.getM10() * matrix.getM22() - matrix.getM20() * matrix.getM12()) * det;
      double invM11 =  (matrix.getM00() * matrix.getM22() - matrix.getM20() * matrix.getM02()) * det;
      double invM12 = -(matrix.getM00() * matrix.getM12() - matrix.getM10() * matrix.getM02()) * det;
      double invM20 =  (matrix.getM10() * matrix.getM21() - matrix.getM20() * matrix.getM11()) * det;
      double invM21 = -(matrix.getM00() * matrix.getM21() - matrix.getM20() * matrix.getM01()) * det;
      double invM22 =  (matrix.getM00() * matrix.getM11() - matrix.getM10() * matrix.getM01()) * det;

      double x = invM00 * tupleOriginal.getX() + invM01 * tupleOriginal.getY() + invM02 * tupleOriginal.getZ();
      double y = invM10 * tupleOriginal.getX() + invM11 * tupleOriginal.getY() + invM12 * tupleOriginal.getZ();
      double z = invM20 * tupleOriginal.getX() + invM21 * tupleOriginal.getY() + invM22 * tupleOriginal.getZ();
      tupleTransformed.set(x, y, z);
   }

   public static void inverseTransform(Matrix3DReadOnly matrix, Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      if (checkIfTransformInXYPlane)
         Matrix3DFeatures.checkIfMatrix2D(matrix);

      // Compute only the determinant of the sub matrix that transforms in the XY plane.
      double det = matrix.getM00() * matrix.getM11() - matrix.getM10() * matrix.getM01(); //determinant(matrix);
      if (Math.abs(det) < EPS_INVERT)
         throw new SingularMatrixException(matrix);

      det = 1.0 / det;
      double invM00 =  matrix.getM11() * det;
      double invM01 = -matrix.getM01() * det;
      double invM10 = -matrix.getM10() * det;
      double invM11 =  matrix.getM00() * det;

      double x = invM00 * tupleOriginal.getX() + invM01 * tupleOriginal.getY();
      double y = invM10 * tupleOriginal.getX() + invM11 * tupleOriginal.getY();
      tupleTransformed.set(x, y);
   }

   public static void inverseTransform(Matrix3DReadOnly matrix, Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      double det = Matrix3DFeatures.determinant(matrix);
      if (Math.abs(det) < EPS_INVERT)
         throw new SingularMatrixException(matrix);

      det = 1.0 / det;
      double invM00 =  (matrix.getM11() * matrix.getM22() - matrix.getM21() * matrix.getM12()) * det;
      double invM01 = -(matrix.getM01() * matrix.getM22() - matrix.getM21() * matrix.getM02()) * det;
      double invM02 =  (matrix.getM01() * matrix.getM12() - matrix.getM11() * matrix.getM02()) * det;
      double invM10 = -(matrix.getM10() * matrix.getM22() - matrix.getM20() * matrix.getM12()) * det;
      double invM11 =  (matrix.getM00() * matrix.getM22() - matrix.getM20() * matrix.getM02()) * det;
      double invM12 = -(matrix.getM00() * matrix.getM12() - matrix.getM10() * matrix.getM02()) * det;
      double invM20 =  (matrix.getM10() * matrix.getM21() - matrix.getM20() * matrix.getM11()) * det;
      double invM21 = -(matrix.getM00() * matrix.getM21() - matrix.getM20() * matrix.getM01()) * det;
      double invM22 =  (matrix.getM00() * matrix.getM11() - matrix.getM10() * matrix.getM01()) * det;

      double x = invM00 * vectorOriginal.getX() + invM01 * vectorOriginal.getY() + invM02 * vectorOriginal.getZ();
      double y = invM10 * vectorOriginal.getX() + invM11 * vectorOriginal.getY() + invM12 * vectorOriginal.getZ();
      double z = invM20 * vectorOriginal.getX() + invM21 * vectorOriginal.getY() + invM22 * vectorOriginal.getZ();
      vectorTransformed.set(x, y, z, vectorOriginal.getS());
   }

   /**
    * Find and return max of the three arguments.
    * @param a
    * @param b
    * @param c
    * @return
    */
   public static final double max(double a, double b, double c)
   {
      if (a > b)
         return a > c ? a : c;
      else
         return b > c ? b : c;
   }
}
