package us.ihmc.euclid.matrix;

import us.ihmc.euclid.exceptions.SingularMatrixException;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.matrix.interfaces.CommonMatrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationScaleMatrixReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tools.Matrix3DTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

/**
 * A {@code Matrix3D} is a 3-by-3 matrix used for general linear applications.
 * <p>
 * This version of 3D matrix uses double precision fields to save the value of each component. It is
 * meant for garbage free usage.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class Matrix3D implements CommonMatrix3DBasics, GeometryObject<Matrix3D>
{
   /** The 1st row 1st column coefficient of this matrix. */
   private double m00;
   /** The 1st row 2nd column coefficient of this matrix. */
   private double m01;
   /** The 1st row 3rd column coefficient of this matrix. */
   private double m02;
   /** The 2nd row 1st column coefficient of this matrix. */
   private double m10;
   /** The 2nd row 2nd column coefficient of this matrix. */
   private double m11;
   /** The 2nd row 3rd column coefficient of this matrix. */
   private double m12;
   /** The 3rd row 1st column coefficient of this matrix. */
   private double m20;
   /** The 3rd row 2nd column coefficient of this matrix. */
   private double m21;
   /** The 3rd row 3rd column coefficient of this matrix. */
   private double m22;

   /**
    * Creates a new 3D matrix with all its coefficients set to zero.
    */
   public Matrix3D()
   {
   }

   /**
    * Creates a new 3D matrix and initializes it from the given array.
    *
    * <pre>
    *        / matrixArray[0]  matrixArray[1]  matrixArray[2] \
    * this = | matrixArray[3]  matrixArray[4]  matrixArray[5] |
    *        \ matrixArray[6]  matrixArray[7]  matrixArray[8] /
    * </pre>
    *
    * @param matrixArray the array containing the values for this matrix. Not modified.
    */
   public Matrix3D(double[] matrixArray)
   {
      set(matrixArray);
   }

   /**
    * Creates a new 3D matrix and initializes it from the given 9 coefficients.
    *
    * @param m00 the 1st row 1st column coefficient for this matrix.
    * @param m01 the 1st row 2nd column coefficient for this matrix.
    * @param m02 the 1st row 3rd column coefficient for this matrix.
    * @param m10 the 2nd row 1st column coefficient for this matrix.
    * @param m11 the 2nd row 2nd column coefficient for this matrix.
    * @param m12 the 2nd row 3rd column coefficient for this matrix.
    * @param m20 the 3rd row 1st column coefficient for this matrix.
    * @param m21 the 3rd row 2nd column coefficient for this matrix.
    * @param m22 the 3rd row 3rd column coefficient for this matrix.
    */
   public Matrix3D(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Creates a new 3D matrix that is the same as {@code other}.
    *
    * @param other the other 3D matrix to copy the values from. Not modified.
    */
   public Matrix3D(Matrix3DReadOnly other)
   {
      set(other);
   }

   /**
    * Sets all the coefficients of this matrix to zero.
    */
   @Override
   public void setToZero()
   {
      set(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
   }

   /**
    * Transposes this matrix: m = m<sup>T</sup>.
    */
   public void transpose()
   {
      double temp;

      temp = m01;
      m01 = m10;
      m10 = temp;

      temp = m02;
      m02 = m20;
      m20 = temp;

      temp = m12;
      m12 = m21;
      m21 = temp;
   }

   /** {@inheritDoc} */
   @Override
   public void set(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
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

   /** {@inheritDoc} */
   @Override
   public void set(Matrix3D other)
   {
      set((Matrix3DReadOnly) other);
   }

   /**
    * Converts a vector to tilde form (matrix implementation of cross product).
    *
    * <pre>
    *        /  0 -z  y \
    * this = |  z  0 -x |
    *        \ -y  x  0 /
    * </pre>
    *
    * @param tuple the tuple to use to create its tilde form. Not modified.
    */
   public void setToTildeForm(Tuple3DReadOnly tuple)
   {
      m00 = 0.0;
      m01 = -tuple.getZ();
      m02 = tuple.getY();

      m10 = tuple.getZ();
      m11 = 0.0;
      m12 = -tuple.getX();

      m20 = -tuple.getY();
      m21 = tuple.getX();
      m22 = 0.0;
   }

   /**
    * Sets this matrix to be equal to the outer-product of {@code other}.
    * <p>
    * this = other * other<sup>T<sup>
    * </p>
    * 
    * @param other the other matrix used for this operation. Not modified.
    */
   public void setAndMultiplyOuter(Matrix3DReadOnly other)
   {
      set(other);
      multiplyOuter();
   }

   /**
    * Set this matrix to the inverse of the other matrix.
    * <p>
    * this = other<sup>-1</sup>
    * </p>
    *
    * @param other the other matrix. Not modified.
    * @throws SingularMatrixException if the matrix is not invertible.
    */
   public void setAndInvert(Matrix3DReadOnly other)
   {
      set(other);
      invert();
   }

   /**
    * Sets this matrix to equal the other matrix and then normalizes this, see {@link #normalize()}.
    *
    * @param other the other matrix used to update this matrix. Not modified.
    */
   public void setAndNormalize(Matrix3DReadOnly other)
   {
      set(other);
      normalize();
   }

   /**
    * Sets this matrix to equal the other matrix and then transposes this.
    * <p>
    * this = other<sup>T</sup>
    * </p>
    *
    * @param other the other matrix used to update this matrix. Not modified.
    */
   public void setAndTranspose(Matrix3DReadOnly other)
   {
      set(other);
      transpose();
   }

   /**
    * Sets this matrix to {@code other} and then calls {@link #negate()}.
    *
    * @param other the other matrix used to update this matrix. Not modified.
    */
   public void setAndNegate(Matrix3DReadOnly other)
   {
      set(other);
      negate();
   }

   /**
    * Sets this matrix to represent to represent a counter clockwise rotation around the z-axis of
    * an angle {@code yaw}.
    *
    * <pre>
    *        / cos(yaw) -sin(yaw) 0 \
    * this = | sin(yaw)  cos(yaw) 0 |
    *        \    0         0     1 /
    * </pre>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   public void setToYawMatrix(double yaw)
   {
      double sinYaw = Math.sin(yaw);
      double cosYaw = Math.cos(yaw);
      set(cosYaw, -sinYaw, 0.0, sinYaw, cosYaw, 0.0, 0.0, 0.0, 1.0);
   }

   /**
    * Sets this matrix to represent a counter clockwise rotation around the y-axis of an angle
    * {@code pitch}.
    *
    * <pre>
    *        /  cos(pitch) 0 sin(pitch) \
    * this = |      0      1     0      |
    *        \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   public void setToPitchMatrix(double pitch)
   {
      double sinPitch = Math.sin(pitch);
      double cosPitch = Math.cos(pitch);
      set(cosPitch, 0.0, sinPitch, 0.0, 1.0, 0.0, -sinPitch, 0.0, cosPitch);
   }

   /**
    * Sets this matrix to represent a counter clockwise rotation around the x-axis of an angle
    * {@code roll}.
    *
    * <pre>
    *        / 1     0          0     \
    * this = | 0 cos(roll) -sin(roll) |
    *        \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   public void setToRollMatrix(double roll)
   {
      double sinRoll = Math.sin(roll);
      double cosRoll = Math.cos(roll);
      set(1.0, 0.0, 0.0, 0.0, cosRoll, -sinRoll, 0.0, sinRoll, cosRoll);
   }

   /**
    * Performs a per-component addition onto the coefficients of this matrix.
    * <p>
    * this = this + other
    * </p>
    *
    * @param other the other matrix to use for the addition. Not modified.
    */
   public void add(Matrix3DReadOnly other)
   {
      m00 += other.getM00();
      m01 += other.getM01();
      m02 += other.getM02();

      m10 += other.getM10();
      m11 += other.getM11();
      m12 += other.getM12();

      m20 += other.getM20();
      m21 += other.getM21();
      m22 += other.getM22();
   }

   /**
    * Sets this matrix coefficients to the per-component sum of the two given matrices.
    * <p>
    * this = matrix1 + matrix2
    * </p>
    *
    * @param matrix1 the first term of the addition. Not modified.
    * @param matrix2 the second term of the addition. Not modified.
    */
   public void add(Matrix3DReadOnly matrix1, Matrix3DReadOnly matrix2)
   {
      m00 = matrix1.getM00() + matrix2.getM00();
      m01 = matrix1.getM01() + matrix2.getM01();
      m02 = matrix1.getM02() + matrix2.getM02();

      m10 = matrix1.getM10() + matrix2.getM10();
      m11 = matrix1.getM11() + matrix2.getM11();
      m12 = matrix1.getM12() + matrix2.getM12();

      m20 = matrix1.getM20() + matrix2.getM20();
      m21 = matrix1.getM21() + matrix2.getM21();
      m22 = matrix1.getM22() + matrix2.getM22();
   }

   /**
    * Performs a per-component subtraction onto the coefficients of this matrix.
    * <p>
    * this = this - other
    * </p>
    *
    * @param other the other matrix to use for the subtraction. Not modified.
    */
   public void sub(Matrix3DReadOnly other)
   {
      m00 -= other.getM00();
      m01 -= other.getM01();
      m02 -= other.getM02();

      m10 -= other.getM10();
      m11 -= other.getM11();
      m12 -= other.getM12();

      m20 -= other.getM20();
      m21 -= other.getM21();
      m22 -= other.getM22();
   }

   /**
    * Sets this matrix coefficients to the per-component difference of the two given matrices.
    * <p>
    * this = matrix1 - matrix2
    * </p>
    *
    * @param matrix1 the first term of the addition. Not modified.
    * @param matrix2 the second term of the addition. Not modified.
    */
   public void sub(Matrix3DReadOnly matrix1, Matrix3DReadOnly matrix2)
   {
      m00 = matrix1.getM00() - matrix2.getM00();
      m01 = matrix1.getM01() - matrix2.getM01();
      m02 = matrix1.getM02() - matrix2.getM02();

      m10 = matrix1.getM10() - matrix2.getM10();
      m11 = matrix1.getM11() - matrix2.getM11();
      m12 = matrix1.getM12() - matrix2.getM12();

      m20 = matrix1.getM20() - matrix2.getM20();
      m21 = matrix1.getM21() - matrix2.getM21();
      m22 = matrix1.getM22() - matrix2.getM22();
   }

   /**
    * Sets all the coefficients of this matrix to be equal to {@code scalar}.
    *
    * <pre>
    *        / scalar scalar scalar \
    * this = | scalar scalar scalar |
    *        \ scalar scalar scalar /
    * </pre>
    *
    * @param scalar the scalar value to fill this matrix with.
    */
   public void fill(double scalar)
   {
      m00 = scalar;
      m01 = scalar;
      m02 = scalar;

      m10 = scalar;
      m11 = scalar;
      m12 = scalar;

      m20 = scalar;
      m21 = scalar;
      m22 = scalar;
   }

   /**
    * Performs a per-component scale on this matrix.
    * <p>
    * this = scalar * this
    * </p>
    *
    * @param scalar the scale factor to use on the components of this matrix.
    */
   public void scale(double scalar)
   {
      m00 *= scalar;
      m01 *= scalar;
      m02 *= scalar;

      m10 *= scalar;
      m11 *= scalar;
      m12 *= scalar;

      m20 *= scalar;
      m21 *= scalar;
      m22 *= scalar;
   }

   /**
    * Scales individually each row of this matrix.
    *
    * <pre>
    *        / scalarRow0 * m00 scalarRow0 * m01 scalarRow0 * m02 \
    * this = | scalarRow1 * m10 scalarRow1 * m11 scalarRow1 * m12 |
    *        \ scalarRow2 * m20 scalarRow2 * m21 scalarRow2 * m22 /
    * </pre>
    * <p>
    * This operation is equivalent to pre-multiplying this matrix, i.e. this = D * this, by the
    * following diagonal matrix D:
    *
    * <pre>
    *     / scaleRow0     0         0     \
    * D = |     0     scaleRow1     0     |
    *     \     0         0     scaleRow2 /
    * </pre>
    * </p>
    *
    * @param scalarRow0 the scale factor to use on the components of the 1st row.
    * @param scalarRow1 the scale factor to use on the components of the 2nd row.
    * @param scalarRow2 the scale factor to use on the components of the 3rd row.
    */
   public void scaleRows(double scalarRow0, double scalarRow1, double scalarRow2)
   {
      m00 *= scalarRow0;
      m01 *= scalarRow0;
      m02 *= scalarRow0;

      m10 *= scalarRow1;
      m11 *= scalarRow1;
      m12 *= scalarRow1;

      m20 *= scalarRow2;
      m21 *= scalarRow2;
      m22 *= scalarRow2;
   }

   /**
    * Scales individually each column of this matrix.
    *
    * <pre>
    *        / scalarColumn0 * m00 scalarColumn1 * m01 scalarColumn2 * m02 \
    * this = | scalarColumn0 * m10 scalarColumn1 * m11 scalarColumn2 * m12 |
    *        \ scalarColumn0 * m20 scalarColumn1 * m21 scalarColumn2 * m22 /
    * </pre>
    * <p>
    * This operation is equivalent to multiplying this matrix, i.e. this = this * D, by the
    * following diagonal matrix D:
    *
    * <pre>
    *     / scalarColumn0       0             0       \
    * D = |       0       scalarColumn1       0       |
    *     \       0             0       scalarColumn2 /
    * </pre>
    * </p>
    *
    * @param scalarColumn0 the scale factor to use on the components of the 1st column.
    * @param scalarColumn1 the scale factor to use on the components of the 2nd column.
    * @param scalarColumn2 the scale factor to use on the components of the 3rd column.
    */
   public void scaleColumns(double scalarColumn0, double scalarColumn1, double scalarColumn2)
   {
      m00 *= scalarColumn0;
      m01 *= scalarColumn1;
      m02 *= scalarColumn2;

      m10 *= scalarColumn0;
      m11 *= scalarColumn1;
      m12 *= scalarColumn2;

      m20 *= scalarColumn0;
      m21 *= scalarColumn1;
      m22 *= scalarColumn2;
   }

   /**
    * Sets this matrix to be equal to its outer-product.
    * <p>
    * this = this * this<sup>T<sup>
    * </p>
    */
   public void multiplyOuter()
   {
      Matrix3DTools.multiplyTransposeRight(this, this, this);
   }

   /**
    * {@inheritDoc}
    * <p>
    * this = R * this * R<sup>T</sup><br>
    * where 'R' is the 3-by-3 matrix representing the rotation part of the {@code transform}.
    * </p>
    * <p>
    * Note: the transformation of a {@code Matrix3D} strongly differs from the transformation of a
    * {@link RotationMatrix}.
    * </p>
    */
   @Override
   public void applyTransform(Transform transform)
   {
      transform.transform(this);
   }

   /**
    * {@inheritDoc}
    * <p>
    * this = R<sup>T</sup> * this * R<br>
    * where 'R' is the 3-by-3 matrix representing the rotation part of the {@code transform}.
    * </p>
    * <p>
    * Note: the transformation of a {@code Matrix3D} strongly differs from the transformation of a
    * {@link RotationMatrix}.
    * </p>
    */
   @Override
   public void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(this);
   }

   /**
    * Invert this matrix.
    * <p>
    * this = this<sup>-1</sup>
    * </p>
    *
    * @throws SingularMatrixException if the matrix is not invertible.
    */
   public void invert()
   {
      boolean success = Matrix3DTools.invert(this);
      if (!success)
         throw new SingularMatrixException(this);
   }

   /**
    * Orthonormalization of this matrix using the
    * <a href="https://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process"> Gram-Schmidt method</a>.
    */
   public void normalize()
   {
      Matrix3DTools.normalize(this);
   }

   /**
    * Negates each component of this matrix.
    */
   public void negate()
   {
      m00 = -m00;
      m01 = -m01;
      m02 = -m02;

      m10 = -m10;
      m11 = -m11;
      m12 = -m12;

      m20 = -m20;
      m21 = -m21;
      m22 = -m22;
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this * other
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   public void multiply(Matrix3DReadOnly other)
   {
      Matrix3DTools.multiply(this, other, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this<sup>T</sup> * other
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   public void multiplyTransposeThis(Matrix3DReadOnly other)
   {
      Matrix3DTools.multiplyTransposeLeft(this, other, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this * other<sup>T</sup>
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   public void multiplyTransposeOther(Matrix3DReadOnly other)
   {
      Matrix3DTools.multiplyTransposeRight(this, other, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this<sup>T</sup> * other<sup>T</sup>
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   public void multiplyTransposeBoth(Matrix3DReadOnly other)
   {
      Matrix3DTools.multiplyTransposeBoth(this, other, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this<sup>-1</sup> * other
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    * @throws SingularMatrixException if {@code this} is not invertible.
    */
   public void multiplyInvertThis(Matrix3DReadOnly other)
   {
      Matrix3DTools.multiplyInvertLeft(this, other, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this * other<sup>-1</sup>
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    * @throws SingularMatrixException if {@code other} is not invertible.
    */
   public void multiplyInvertOther(Matrix3DReadOnly other)
   {
      Matrix3DTools.multiplyInvertRight(this, other, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this * other<sup>-1</sup>
    * </p>
    * <p>
    * This operation uses the property: <br>
    * R<sup>-1</sup> = R<sup>T</sup> </br>
    * of the rotation matrix preventing to actually compute its inverse.
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   public void multiplyInvertOther(RotationMatrixReadOnly other)
   {
      Matrix3DTools.multiplyInvertRight(this, other, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this * other<sup>-1</sup>
    * </p>
    * <p>
    * This operation uses the property: <br>
    * (R * S)<sup>-1</sup> = S<sup>-1</sup> * R<sup>T</sup> </br>
    * of the rotation-scale matrix preventing to actually compute its inverse.
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   public void multiplyInvertOther(RotationScaleMatrixReadOnly other)
   {
      Matrix3DTools.multiplyInvertRight(this, other, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other * this
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   public void preMultiply(Matrix3DReadOnly other)
   {
      Matrix3DTools.multiply(other, this, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other * this<sup>T</sup>
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   public void preMultiplyTransposeThis(Matrix3DReadOnly other)
   {
      Matrix3DTools.multiplyTransposeRight(other, this, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other<sup>T</sup> * this
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   public void preMultiplyTransposeOther(Matrix3DReadOnly other)
   {
      Matrix3DTools.multiplyTransposeLeft(other, this, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other<sup>T</sup> * this<sup>T</sup>
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   public void preMultiplyTransposeBoth(Matrix3DReadOnly other)
   {
      Matrix3DTools.multiplyTransposeBoth(other, this, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other * this<sup>-1</sup>
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    * @throws SingularMatrixException if {@code this} is not invertible.
    */
   public void preMultiplyInvertThis(Matrix3DReadOnly other)
   {
      Matrix3DTools.multiplyInvertRight(other, this, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other<sup>-1</sup> * this
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    * @throws SingularMatrixException if {@code other} is not invertible.
    */
   public void preMultiplyInvertOther(Matrix3DReadOnly other)
   {
      Matrix3DTools.multiplyInvertLeft(other, this, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other<sup>-1</sup> * this
    * </p>
    * <p>
    * This operation uses the property: <br>
    * R<sup>-1</sup> = R<sup>T</sup> </br>
    * of the rotation matrix preventing to actually compute its inverse.
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   public void preMultiplyInvertOther(RotationMatrixReadOnly other)
   {
      Matrix3DTools.multiplyInvertLeft(other, this, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other<sup>-1</sup> * this
    * </p>
    * <p>
    * This operation uses the property: <br>
    * (R * S)<sup>-1</sup> = S<sup>-1</sup> * R<sup>T</sup> </br>
    * of the rotation-scale matrix preventing to actually compute its inverse.
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   public void preMultiplyInvertOther(RotationScaleMatrixReadOnly other)
   {
      Matrix3DTools.multiplyInvertLeft(other, this, this);
   }

   /** {@inheritDoc} */
   @Override
   public void transform(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
   {
      Matrix3DTools.transform(this, matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      Matrix3DTools.inverseTransform(this, tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      Matrix3DTools.inverseTransform(this, tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      Matrix3DTools.inverseTransform(this, vectorOriginal, vectorTransformed);
   }

   /** {@inheritDoc} */
   @Override
   public void inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
   {
      Matrix3DTools.inverseTransform(this, matrixOriginal, matrixTransformed);
   }

   /**
    * Sets the {@code row}<sup>th</sup> row components to the values contained in the given array
    * {@code rowArray}.
    *
    * @param row the index of the row to set the values of.
    * @param rowArray the array containing the new values for the row. Not modified.
    * @throws ArrayIndexOutOfBoundsException if {@code row} &notin; [0, 2].
    */
   public void setRow(int row, double rowArray[])
   {
      setRow(row, rowArray[0], rowArray[1], rowArray[2]);
   }

   /**
    * Sets the {@code row}<sup>th</sup> row components to the values contained in the given tuple
    * {@code rowValues}.
    *
    * @param row the index of the row to set the values of.
    * @param rowValues the tuple containing the new values for the row. Not modified.
    * @throws ArrayIndexOutOfBoundsException if {@code row} &notin; [0, 2].
    */
   public void setRow(int row, Tuple3DReadOnly rowValues)
   {
      setRow(row, rowValues.getX(), rowValues.getY(), rowValues.getZ());
   }

   /**
    * Sets the {@code row}<sup>th</sup> row components to the given values.
    *
    * @param row the index of the row to set the values of.
    * @param x the new value of the first component in the row.
    * @param y the new value of the second component in the row.
    * @param z the new value of the third component in the row.
    * @throws ArrayIndexOutOfBoundsException if {@code row} &notin; [0, 2].
    */
   public void setRow(int row, double x, double y, double z)
   {
      switch (row)
      {
      case 0:
         m00 = x;
         m01 = y;
         m02 = z;
         return;

      case 1:
         m10 = x;
         m11 = y;
         m12 = z;
         return;

      case 2:
         m20 = x;
         m21 = y;
         m22 = z;
         return;

      default:
         throw Matrix3DTools.rowOutOfBoundsException(2, row);
      }
   }

   /**
    * Sets the {@code column}<sup>th</sup> column components to the values contained in the given
    * array {@code columnArray}.
    *
    * @param column the index of the column to set the values of.
    * @param columnArray the array containing the new values for the column. Not modified.
    * @throws ArrayIndexOutOfBoundsException if {@code column} &notin; [0, 2].
    */
   public void setColumn(int column, double columnArray[])
   {
      setColumn(column, columnArray[0], columnArray[1], columnArray[2]);
   }

   /**
    * Sets the {@code column}<sup>th</sup> column components to the values contained in the given
    * tuple {@code columnValues}.
    *
    * @param column the index of the column to set the values of.
    * @param columnValues the tuple containing the new values for the column. Not modified.
    * @throws ArrayIndexOutOfBoundsException if {@code column} &notin; [0, 2].
    */
   public void setColumn(int column, Tuple3DReadOnly columnValues)
   {
      setColumn(column, columnValues.getX(), columnValues.getY(), columnValues.getZ());
   }

   /**
    * Sets the {@code column}<sup>th</sup> column components to the given values.
    *
    * @param column the index of the column to set the values of.
    * @param x the new value of the first component in the column.
    * @param y the new value of the second component in the column.
    * @param z the new value of the third component in the column.
    * @throws ArrayIndexOutOfBoundsException if {@code column} &notin; [0, 2].
    */
   public void setColumn(int column, double x, double y, double z)
   {
      switch (column)
      {
      case 0:
         m00 = x;
         m10 = y;
         m20 = z;
         break;

      case 1:
         m01 = x;
         m11 = y;
         m21 = z;
         break;

      case 2:
         m02 = x;
         m12 = y;
         m22 = z;
         break;

      default:
         throw Matrix3DTools.columnOutOfBoundsException(2, column);
      }
   }

   /**
    * Sets this matrix to be a diagonal matrix as follows:
    * 
    * <pre>
    *        / m00  0   0  \
    * this = |  0  m11  0  |
    *        \  0   0  m22 /
    * </pre>
    * 
    * @param m00 the first diagonal element.
    * @param m11 the second diagonal element.
    * @param m22 the third diagonal element.
    */
   public void setToDiagonal(double m00, double m11, double m22)
   {
      set(m00, 0.0, 0.0, 0.0, m11, 0.0, 0.0, 0.0, m22);
   }

   /**
    * Sets this matrix to be a diagonal matrix as follows:
    * 
    * <pre>
    *        / x 0 0 \
    * this = | 0 y 0 |
    *        \ 0 0 z /
    * </pre>
    * 
    * where x, y, and z are the components of the given tuple.
    * 
    * @param tuple the tuple used to set this matrix diagonal elements. Not modified.
    */
   public void setToDiagonal(Tuple3DReadOnly tuple)
   {
      setToDiagonal(tuple.getX(), tuple.getY(), tuple.getZ());
   }

   /**
    * Scales the components of the {@code row}<sup>th</sup> row of this matrix.
    *
    * @param row the index of the row to scale.
    * @param scalar the scale factor to apply.
    * @throws ArrayIndexOutOfBoundsException if {@code row} &notin; [0, 2].
    */
   public void scaleRow(int row, double scalar)
   {
      switch (row)
      {
      case 0:
         m00 *= scalar;
         m01 *= scalar;
         m02 *= scalar;
         return;

      case 1:
         m10 *= scalar;
         m11 *= scalar;
         m12 *= scalar;
         return;

      case 2:
         m20 *= scalar;
         m21 *= scalar;
         m22 *= scalar;
         return;

      default:
         throw Matrix3DTools.rowOutOfBoundsException(2, row);
      }
   }

   /**
    * Scales the components of the {@code column}<sup>th</sup> column of this matrix.
    *
    * @param column the index of the column to scale.
    * @param scalar the scale factor to apply.
    * @throws ArrayIndexOutOfBoundsException if {@code column} &notin; [0, 2].
    */
   public void scaleColumn(int column, double scalar)
   {
      switch (column)
      {
      case 0:
         m00 *= scalar;
         m10 *= scalar;
         m20 *= scalar;
         break;

      case 1:
         m01 *= scalar;
         m11 *= scalar;
         m21 *= scalar;
         break;

      case 2:
         m02 *= scalar;
         m12 *= scalar;
         m22 *= scalar;
         break;

      default:
         throw Matrix3DTools.columnOutOfBoundsException(2, column);
      }
   }

   /**
    * Sets the value of the component of this matrix located by its row and column indices.
    *
    * @param row the index of the component's row.
    * @param column the index of the component's column.
    * @param value the new value of the component.
    * @throws ArrayIndexOutOfBoundsException if {@code row} &notin; [0, 2] or {@code column} &notin;
    *            [0, 2]
    */
   public void setElement(int row, int column, double value)
   {
      switch (row)
      {
      case 0:
         switch (column)
         {
         case 0:
            setM00(value);
            return;
         case 1:
            setM01(value);
            return;
         case 2:
            setM02(value);
            return;
         default:
            throw Matrix3DTools.columnOutOfBoundsException(2, column);
         }

      case 1:
         switch (column)
         {
         case 0:
            setM10(value);
            return;
         case 1:
            setM11(value);
            return;
         case 2:
            setM12(value);
            return;
         default:
            throw Matrix3DTools.columnOutOfBoundsException(2, column);
         }

      case 2:
         switch (column)
         {
         case 0:
            setM20(value);
            return;
         case 1:
            setM21(value);
            return;
         case 2:
            setM22(value);
            return;
         default:
            throw Matrix3DTools.columnOutOfBoundsException(2, column);
         }

      default:
         throw Matrix3DTools.rowOutOfBoundsException(2, row);
      }
   }

   /**
    * Sets the value of the 1st row 1st column component.
    *
    * @param m00 the new value of the component.
    */
   public void setM00(double m00)
   {
      this.m00 = m00;
   }

   /**
    * Sets the value of the 1st row 2nd column component.
    *
    * @param m01 the new value of the component.
    */
   public void setM01(double m01)
   {
      this.m01 = m01;
   }

   /**
    * Sets the value of the 1st row 3rd column component.
    *
    * @param m02 the new value of the component.
    */
   public void setM02(double m02)
   {
      this.m02 = m02;
   }

   /**
    * Sets the value of the 2nd row 1st column component.
    *
    * @param m10 the new value of the component.
    */
   public void setM10(double m10)
   {
      this.m10 = m10;
   }

   /**
    * Sets the value of the 2nd row 2nd column component.
    *
    * @param m11 the new value of the component.
    */
   public void setM11(double m11)
   {
      this.m11 = m11;
   }

   /**
    * Sets the value of the 2nd row 3rd column component.
    *
    * @param m12 the new value of the component.
    */
   public void setM12(double m12)
   {
      this.m12 = m12;
   }

   /**
    * Sets the value of the 3rd row 1st column component.
    *
    * @param m20 the new value of the component.
    */
   public void setM20(double m20)
   {
      this.m20 = m20;
   }

   /**
    * Sets the value of the 3rd row 2nd column component.
    *
    * @param m21 the new value of the component.
    */
   public void setM21(double m21)
   {
      this.m21 = m21;
   }

   /**
    * Sets the value of the 3rd row 3rd column component.
    *
    * @param m22 the new value of the component.
    */
   public void setM22(double m22)
   {
      this.m22 = m22;
   }

   /**
    * Scales the value of the 1st row 1st column component.
    *
    * @param scalar the scale factor to apply.
    */
   public void scaleM00(double scalar)
   {
      m00 *= scalar;
   }

   /**
    * Scales the value of the 1st row 2nd column component.
    *
    * @param scalar the scale factor to apply.
    */
   public void scaleM01(double scalar)
   {
      m01 *= scalar;
   }

   /**
    * Scales the value of the 1st row 3rd column component.
    *
    * @param scalar the scale factor to apply.
    */
   public void scaleM02(double scalar)
   {
      m02 *= scalar;
   }

   /**
    * Scales the value of the 2nd row 1st column component.
    *
    * @param scalar the scale factor to apply.
    */
   public void scaleM10(double scalar)
   {
      m10 *= scalar;
   }

   /**
    * Scales the value of the 2nd row 2nd column component.
    *
    * @param scalar the scale factor to apply.
    */
   public void scaleM11(double scalar)
   {
      m11 *= scalar;
   }

   /**
    * Scales the value of the 2nd row 3rd column component.
    *
    * @param scalar the scale factor to apply.
    */
   public void scaleM12(double scalar)
   {
      m12 *= scalar;
   }

   /**
    * Scales the value of the 3rd row 1st column component.
    *
    * @param scalar the scale factor to apply.
    */
   public void scaleM20(double scalar)
   {
      m20 *= scalar;
   }

   /**
    * Scales the value of the 3rd row 2nd column component.
    *
    * @param scalar the scale factor to apply.
    */
   public void scaleM21(double scalar)
   {
      m21 *= scalar;
   }

   /**
    * Scales the value of the 3rd row 3rd column component.
    *
    * @param scalar the scale factor to apply.
    */
   public void scaleM22(double scalar)
   {
      m22 *= scalar;
   }

   /** {@inheritDoc} */
   @Override
   public double getM00()
   {
      return m00;
   }

   /** {@inheritDoc} */
   @Override
   public double getM01()
   {
      return m01;
   }

   /** {@inheritDoc} */
   @Override
   public double getM02()
   {
      return m02;
   }

   /** {@inheritDoc} */
   @Override
   public double getM10()
   {
      return m10;
   }

   /** {@inheritDoc} */
   @Override
   public double getM11()
   {
      return m11;
   }

   /** {@inheritDoc} */
   @Override
   public double getM12()
   {
      return m12;
   }

   /** {@inheritDoc} */
   @Override
   public double getM20()
   {
      return m20;
   }

   /** {@inheritDoc} */
   @Override
   public double getM21()
   {
      return m21;
   }

   /** {@inheritDoc} */
   @Override
   public double getM22()
   {
      return m22;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method
    * returns {@link #equals(Matrix3DReadOnly)}, it returns {@code false} otherwise or if the
    * {@code object} is {@code null}.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((Matrix3DReadOnly) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Tests on a per coefficient basis if this matrix is equal to the given {@code other} to an
    * {@code epsilon}.
    *
    * @param other the other matrix to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two matrices are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(Matrix3D other, double epsilon)
   {
      return CommonMatrix3DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Two 3D matrices are considered geometrically equal if they are epsilon equal.
    * <p>
    * This method is equivalent to {@link #epsilonEquals(Matrix3D, double)}.
    * </p>
    *
    * @param other the other matrix to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two matrices are equal, {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(Matrix3D other, double epsilon)
   {
      return epsilonEquals(other, epsilon);
   }

   /**
    * Provides a {@code String} representation of this matrix as follows: <br>
    * m00, m01, m02 <br>
    * m10, m11, m12 <br>
    * m20, m21, m22
    *
    * @return the {@code String} representing this matrix.
    */
   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getMatrixString(this);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this matrix.
    *
    * @return the hash code value for this matrix.
    */
   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = EuclidHashCodeTools.addToHashCode(bits, m00);
      bits = EuclidHashCodeTools.addToHashCode(bits, m01);
      bits = EuclidHashCodeTools.addToHashCode(bits, m02);
      bits = EuclidHashCodeTools.addToHashCode(bits, m10);
      bits = EuclidHashCodeTools.addToHashCode(bits, m11);
      bits = EuclidHashCodeTools.addToHashCode(bits, m12);
      bits = EuclidHashCodeTools.addToHashCode(bits, m20);
      bits = EuclidHashCodeTools.addToHashCode(bits, m21);
      bits = EuclidHashCodeTools.addToHashCode(bits, m22);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }
}
