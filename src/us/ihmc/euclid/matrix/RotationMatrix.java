package us.ihmc.euclid.matrix;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.rotationConversion.RotationMatrixConversion;
import us.ihmc.euclid.tools.*;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

/**
 * A {@code RotationMatrix} is a 3-by-3 matrix used to represent 3d orientations.
 * <p>
 * A rotation matrix has to comply to several constraints:
 * <ul>
 * <li>each column of the matrix represents a unitary vector,
 * <li>each row of the matrix represents a unitary vector,
 * <li>every pair of columns of the matrix represents two orthogonal vectors,
 * <li>every pair of rows of the matrix represents two orthogonal vectors,
 * <li>the matrix determinant is equal to {@code 1}.
 * </ul>
 * A rotation matrix has the nice property <i>R<sup>T</sup> = R<sup>-1</sup></i>.
 * </p>
 * <p>
 * A best effort has been put in the interface of {@code RotationMatrix} to maximize the use of the
 * inherent properties of a rotation matrix and to minimize manipulation errors resulting in an
 * improper rotation matrix.
 * </p>
 *
 * @author Sylvain Bertrand
 *
 */
public class RotationMatrix implements Matrix3DBasics, RotationMatrixReadOnly, GeometryObject<RotationMatrix>
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
    * Create a new rotation matrix initialized to identity.
    */
   public RotationMatrix()
   {
      setIdentity();
   }

   /**
    * Creates a new rotation matrix and initializes it from the given 9 coefficients.
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
    * @throws NotARotationMatrixException if the resulting matrix is not a rotation matrix.
    */
   public RotationMatrix(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Creates a new rotation matrix and initializes it from the given array.
    *
    * <pre>
    *        / rotationMatrixArray[0]  rotationMatrixArray[1]  rotationMatrixArray[2] \
    * this = | rotationMatrixArray[3]  rotationMatrixArray[4]  rotationMatrixArray[5] |
    *        \ rotationMatrixArray[6]  rotationMatrixArray[7]  rotationMatrixArray[8] /
    * </pre>
    *
    * @param rotationMatrixArray the array containing the values for this matrix. Not modified.
    * @throws NotARotationMatrixException if the resulting matrix is not a rotation matrix.
    */
   public RotationMatrix(double[] rotationMatrixArray)
   {
      set(rotationMatrixArray);
   }

   /**
    * Creates a new rotation matrix that is the same as {@code rotationMatrix}.
    *
    * @param rotationMatrix the other 3D matrix to copy the values from. Not modified.
    * @throws NotARotationMatrixException if the resulting matrix is not a rotation matrix.
    */
   public RotationMatrix(DenseMatrix64F rotationMatrix)
   {
      set(rotationMatrix);
   }

   /**
    * Creates a new rotation matrix that is the same as {@code rotationMatrix}.
    *
    * @param rotationMatrix the other 3D matrix to copy the values from. Not modified.
    * @throws NotARotationMatrixException if the resulting matrix is not a rotation matrix.
    */
   public RotationMatrix(Matrix3DReadOnly rotationMatrix)
   {
      set(rotationMatrix);
   }

   /**
    * Creates a new rotation matrix that is the same as {@code other}.
    *
    * @param other the other 3D matrix to copy the values from. Not modified.
    */
   public RotationMatrix(RotationMatrixReadOnly other)
   {
      set(other);
   }

   /**
    * Creates a new rotation matrix representing the same orientation as the given
    * {@code axisAngle}.
    *
    * @param axisAngle the axis-angle used to initialize this rotation matrix. Not modified.
    */
   public RotationMatrix(AxisAngleReadOnly axisAngle)
   {
      set(axisAngle);
   }

   /**
    * Creates a new rotation matrix representing the same orientation as the given
    * {@code quaternion}.
    *
    * @param quaternion the quaternion used to initialize this rotation matrix. Not modified.
    */
   public RotationMatrix(QuaternionReadOnly quaternion)
   {
      set(quaternion);
   }

   /**
    * Creates a new rotation matrix representing the same orientation as the given rotation vector
    * {@code rotationVector}.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param rotationVector the rotation vector used to initialize this rotation matrix. Not
    *           modified.
    */
   public RotationMatrix(Vector3DReadOnly rotationVector)
   {
      set(rotationVector);
   }

   /**
    * Sets this rotation matrix to identity representing a 'zero' rotation.
    */
   @Override
   public void setToZero()
   {
      setIdentity();
   }

   /**
    * Orthonormalization of the rotation matrix using the
    * <a href="https://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process"> Gram-Schmidt method</a>.
    *
    * @throws NotARotationMatrixException if the orthonormalization failed.
    */
   public void normalize()
   {
      Matrix3DTools.normalize(this);
   }

   /**
    * Transposes this matrix: m = m<sup>T</sup>.
    *
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

   /**
    * Sets the 9 coefficients of this rotation matrix without performing any checks on the data
    * provided.
    * <p>
    * This method is meant for internal usage. Prefer using
    * {@link #set(double, double, double, double, double, double, double, double, double)} or
    * {@link #setAndNormalize(double, double, double, double, double, double, double, double, double)}.
    * </p>
    *
    * @param m00 the new 1st row 1st column coefficient for this matrix.
    * @param m01 the new 1st row 2nd column coefficient for this matrix.
    * @param m02 the new 1st row 3rd column coefficient for this matrix.
    * @param m10 the new 2nd row 1st column coefficient for this matrix.
    * @param m11 the new 2nd row 2nd column coefficient for this matrix.
    * @param m12 the new 2nd row 3rd column coefficient for this matrix.
    * @param m20 the new 3rd row 1st column coefficient for this matrix.
    * @param m21 the new 3rd row 2nd column coefficient for this matrix.
    * @param m22 the new 3rd row 3rd column coefficient for this matrix.
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

   /**
    * {@inheritDoc}
    *
    * @throws NotARotationMatrixException if the resulting matrix is not a rotation matrix.
    */
   @Override
   public void set(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      checkIfRotationMatrix();
   }

   /**
    * Sets this rotation matrix from the given tuples each holding on the values of each row.
    *
    * <pre>
    *        /  firstRow.getX()  firstRow.getY()  firstRow.getZ() \
    * this = | secondRow.getX() secondRow.getY() secondRow.getZ() |
    *        \  thirdRow.getX()  thirdRow.getY()  thirdRow.getZ() /
    * </pre>
    *
    * @param firstRow the tuple holding onto the values of the first row. Not modified.
    * @param secondRow the tuple holding onto the values of the second row. Not modified.
    * @param thirdRow the tuple holding onto the values of the third row. Not modified.
    * @throws NotARotationMatrixException if the resulting matrix is not a rotation matrix.
    */
   public void setRows(Tuple3DReadOnly firstRow, Tuple3DReadOnly secondRow, Tuple3DReadOnly thirdRow)
   {
      m00 = firstRow.getX();
      m01 = firstRow.getY();
      m02 = firstRow.getZ();

      m10 = secondRow.getX();
      m11 = secondRow.getY();
      m12 = secondRow.getZ();

      m20 = thirdRow.getX();
      m21 = thirdRow.getY();
      m22 = thirdRow.getZ();

      checkIfRotationMatrix();
   }

   /**
    * Sets this rotation matrix from the given tuples each holding on the values of each column.
    *
    * <pre>
    *        / firstColumn.getX() secondColumn.getX() thirdColumn.getX() \
    * this = | firstColumn.getY() secondColumn.getY() thirdColumn.getY() |
    *        \ firstColumn.getZ() secondColumn.getZ() thirdColumn.getZ() /
    * </pre>
    *
    * @param firstColumn the tuple holding onto the values of the first column. Not modified.
    * @param secondColumn the tuple holding onto the values of the second column. Not modified.
    * @param thirdColumn the tuple holding onto the values of the third column. Not modified.
    * @throws NotARotationMatrixException if the resulting matrix is not a rotation matrix.
    */
   public void setColumns(Tuple3DReadOnly firstColumn, Tuple3DReadOnly secondColumn, Tuple3DReadOnly thirdColumn)
   {
      m00 = firstColumn.getX();
      m10 = firstColumn.getY();
      m20 = firstColumn.getZ();

      m01 = secondColumn.getX();
      m11 = secondColumn.getY();
      m21 = secondColumn.getZ();

      m02 = thirdColumn.getX();
      m12 = thirdColumn.getY();
      m22 = thirdColumn.getZ();

      checkIfRotationMatrix();
   }

   /**
    * Sets the 9 coefficients of this rotation matrix and then normalizes {@code this}.
    *
    * @param m00 the new 1st row 1st column coefficient for this matrix.
    * @param m01 the new 1st row 2nd column coefficient for this matrix.
    * @param m02 the new 1st row 3rd column coefficient for this matrix.
    * @param m10 the new 2nd row 1st column coefficient for this matrix.
    * @param m11 the new 2nd row 2nd column coefficient for this matrix.
    * @param m12 the new 2nd row 3rd column coefficient for this matrix.
    * @param m20 the new 3rd row 1st column coefficient for this matrix.
    * @param m21 the new 3rd row 2nd column coefficient for this matrix.
    * @param m22 the new 3rd row 3rd column coefficient for this matrix.
    * @throws NotARotationMatrixException if the normalization failed.
    */
   public void setAndNormalize(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      normalize();
   }

   /**
    * Sets this rotation matrix to equal the given one {@code other}.
    *
    * @param other the other rotation matrix to copy the values from. Not modified.
    */
   @Override
   public void set(RotationMatrix other)
   {
      set((RotationMatrixReadOnly) other);
   }

   /**
    * Sets this rotation matrix to equal the given one {@code other}.
    *
    * @param other the other rotation matrix to copy the values from. Not modified.
    */
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

   /**
    * Sets this rotation matrix to equal the 3D matrix {@code matrix} and then normalizes
    * {@code this}.
    *
    * @param matrix the matrix to copy the values from. Not modified.
    * @throws NotARotationMatrixException if the normalization failed.
    */
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

   /**
    * Sets this rotation matrix to equal the other given one {@code other} and then normalizes
    * {@code this}.
    *
    * @param other the matrix to copy the values from. Not modified.
    * @throws NotARotationMatrixException if the normalization failed.
    */
   public void setAndNormalize(RotationMatrixReadOnly other)
   {
      set(other);
      normalize();
   }

   /**
    * Sets this rotation matrix to the invert of the given {@code matrix}.
    * <p>
    * This operation uses the property: <br>
    * R<sup>-1</sup> = R<sup>T</sup> </br>
    * of a rotation matrix preventing to actually compute the inverse of the matrix.
    * </p>
    *
    * @param matrix the matrix to copy the values from. Not modified.
    * @throws NotARotationMatrixException if {@code matrix} is not a rotation matrix.
    */
   public void setAndInvert(Matrix3DReadOnly matrix)
   {
      setAndTranspose(matrix);
   }

   /**
    * Sets this rotation matrix to the invert of the given one {@code other}.
    * <p>
    * This operation uses the property: <br>
    * R<sup>-1</sup> = R<sup>T</sup> </br>
    * of a rotation matrix preventing to actually compute the inverse of the matrix.
    * </p>
    *
    * @param other the matrix to copy the values from. Not modified.
    */
   public void setAndInvert(RotationMatrixReadOnly other)
   {
      setAndTranspose(other);
   }

   /**
    * Sets this rotation matrix to the transpose of the given {@code matrix}.
    *
    * @param matrix the matrix to copy the values from. Not modified.
    * @throws NotARotationMatrixException if {@code matrix} is not a rotation matrix.
    */
   public void setAndTranspose(Matrix3DReadOnly matrix)
   {
      set(matrix);
      transpose();
   }

   /**
    * Sets this rotation matrix to the transpose of the given {@code other}.
    *
    * @param other the matrix to copy the values from. Not modified.
    */
   public void setAndTranspose(RotationMatrixReadOnly other)
   {
      set(other);
      transpose();
   }

   /**
    * Sets this rotation matrix to the same orientation described by the given {@code axisAngle}.
    *
    * @param axisAngle the axis-angle used to set this matrix. Not modified.
    */
   public void set(AxisAngleReadOnly axisAngle)
   {
      RotationMatrixConversion.convertAxisAngleToMatrix(axisAngle, this);
   }

   /**
    * Sets this rotation matrix to the same orientation described by the given {@code quaternion}.
    *
    * @param quaternion the quaternion used to set this matrix. Not modified.
    */
   public void set(QuaternionReadOnly quaternion)
   {
      RotationMatrixConversion.convertQuaternionToMatrix(quaternion, this);
   }

   /**
    * Sets this rotation matrix to the same orientation described by the given rotation vector
    * {@code rotationVector}.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param rotation vector the rotation vector used to set this matrix. Not modified.
    */
   public void set(Vector3DReadOnly rotationVector)
   {
      RotationMatrixConversion.convertRotationVectorToMatrix(rotationVector, this);
   }

   /**
    * Sets this rotation matrix to represent a counter clockwise rotation around the z-axis of an
    * angle {@code yaw}.
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
      RotationMatrixConversion.computeYawMatrix(yaw, this);
   }

   /**
    * Sets this rotation matrix to represent a counter clockwise rotation around the y-axis of an
    * angle {@code pitch}.
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
      RotationMatrixConversion.computePitchMatrix(pitch, this);
   }

   /**
    * Sets this rotation matrix to represent a counter clockwise rotation around the x-axis of an
    * angle {@code roll}.
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
      RotationMatrixConversion.computeRollMatrix(roll, this);
   }

   /**
    * Sets this rotation matrix to represent the same orientation as the given yaw-pitch-roll
    * {@code yawPitchRoll}.
    *
    * <pre>
    *        / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
    * this = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
    *        \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param yawPitchRoll the yaw-pitch-roll Euler angles to copy the orientation from. Not
    *           modified.
    */
   public void setYawPitchRoll(double[] yawPitchRoll)
   {
      setYawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
   }

   /**
    * Sets this rotation matrix to represent the same orientation as the given yaw-pitch-roll
    * {@code yaw}, {@code pitch}, and {@code roll}.
    *
    * <pre>
    *        / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
    * this = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
    *        \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param yaw the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll the angle to rotate about the x-axis.
    */
   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      RotationMatrixConversion.convertYawPitchRollToMatrix(yaw, pitch, roll, this);
   }

   /**
    * Sets this rotation matrix to represent the same orientation as the given Euler angles
    * {@code eulerAngles}.
    *
    * <pre>
    *        / cos(eulerAngles.z) -sin(eulerAngles.z) 0 \   /  cos(eulerAngles.y) 0 sin(eulerAngles.y) \   / 1         0                   0          \
    * this = | sin(eulerAngles.z)  cos(eulerAngles.z) 0 | * |          0          1         0          | * | 0 cos(eulerAngles.x) -sin(eulerAngles.x) |
    *        \         0                   0          1 /   \ -sin(eulerAngles.y) 0 cos(eulerAngles.y) /   \ 0 sin(eulerAngles.x)  cos(eulerAngles.x) /
    * </pre>
    * <p>
    * This is equivalent to
    * {@code this.setYawPitchRoll(eulerAngles.getZ(), eulerAngles.getY(), eulerAngles.getX())}.
    * </p>
    *
    * @param eulerAngles the Euler angles to copy the orientation from. Not modified.
    */
   public void setEuler(Vector3DReadOnly eulerAngles)
   {
      setYawPitchRoll(eulerAngles.getZ(), eulerAngles.getY(), eulerAngles.getX());
   }

   /**
    * Sets this rotation matrix to represent the same orientation as the given Euler angles
    * {@code rotX}, {@code rotY}, and {@code rotZ}.
    *
    * <pre>
    *        / cos(rotZ) -sin(rotZ) 0 \   /  cos(rotY) 0 sin(rotY) \   / 1     0          0     \
    * this = | sin(rotZ)  cos(rotZ) 0 | * |      0     1     0     | * | 0 cos(rotX) -sin(rotX) |
    *        \     0          0     1 /   \ -sin(rotY) 0 cos(rotY) /   \ 0 sin(rotX)  cos(rotX) /
    * </pre>
    * <p>
    * This is equivalent to {@code this.setYawPitchRoll(rotZ, rotY, rotX)}.
    * </p>
    *
    * @param rotX the angle to rotate about the x-axis.
    * @param rotY the angle to rotate about the y-axis.
    * @param rotZ the angle to rotate about the z-axis.
    */
   public void setEuler(double rotX, double rotY, double rotZ)
   {
      setYawPitchRoll(rotZ, rotY, rotX);
   }

   /**
    * Inverts this rotation matrix.
    * <p>
    * This operation uses the property: <br>
    * R<sup>-1</sup> = R<sup>T</sup> </br>
    * of a rotation matrix preventing to actually compute the inverse of the matrix.
    * </p>
    * <p>
    * This is equivalent to {@code this.transpose()}.
    * </p>
    */
   public void invert()
   {
      transpose();
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this * other
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   public void multiply(RotationMatrixReadOnly other)
   {
      RotationMatrixTools.multiply(this, other, this);
   }

   /**
    * Performs a multiplication on this.
    * <p>
    * this = this * R(quaternion) <br>
    * where R(quaternion) is the function to convert a quaternion into a rotation matrix.
    * </p>
    *
    * @param quaternion the quaternion to multiply this. Not modified.
    */
   public void multiply(QuaternionReadOnly quaternion)
   {
      QuaternionTools.multiply(this, quaternion, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this<sup>T</sup> * other
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   public void multiplyTransposeThis(RotationMatrixReadOnly other)
   {
      RotationMatrixTools.multiplyTransposeLeft(this, other, this);
   }

   /**
    * Performs a multiplication on this.
    * <p>
    * this = this<sup>T</sup> * R(quaternion) <br>
    * where R(quaternion) is the function to convert a quaternion into a rotation matrix.
    * </p>
    *
    * @param quaternion the quaternion to multiply this. Not modified.
    */
   public void multiplyTransposeThis(QuaternionReadOnly quaternion)
   {
      QuaternionTools.multiplyTransposeMatrix(this, quaternion, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this * other<sup>T</sup>
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   public void multiplyTransposeOther(RotationMatrixReadOnly other)
   {
      RotationMatrixTools.multiplyTransposeRight(this, other, this);
   }

   /**
    * Performs a multiplication on this.
    * <p>
    * this = this * R(quaternion*) <br>
    * where R(quaternion) is the function to convert a quaternion into a rotation matrix.
    * </p>
    *
    * @param quaternion the quaternion to multiply this. Not modified.
    */
   public void multiplyConjugateQuaternion(QuaternionReadOnly quaternion)
   {
      QuaternionTools.multiplyConjugateQuaternion(this, quaternion, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this<sup>T</sup> * other<sup>T</sup>
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   public void multiplyTransposeBoth(RotationMatrixReadOnly other)
   {
      RotationMatrixTools.multiplyTransposeBoth(this, other, this);
   }

   /**
    * Performs a multiplication on this.
    * <p>
    * this = this<sup>T</sup> * R(quaternion*) <br>
    * where R(quaternion) is the function to convert a quaternion into a rotation matrix.
    * </p>
    *
    * @param quaternion the quaternion to multiply this. Not modified.
    */
   public void multiplyTransposeThisConjugateQuaternion(QuaternionReadOnly quaternion)
   {
      QuaternionTools.multiplyTransposeMatrixConjugateQuaternion(this, quaternion, this);
   }

   /**
    * Append a rotation about the z-axis to this rotation matrix.
    *
    * <pre>
    *               / cos(yaw) -sin(yaw) 0 \
    * this = this * | sin(yaw)  cos(yaw) 0 |
    *               \    0         0     1 /
    * </pre>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   public void appendYawRotation(double yaw)
   {
      RotationMatrixTools.appendYawRotation(this, yaw, this);
   }

   /**
    * Append a rotation about the y-axis to this rotation matrix.
    *
    * <pre>
    *               /  cos(pitch) 0 sin(pitch) \
    * this = this * |      0      1     0      |
    *               \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   public void appendPitchRotation(double pitch)
   {
      RotationMatrixTools.appendPitchRotation(this, pitch, this);
   }

   /**
    * Append a rotation about the x-axis to this rotation matrix.
    *
    * <pre>
    *               /  cos(pitch) 0 sin(pitch) \
    * this = this * |      0      1     0      |
    *               \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   public void appendRollRotation(double roll)
   {
      RotationMatrixTools.appendRollRotation(this, roll, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other * this
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   public void preMultiply(RotationMatrixReadOnly other)
   {
      RotationMatrixTools.multiply(other, this, this);
   }

   /**
    * Performs a multiplication on this.
    * <p>
    * this = R(quaternion) * this <br>
    * where R(quaternion) is the function to convert a quaternion into a rotation matrix.
    * </p>
    *
    * @param quaternion the quaternion to multiply this. Not modified.
    */
   public void preMultiply(QuaternionReadOnly quaternion)
   {
      QuaternionTools.multiply(quaternion, this, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other * this<sup>T</sup>
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   public void preMultiplyTransposeThis(RotationMatrixReadOnly other)
   {
      RotationMatrixTools.multiplyTransposeRight(other, this, this);
   }

   /**
    * Performs a multiplication on this.
    * <p>
    * this = R(quaternion) * this<sup>T</sup> <br>
    * where R(quaternion) is the function to convert a quaternion into a rotation matrix.
    * </p>
    *
    * @param quaternion the quaternion to multiply this. Not modified.
    */
   public void preMultiplyTransposeThis(QuaternionReadOnly quaternion)
   {
      QuaternionTools.multiplyTransposeMatrix(quaternion, this, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other<sup>T</sup> * this
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   public void preMultiplyTransposeOther(RotationMatrixReadOnly other)
   {
      RotationMatrixTools.multiplyTransposeLeft(other, this, this);
   }

   /**
    * Performs a multiplication on this.
    * <p>
    * this = R(quaternion*) * this <br>
    * where R(quaternion) is the function to convert a quaternion into a rotation matrix.
    * </p>
    *
    * @param quaternion the quaternion to multiply this. Not modified.
    */
   public void preMultiplyConjugateQuaternion(QuaternionReadOnly quaternion)
   {
      QuaternionTools.multiplyConjugateQuaternion(quaternion, this, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other<sup>T</sup> * this<sup>T</sup>
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   public void preMultiplyTransposeBoth(RotationMatrixReadOnly other)
   {
      RotationMatrixTools.multiplyTransposeBoth(other, this, this);
   }

   /**
    * Performs a multiplication on this.
    * <p>
    * this = R(quaternion*) * this<sup>T</sup> <br>
    * where R(quaternion) is the function to convert a quaternion into a rotation matrix.
    * </p>
    *
    * @param quaternion the quaternion to multiply this. Not modified.
    */
   public void preMultiplyTransposeThisConjugateQuaternion(QuaternionReadOnly quaternion)
   {
      QuaternionTools.multiplyConjugateQuaternionTransposeMatrix(quaternion, this, this);
   }

   /**
    * Prepend a rotation about the z-axis to this rotation matrix.
    *
    * <pre>
    *        / cos(yaw) -sin(yaw) 0 \
    * this = | sin(yaw)  cos(yaw) 0 | * this
    *        \    0         0     1 /
    * </pre>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   public void prependYawRotation(double yaw)
   {
      RotationMatrixTools.prependYawRotation(yaw, this, this);
   }

   /**
    * Prepend a rotation about the y-axis to this rotation matrix.
    *
    * <pre>
    *        /  cos(pitch) 0 sin(pitch) \
    * this = |      0      1     0      | * this
    *        \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   public void prependPitchRotation(double pitch)
   {
      RotationMatrixTools.prependPitchRotation(pitch, this, this);
   }

   /**
    * Append a rotation about the x-axis to this rotation matrix.
    *
    * <pre>
    *        /  cos(pitch) 0 sin(pitch) \
    * this = |      0      1     0      | * this
    *        \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   public void prependRollRotation(double roll)
   {
      RotationMatrixTools.prependRollRotation(roll, this, this);
   }

   /**
    * Performs a linear interpolation in SO(3) from {@code this} to {@code rf} given the percentage
    * {@code alpha}.
    * <p>
    * This is equivalent to but much more computationally expensive than the <i>Spherical Linear
    * Interpolation</i> performed with quaternions.
    * </p>
    *
    * @param rf the other rotation matrix used for the interpolation. Not modified.
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not
    *           modifying this rotation matrix, while a value of 1 is equivalent to setting this
    *           rotation matrix to {@code rf}.
    */
   public void interpolate(RotationMatrixReadOnly rf, double alpha)
   {
      interpolate(this, rf, alpha);
   }

   /**
    * Performs a linear interpolation in SO(3) from {@code r0} to {@code rf} given the percentage
    * {@code alpha}.
    * <p>
    * This is equivalent to but much more computationally expensive than the <i>Spherical Linear
    * Interpolation</i> performed with quaternions.
    * </p>
    *
    * @param r0 the first rotation matrix used in the interpolation. Not modified.
    * @param rf the second rotation matrix used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           this rotation matrix to {@code r0}, while a value of 1 is equivalent to setting this
    *           rotation matrix to {@code rf}.
    */
   public void interpolate(RotationMatrixReadOnly r0, RotationMatrixReadOnly rf, double alpha)
   {
      RotationMatrixTools.interpolate(r0, rf, alpha, this);
   }

   /**
    * {@inheritDoc}
    * <p>
    * this = R * this where 'R' is the 3-by-3 matrix representing the rotation part of the
    * {@code transform}.
    * </p>
    * <p>
    * Note: the transformation of a {@code RotationMatrix} strongly differs from the transformation
    * of a {@link Matrix3D}.
    * </p>
    */
   @Override
   public void applyTransform(Transform transform)
   {
      transform.transform(this);
      normalize();
   }

   /**
    * {@inheritDoc}
    * <p>
    * this = R<sup>T</sup> * this where 'R' is the 3-by-3 matrix representing the rotation part of
    * the {@code transform}.
    * </p>
    * <p>
    * Note: the transformation of a {@code RotationMatrix} strongly differs from the transformation
    * of a {@link Matrix3D}.
    * </p>
    */
   @Override
   public void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(this);
      normalize();
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
   public boolean epsilonEquals(RotationMatrix other, double epsilon)
   {
      return RotationMatrixReadOnly.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests if {@code this} and {@code other} represent the same orientation to an {@code epsilon}.
    * <p>
    * Two rotation matrices are considered geometrically equal if the magnitude of their difference
    * is less than or equal to {@code epsilon}.
    * </p>
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * {@code this.epsilonEquals(other, epsilon)} and vice versa.
    * </p>
    *
    * @param other the other rotation matrix to compare against this. Not modified.
    * @param epsilon the maximum angle between the two rotation matrices to be considered equal.
    * @return {@code true} if the two rotation matrices represent the same geometry, {@code false}
    *         otherwise.
    */
   @Override
   public boolean geometricallyEquals(RotationMatrix other, double epsilon)
   {
      return RotationMatrixReadOnly.super.geometricallyEquals(other, epsilon);
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
