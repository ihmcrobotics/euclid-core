package us.ihmc.geometry.matrix;

import java.io.Serializable;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.geometry.exceptions.NotARotationMatrixException;
import us.ihmc.geometry.interfaces.GeometryObject;
import us.ihmc.geometry.matrix.interfaces.Matrix3DBasics;
import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.geometry.transform.interfaces.Transform;
import us.ihmc.geometry.tuple.RotationVectorConversion;
import us.ihmc.geometry.tuple.interfaces.Tuple3DBasics;
import us.ihmc.geometry.tuple.interfaces.VectorBasics;
import us.ihmc.geometry.tuple.interfaces.VectorReadOnly;
import us.ihmc.geometry.tuple4D.QuaternionConversion;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.geometry.yawPitchRoll.YawPitchRollConversion;

/**
 * A {@code RotationMatrix} is a 3-by-3 matrix used to represent 3d orientations.
 * <p>
 * A rotation matrix has to comply to several constraints:
 * <ul>
 *    <li> each column of the matrix represents a unitary vector,
 *    <li> each row of the matrix represents a unitary vector,
 *    <li> every pair of columns of the matrix represents two orthogonal vectors,
 *    <li> every pair of rows of the matrix represents two orthogonal vectors,
 *    <li> the matrix determinant is equal to {@code 1}.
 * </ul>
 * A rotation matrix has the nice property <i>R<sup>T</sup> = R<sup>-1</sup></i>.
 * </p>
 * <p>
 * A best effort has been put in the interface of {@code RotationMatrix} to maximize
 * the use of the inherent properties of a rotation matrix and to minimize
 * manipulation errors resulting in an improper rotation matrix.
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 */
public class RotationMatrix implements Serializable, Matrix3DBasics<RotationMatrix>, RotationMatrixReadOnly<RotationMatrix>, GeometryObject<RotationMatrix>
{
   private static final long serialVersionUID = 2802307840830134164L;

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
   public RotationMatrix(Matrix3DReadOnly<?> rotationMatrix)
   {
      set(rotationMatrix);
   }

   /**
    * Creates a new rotation matrix that is the same as {@code other}.
    * 
    * @param other the other 3D matrix to copy the values from. Not modified.
    */
   public RotationMatrix(RotationMatrixReadOnly<?> other)
   {
      set(other);
   }

   /**
    * Creates a new rotation matrix representing the same orientation as the given {@code axisAngle}.
    * 
    * @param axisAngle the axis-angle used to initialize this rotation matrix. Not modified.
    */
   public RotationMatrix(AxisAngleReadOnly<?> axisAngle)
   {
      set(axisAngle);
   }

   /**
    * Creates a new rotation matrix representing the same orientation as the given {@code quaternion}.
    * 
    * @param quaternion the quaternion used to initialize this
    *  rotation matrix. Not modified.
    */
   public RotationMatrix(QuaternionReadOnly quaternion)
   {
      set(quaternion);
   }

   /**
    * Creates a new rotation matrix representing the same orientation as the given rotation vector {@code rotationVector}.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    * 
    * @param rotationVector the rotation vector used to initialize this rotation matrix. Not modified.
    */
   public RotationMatrix(VectorReadOnly rotationVector)
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

   /** {@inheritDoc} */
   @Override
   public void normalize()
   {
      Matrix3DTools.normalize(this);
   }

   /**
    * Sets the 9 coefficients of this rotation matrix without performing any checks on the data provided.
    * <p>
    * This method is meant for internal usage.
    * Prefer using {@link #set(double, double, double, double, double, double, double, double, double)}
    * or {@link #setAndNormalize(double, double, double, double, double, double, double, double, double)}.
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
      set((RotationMatrixReadOnly<?>) other);
   }

   /**
    * Sets this rotation matrix to equal the given one {@code other}.
    * 
    * @param other the other rotation matrix to copy the values from. Not modified.
    */
   public void set(RotationMatrixReadOnly<?> other)
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
    * Sets this rotation matrix to equal the 3D matrix {@code matrix} and then normalizes {@code this}.
    * 
    * @param matrix the matrix to copy the values from. Not modified.
    * @throws NotARotationMatrixException if the normalization failed.
    */
   public final void setAndNormalize(Matrix3DReadOnly<?> matrix)
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
    * Sets this rotation matrix to equal the other given one {@code other} and then normalizes {@code this}.
    * 
    * @param other the matrix to copy the values from. Not modified.
    * @throws NotARotationMatrixException if the normalization failed.
    */
   public void setAndNormalize(RotationMatrixReadOnly<?> other)
   {
      set(other);
      normalize();
   }

   /**
    * Sets this rotation matrix to the invert of the given {@code matrix}.
    * <p>
    * This operation uses the property:
    * <br> R<sup>-1</sup> = R<sup>T</sup> </br>
    * of a rotation matrix preventing to actually compute the inverse of the matrix.
    * </p>
    * 
    * @param matrix the matrix to copy the values from. Not modified.
    * @throws NotARotationMatrixException if {@code matrix} is not a
    *  rotation matrix.
    */
   public void setAndInvert(Matrix3DReadOnly<?> matrix)
   {
      setAndTranspose(matrix);
   }

   /**
    * Sets this rotation matrix to the invert of the given one {@code other}.
    * <p>
    * This operation uses the property:
    * <br> R<sup>-1</sup> = R<sup>T</sup> </br>
    * of a rotation matrix preventing to actually compute the inverse of the matrix.
    * </p>
    * 
    * @param other the matrix to copy the values from. Not modified.
    */
   public void setAndInvert(RotationMatrixReadOnly<?> other)
   {
      setAndTranspose(other);
   }

   /**
    * Sets this rotation matrix to the transpose of the given {@code matrix}.
    * 
    * @param matrix the matrix to copy the values from. Not modified.
    * @throws NotARotationMatrixException if {@code matrix} is not a rotation matrix.
    */
   public void setAndTranspose(Matrix3DReadOnly<?> matrix)
   {
      set(matrix);
      transpose();
   }

   /**
    * Sets this rotation matrix to the transpose of the given {@code other}.
    * 
    * @param other the matrix to copy the values from. Not modified.
    */
   public void setAndTranspose(RotationMatrixReadOnly<?> other)
   {
      set(other);
      transpose();
   }

   /**
    * Sets this rotation matrix to the same orientation described by the given {@code axisAngle}.
    * 
    * @param axisAngle the axis-angle used to set this matrix. Not modified.
    */
   public void set(AxisAngleReadOnly<?> axisAngle)
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
    * Sets this rotation matrix to the same orientation described by the given rotation vector {@code rotationVector}.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the same axis-angle.
    * </p>
    * 
    * @param rotation vector the rotation vector used to set this matrix. Not modified.
    */
   public void set(VectorReadOnly rotationVector)
   {
      RotationMatrixConversion.convertRotationVectorToMatrix(rotationVector, this);
   }

   /**
    * Sets this rotation matrix to represent a counter clockwise rotation around the z-axis of an angle {@code yaw}.
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
    * Sets this rotation matrix to represent a counter clockwise rotation around the y-axis of an angle {@code pitch}.
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
    * Sets this rotation matrix to represent a counter clockwise rotation around the x-axis of an angle {@code roll}.
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
    * Sets this rotation matrix to represent the same orientation as the given yaw-pitch-roll {@code yawPitchRoll}.
    * <pre>
    *        / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
    * this = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
    *        \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
    * </pre>
    * 
    * @param yawPitchRoll the yaw-pitch-roll Euler angles to copy the orientation from. Not modified.
    */
   public void setYawPitchRoll(double[] yawPitchRoll)
   {
      setYawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
   }

   /**
    * Sets this rotation matrix to represent the same orientation
    * as the given yaw-pitch-roll {@code yaw}, {@code pitch}, and {@code roll}.
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
    * Sets this rotation matrix to represent the same orientation as the given Euler angles {@code eulerAngles}.
    * <pre>
    *        / cos(eulerAngles.z) -sin(eulerAngles.z) 0 \   /  cos(eulerAngles.y) 0 sin(eulerAngles.y) \   / 1         0                   0          \
    * this = | sin(eulerAngles.z)  cos(eulerAngles.z) 0 | * |          0          1         0          | * | 0 cos(eulerAngles.x) -sin(eulerAngles.x) |
    *        \         0                   0          1 /   \ -sin(eulerAngles.y) 0 cos(eulerAngles.y) /   \ 0 sin(eulerAngles.x)  cos(eulerAngles.x) /
    * </pre>
    * <p>
    * This is equivalent to {@code this.setYawPitchRoll(eulerAngles.getZ(), eulerAngles.getY(), eulerAngles.getX())}.
    * </p>
    * 
    * @param eulerAngles the Euler angles to copy the orientation from. Not modified.
    */
   public void setEuler(VectorReadOnly eulerAngles)
   {
      setYawPitchRoll(eulerAngles.getZ(), eulerAngles.getY(), eulerAngles.getX());
   }

   /**
    * Sets this rotation matrix to represent the same orientation as the given Euler angles {@code rotX}, {@code rotY}, and {@code rotZ}.
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
    * This operation uses the property:
    * <br> R<sup>-1</sup> = R<sup>T</sup> </br>
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
   public void multiply(RotationMatrixReadOnly<?> other)
   {
      RotationMatrixTools.multiply(this, other, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this<sup>T</sup> * other
    * </p>
    * 
    * @param other the other matrix to multiply this by. Not modified.
    */
   public void multiplyTransposeThis(RotationMatrixReadOnly<?> other)
   {
      RotationMatrixTools.multiplyTransposeLeft(this, other, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this * other<sup>T</sup>
    * </p>
    * 
    * @param other the other matrix to multiply this by. Not modified.
    */
   public void multiplyTransposeOther(RotationMatrixReadOnly<?> other)
   {
      RotationMatrixTools.multiplyTransposeRight(this, other, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this<sup>T</sup> * other<sup>T</sup>
    * </p>
    * 
    * @param other the other matrix to multiply this by. Not modified.
    */
   public void multiplyTransposeBoth(RotationMatrixReadOnly<?> other)
   {
      RotationMatrixTools.multiplyTransposeBoth(this, other, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other * this
    * </p>
    * 
    * @param other the other matrix to multiply this by. Not modified.
    */
   public void preMultiply(RotationMatrixReadOnly<?> other)
   {
      RotationMatrixTools.multiply(other, this, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other * this<sup>T</sup>
    * </p>
    * 
    * @param other the other matrix to multiply this by. Not modified.
    */
   public void preMultiplyTransposeThis(RotationMatrixReadOnly<?> other)
   {
      RotationMatrixTools.multiplyTransposeRight(other, this, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other<sup>T</sup> * this
    * </p>
    * 
    * @param other the other matrix to multiply this by. Not modified.
    */
   public void preMultiplyTransposeOther(RotationMatrixReadOnly<?> other)
   {
      RotationMatrixTools.multiplyTransposeLeft(other, this, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other<sup>T</sup> * this<sup>T</sup>
    * </p>
    * 
    * @param other the other matrix to multiply this by. Not modified.
    */
   public void preMultiplyTransposeBoth(RotationMatrixReadOnly<?> other)
   {
      RotationMatrixTools.multiplyTransposeBoth(other, this, this);
   }

   /**
    * {@inheritDoc}
    * <p>
    * this = R * this
    * where 'R' is the 3-by-3 matrix representing the rotation part of the {@code transform}.
    * </p>
    * <p>
    * Note: the transformation of a {@code RotationMatrix} strongly differs from the transformation of a {@link Matrix3D}.
    * </p>
    */
   @Override
   public void applyTransform(Transform transform)
   {
      transform.transform(this);
      normalize();
   }

   // FIXME delete
   public void get(QuaternionBasics quaternionToPack)
   {
      QuaternionConversion.convertMatrixToQuaternion(this, quaternionToPack);
   }

   /**
    * Computes and packs the orientation described
    * by this rotation matrix as a rotation vector.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the same axis-angle.
    * </p>
    * 
    * @param rotationVectorToPack the rotation vector representing
    *  the same orientation as this. Modified.
    */
   public void get(VectorBasics rotationVectorToPack)
   {
      RotationVectorConversion.convertMatrixToRotationVector(this, rotationVectorToPack);
   }

   /**
    * Computes and packs the orientation described by this rotation matrix as the Euler angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is sometimes undefined.
    * </p> 
    * 
    * @param eulerAnglesToPack the tuple in which the Euler angles are stored. Modified.
    */
   public void getEuler(Tuple3DBasics eulerAnglesToPack)
   {
      YawPitchRollConversion.convertMatrixToYawPitchRoll(this, eulerAnglesToPack);
   }

   /**
    * Computes and packs the orientation described by this rotation matrix as the yaw-pitch-roll angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is sometimes undefined.
    * </p> 
    * 
    * @param yawPitchRollToPack the array in which the yaw-pitch-roll angles are stored. Modified.
    */
   public void getYawPitchRoll(double[] yawPitchRollToPack)
   {
      YawPitchRollConversion.convertMatrixToYawPitchRoll(this, yawPitchRollToPack);
   }

   /**
    * Computes and returns the yaw angle from the yaw-pitch-roll representation of this rotation matrix.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is sometimes undefined.
    * </p> 
    * 
    * @return the yaw angle around the z-axis.
    */
   public double getYaw()
   {
      return YawPitchRollConversion.computeYaw(this);
   }

   /**
    * Computes and returns the pitch angle from the yaw-pitch-roll representation of this rotation matrix.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is sometimes undefined.
    * </p> 
    * 
    * @return the pitch angle around the y-axis.
    */
   public double getPitch()
   {
      return YawPitchRollConversion.computePitch(this);
   }

   /**
    * Computes and returns the roll angle from the yaw-pitch-roll representation of this rotation matrix.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is sometimes undefined.
    * </p> 
    * 
    * @return the roll angle around the x-axis.
    */
   public double getRoll()
   {
      return YawPitchRollConversion.computeRoll(this);
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
    * Tests if the given {@code object}'s class is the same as this,
    * in which case the method returns {@link #equals(RotationMatrix)}, 
    * it returns {@code false} otherwise or if the {@code object}
    * is {@code null}.
    * 
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
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

   /**
    * Tests on a per component basis if this matrix is
    * exactly equal to {@code other}.
    * <p>
    * The method returns {@code false} if the given matrix
    * is {@code null}.
    * </p>
    * 
    * @param other the other matrix to compare against this. Not modified.
    * @return {@code true} if the two matrices are exactly equal
    *  component-wise, {@code false} otherwise.
    */
   public boolean equals(RotationMatrix other)
   {
      return Matrix3DFeatures.equals(this, other);
   }

   /**
    * Provides a {@code String} representation of this matrix as follows:
    * <br> m00, m01, m02
    * <br> m10, m11, m12
    * <br> m20, m21, m22
    * 
    * @return the {@code String} representing this matrix.
    */
   @Override
   public String toString()
   {
      return Matrix3DReadOnlyTools.toString(this);
   }

   /**
    * Calculates and returns a hash code value from the value
    * of each component of this matrix.
    * 
    * @return the hash code value for this matrix.
    */
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

   /**
    * Tests on a per component basis, if this matrix is equal to {@code other} to an {@code epsilon}.
    * 
    * @param other the other matrix to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two matrix are equal component-wise, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(RotationMatrix other, double epsilon)
   {
      return epsilonEquals((Matrix3DReadOnly<?>) other, epsilon);
   }
}
