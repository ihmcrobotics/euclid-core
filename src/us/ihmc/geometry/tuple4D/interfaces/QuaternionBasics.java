package us.ihmc.geometry.tuple4D.interfaces;

import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.geometry.interfaces.GeometryObject;
import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.geometry.transform.AffineTransform;
import us.ihmc.geometry.transform.QuaternionBasedTransform;
import us.ihmc.geometry.transform.RigidBodyTransform;
import us.ihmc.geometry.transform.interfaces.Transform;
import us.ihmc.geometry.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.tuple4D.QuaternionConversion;
import us.ihmc.geometry.tuple4D.QuaternionTools;

/**
 * Write and read interface for unit-quaternion used to represent 3D orientations.
 * <p>
 * When describing a 4D tuple, its 4 components are often gathered in two groups: the scalar part
 * {@code s} and the vector part ({@code x}, {@code y}, {@code z}).
 * </p>
 * <p>
 * Note on the difference between applying a 3D transform on a quaternion and a 4D vector:
 * <ul>
 * <li>When transformed by a homogeneous transformation matrix, a quaternion is only pre-multiplied
 * by the rotation part of the transform, resulting in concatenating the orientations of the
 * transform and the quaternion.
 * <li>When transformed by a homogeneous transformation matrix, a 4D vector scalar part {@code s}
 * remains unchanged. The vector part ({@code x}, {@code y}, {@code z}) is scaled and rotated, and
 * translated by {@code s} times the translation part of the transform. Note that for {@code s = 0},
 * a 4D vector behaves as a 3D vector, and for {@code s = 1} it behaves as a 3D point.
 * </ul>
 * </p>
 *
 * @author Sylvain Bertrand
 *
 * @param <T> The final type of the quaternion used.
 */
public interface QuaternionBasics<T extends QuaternionBasics<T>> extends QuaternionReadOnly<T>, Tuple4DBasics<T>, GeometryObject<T>
{
   /**
    * Sets the four components of this quaternion without normalizing.
    * <p>
    * This method is meant for internal usage. Prefer using
    * {@link #set(double, double, double, double)}.
    * </p>
    *
    * @param qx the x-component of this quaternion.
    * @param qy the y-component of this quaternion.
    * @param qz the z-component of this quaternion.
    * @param qs the s-component of this quaternion.
    */
   void setUnsafe(double qx, double qy, double qz, double qs);

   /**
    * Sets this quaternion to the neutral quaternion representing a 'zero' rotation.
    */
   @Override
   default void setToZero()
   {
      setUnsafe(0.0, 0.0, 0.0, 1.0);
   }

   /** {@inheritDoc} */
   @Override
   default void absolute()
   {
      setUnsafe(Math.abs(getX()), Math.abs(getY()), Math.abs(getZ()), Math.abs(getS()));
   }

   /** {@inheritDoc} */
   @Override
   default void negate()
   {
      setUnsafe(-getX(), -getY(), -getZ(), -getS());
   }

   /**
    * Sets this quaternion to its conjugate.
    *
    * <pre>
    *      / -qx \
    * q* = | -qy |
    *      | -qz |
    *      \  qs /
    * </pre>
    */
   default void conjugate()
   {
      setUnsafe(-getX(), -getY(), -getZ(), getS());
   }

   /**
    * Sets this quaternion to its inverse.
    * <p>
    * Essentially calling {@link #conjugate()} and then {@link #normalize()}.
    * </p>
    */
   default void inverse()
   {
      conjugate();
      normalize();
   }

   /** {@inheritDoc} */
   @Override
   default void normalize()
   {
      if (containsNaN())
         return;

      double invNorm = norm();

      if (invNorm == 0.0)
      {
         setToZero();
         return;
      }

      invNorm = 1.0 / invNorm;
      double qx = getX() * invNorm;
      double qy = getY() * invNorm;
      double qz = getZ() * invNorm;
      double qs = getS() * invNorm;
      setUnsafe(qx, qy, qz, qs);
   }

   /**
    * Normalizes this quaternion and then limits the angle of the rotation it represents to be &in;
    * [-<i>pi</i>;<i>pi</i>].
    */
   default void normalizeAndLimitToPiMinusPi()
   {
      normalize();

      if (getS() < 0.0)
         negate();
   }

   /** {@inheritDoc} */
   @Override
   default void set(double x, double y, double z, double s)
   {
      setUnsafe(x, y, z, s);
      normalize();
   }

   /** {@inheritDoc} */
   @Override
   default void set(T other)
   {
      setUnsafe(other.getX(), other.getY(), other.getZ(), other.getS());
   }

   default void set(QuaternionReadOnly<?> other)
   {
      setUnsafe(other.getX(), other.getY(), other.getZ(), other.getS());
   }

   /** {@inheritDoc} */
   @Override
   default void setAndNormalize(Tuple4DReadOnly<?> other)
   {
      set(other);
   }

   /**
    * Sets this quaternion to the conjugate of {@code other}.
    *
    * <pre>
    *      / -qx \
    * q* = | -qy |
    *      | -qz |
    *      \  qs /
    * </pre>
    *
    * @param other the other quaternion to copy the values from. Not modified.
    */
   default void setAndConjugate(QuaternionReadOnly<?> other)
   {
      set(other);
      conjugate();
   }

   /**
    * Sets this quaternion to {@code other} and then calls {@link #negate()}.
    *
    * @param other the other quaternion to copy the values from. Not modified.
    */
   default void setAndNegate(QuaternionReadOnly<?> other)
   {
      set(other);
      negate();
   }

   /**
    * Sets this quaternion to the inverse of {@code other}.
    *
    * @param other the other quaternion to copy the values from. Not modified.
    */
   default void setAndInverse(QuaternionReadOnly<?> other)
   {
      set(other);
      inverse();
   }

   /**
    * Sets this quaternion to the same orientation described by the given {@code axisAngle}.
    *
    * @param axisAngle the axis-angle used to set this quaternion. Not modified.
    */
   default void set(AxisAngleReadOnly axisAngle)
   {
      QuaternionConversion.convertAxisAngleToQuaternion(axisAngle, this);
   }

   /**
    * Sets this quaternion to the same orientation described by the given {@code rotationMatrix}.
    *
    * @param rotationMatrix the rotation matrix used to set this quaternion. Not modified.
    */
   default void set(RotationMatrixReadOnly rotationMatrix)
   {
      QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, this);
   }

   /**
    * Sets this quaternion to the same orientation described by the given rotation vector
    * {@code rotationVector}.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param rotation vector the rotation vector used to set this quaternion. Not modified.
    */
   default void set(Vector3DReadOnly rotationVector)
   {
      QuaternionConversion.convertRotationVectorToQuaternion(rotationVector, this);
   }

   /**
    * Sets this quaternion to represent the same orientation as the given yaw-pitch-roll
    * {@code yawPitchRoll}.
    *
    * @param yawPitchRoll the yaw-pitch-roll Euler angles to copy the orientation from. Not
    *           modified.
    */
   default void setYawPitchRoll(double[] yawPitchRoll)
   {
      QuaternionConversion.convertYawPitchRollToQuaternion(yawPitchRoll, this);
   }

   /**
    * Sets this quaternion to represent the same orientation as the given yaw-pitch-roll
    * {@code yaw}, {@code pitch}, and {@code roll}.
    *
    * @param yaw the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll the angle to rotate about the x-axis.
    */
   default void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      QuaternionConversion.convertYawPitchRollToQuaternion(yaw, pitch, roll, this);
   }

   /**
    * Sets this quaternion to represent the same orientation as the given Euler angles
    * {@code eulerAngles}.
    * <p>
    * This is equivalent to
    * {@code this.setYawPitchRoll(eulerAngles.getZ(), eulerAngles.getY(), eulerAngles.getX())}.
    * </p>
    *
    * @param eulerAngles the Euler angles to copy the orientation from. Not modified.
    */
   default void setEuler(Vector3DReadOnly eulerAngles)
   {
      QuaternionConversion.convertYawPitchRollToQuaternion(eulerAngles.getZ(), eulerAngles.getY(), eulerAngles.getX(), this);
   }

   /**
    * Sets this quaternion to represent the same orientation as the given Euler angles {@code rotX},
    * {@code rotY}, and {@code rotZ}.
    * <p>
    * This is equivalent to {@code this.setYawPitchRoll(rotZ, rotY, rotX)}.
    * </p>
    *
    * @param rotX the angle to rotate about the x-axis.
    * @param rotY the angle to rotate about the y-axis.
    * @param rotZ the angle to rotate about the z-axis.
    */
   default void setEuler(double rotX, double rotY, double rotZ)
   {
      QuaternionConversion.convertYawPitchRollToQuaternion(rotZ, rotY, rotX, this);
   }

   /**
    * Sets this quaternion to represent a counter clockwise rotation around the z-axis of an angle
    * {@code yaw}.
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   default void setToYawQuaternion(double yaw)
   {
      QuaternionConversion.computeYawQuaternion(yaw, this);
   }

   /**
    * Sets this quaternion to represent a counter clockwise rotation around the y-axis of an angle
    * {@code pitch}.
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   default void setToPitchQuaternion(double pitch)
   {
      QuaternionConversion.computePitchQuaternion(pitch, this);
   }

   /**
    * Sets this quaternion to represent a counter clockwise rotation around the x-axis of an angle
    * {@code roll}.
    *
    * @param roll the angle to rotate about the x-axis.
    */
   default void setToRollQuaternion(double roll)
   {
      QuaternionConversion.computeRollQuaternion(roll, this);
   }

   /**
    * Sets this quaternion to the difference of {@code q1} and {@code q2}.
    * <p>
    * this = q1<sup>-1</sup> * q2
    * </p>
    *
    * @param q1 the first quaternion in the difference. Not modified.
    * @param q2 the second quaternion in the difference. Not modified.
    */
   default void difference(QuaternionReadOnly<?> q1, QuaternionReadOnly<?> q2)
   {
      QuaternionTools.multiplyConjugateLeft(q1, q2, this);
   }

   /**
    * Multiplies this quaternion by {@code other}.
    * <p>
    * this = this * other
    * </p>
    *
    * @param other the other quaternion to multiply this. Not modified.
    */
   default void multiply(QuaternionReadOnly<?> other)
   {
      QuaternionTools.multiply(this, other, this);
   }

   /**
    * Sets this quaternion to the multiplication of {@code q1} and {@code q2}.
    * <p>
    * this = q1 * q2
    * </p>
    *
    * @param q1 the first quaternion in the multiplication. Not modified.
    * @param q2 the second quaternion in the multiplication. Not modified.
    */
   default void multiply(QuaternionReadOnly<?> q1, QuaternionReadOnly<?> q2)
   {
      QuaternionTools.multiply(q1, q2, this);
   }

   /**
    * Multiplies this quaternion by {@code matrix}.
    * <p>
    * this = this * Q(matrix)<br>
    * where Q(matrix) is the equivalent quaternion for the given rotation matrix.
    * </p>
    *
    * @param matrix the rotation matrix to multiply this. Not modified.
    */
   default void multiply(RotationMatrixReadOnly matrix)
   {
      QuaternionTools.multiply(this, matrix, this);
   }

   /**
    * Multiplies this quaternion by the conjugate of {@code other}.
    * <p>
    * this = this * other*
    * </p>
    *
    * @param other the other quaternion to multiply this. Not modified.
    */
   default void multiplyConjugateOther(QuaternionReadOnly<?> other)
   {
      QuaternionTools.multiplyConjugateRight(this, other, this);
   }

   /**
    * Sets this quaternion to the multiplication of the conjugate of {@code this} and {@code other}.
    * <p>
    * this = this* * other
    * </p>
    *
    * @param other the other quaternion to multiply this. Not modified.
    */
   default void multiplyConjugateThis(QuaternionReadOnly<?> other)
   {
      QuaternionTools.multiplyConjugateLeft(this, other, this);
   }

   /**
    * Pre-multiplies this quaternion by {@code other}.
    * <p>
    * this = other * other
    * </p>
    *
    * @param other the other quaternion to multiply this. Not modified.
    */
   default void preMultiply(QuaternionReadOnly<?> other)
   {
      QuaternionTools.multiply(other, this, this);
   }

   /**
    * Pre-multiplies this quaternion by {@code matrix}.
    * <p>
    * this = Q(matrix) * this<br>
    * where Q(matrix) is the equivalent quaternion for the given rotation matrix.
    * </p>
    *
    * @param matrix the rotation matrix to multiply this. Not modified.
    */
   default void preMultiply(RotationMatrixReadOnly matrix)
   {
      QuaternionTools.multiply(matrix, this, this);
   }

   /**
    * Sets this quaternion to the multiplication of the conjugate of {@code other} and {@code this}.
    * <p>
    * this = other* * this
    * </p>
    *
    * @param other the other quaternion to multiply this. Not modified.
    */
   default void preMultiplyConjugateOther(QuaternionReadOnly<?> other)
   {
      QuaternionTools.multiplyConjugateLeft(other, this, this);
   }

   /**
    * Sets this quaternion to the multiplication of {@code other} and the conjugate of {@code this}.
    * <p>
    * this = other * this*
    * </p>
    *
    * @param other the other quaternion to multiply this. Not modified.
    */
   default void preMultiplyConjugateThis(QuaternionReadOnly<?> other)
   {
      QuaternionTools.multiplyConjugateRight(other, this, this);
   }

   /**
    * Performs a linear interpolation in SO(3) from {@code this} to {@code qf} given the percentage
    * {@code alpha}.
    * <p>
    * The interpolation method used here is often called a <i>Spherical Linear Interpolation</i> or
    * SLERP.
    * </p>
    *
    * @param qf the other quaternion used for the interpolation. Not modified.
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not
    *           modifying this quaternion, while a value of 1 is equivalent to setting this
    *           quaternion to {@code qf}.
    */
   default void interpolate(QuaternionReadOnly<?> qf, double alpha)
   {
      interpolate(this, qf, alpha);
   }

   /**
    * Performs a linear interpolation in SO(3) from {@code q0} to {@code qf} given the percentage
    * {@code alpha}.
    * <p>
    * The interpolation method used here is often called a <i>Spherical Linear Interpolation</i> or
    * SLERP.
    * </p>
    *
    * @param q0 the first quaternion used in the interpolation. Not modified.
    * @param qf the second quaternion used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           this quaternion to {@code q0}, while a value of 1 is equivalent to setting this
    *           quaternion to {@code qf}.
    */
   default void interpolate(QuaternionReadOnly<?> q0, QuaternionReadOnly<?> qf, double alpha)
   {
      double cosHalfTheta = q0.dot(qf);
      double sign = 1.0;

      if (cosHalfTheta < 0.0)
      {
         sign = -1.0;
         cosHalfTheta = -cosHalfTheta;
      }

      double alpha0 = 1.0 - alpha;
      double alphaf = alpha;

      if (1.0 - cosHalfTheta > 1.0e-12)
      {
         double halfTheta = Math.acos(cosHalfTheta);
         double sinHalfTheta = Math.sin(halfTheta);
         alpha0 = Math.sin(alpha0 * halfTheta) / sinHalfTheta;
         alphaf = Math.sin(alphaf * halfTheta) / sinHalfTheta;
      }

      double qx = alpha0 * q0.getX() + sign * alphaf * qf.getX();
      double qy = alpha0 * q0.getY() + sign * alphaf * qf.getY();
      double qz = alpha0 * q0.getZ() + sign * alphaf * qf.getZ();
      double qs = alpha0 * q0.getS() + sign * alphaf * qf.getS();
      set(qx, qy, qz, qs);
   }

   /**
    * Transforms this quaternion using the given {@code transform}.
    * <p>
    * {@link RigidBodyTransform}, {@link QuaternionBasedTransform}, and {@link AffineTransform}
    * prepend their rotation part the given quaternion. No scale or translation is applied to the
    * quaternion such that the output of this method is still a unit-quaternion.
    * </p>
    *
    * @param transform the geometric transform to apply on this vector. Not modified.
    */
   @Override
   default void applyTransform(Transform transform)
   {
      transform.transform(this);
   }
}
