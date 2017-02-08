package us.ihmc.geometry.tuple4D.interfaces;

import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.geometry.interfaces.GeometryObject;
import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.geometry.transform.interfaces.Transform;
import us.ihmc.geometry.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.tuple4D.QuaternionConversion;
import us.ihmc.geometry.tuple4D.QuaternionTools;

public interface QuaternionBasics<T extends QuaternionBasics<T>> extends QuaternionReadOnly<T>, Tuple4DBasics<T>, GeometryObject<T>
{
   void setUnsafe(double qx, double qy, double qz, double qs);

   @Override
   default void setToZero()
   {
      setUnsafe(0.0, 0.0, 0.0, 1.0);
   }

   @Override
   default void absolute()
   {
      setUnsafe(Math.abs(getX()), Math.abs(getY()), Math.abs(getZ()), Math.abs(getS()));
   }

   @Override
   default void negate()
   {
      setUnsafe(-getX(), -getY(), -getZ(), -getS());
   }

   default void conjugate()
   {
      setUnsafe(-getX(), -getY(), -getZ(), getS());
   }

   default void inverse()
   {
      conjugate();
      normalize();
   }

   @Override
   default void normalize()
   {
      QuaternionTools.normalize(this);
   }

   default void normalizeAndLimitToPiMinusPi()
   {
      QuaternionTools.normalizeAndLimitToPiMinusPi(this);
   }

   @Override
   default void set(double x, double y, double z, double s)
   {
      setUnsafe(x, y, z, s);
      normalize();
   }

   @Override
   default void set(T other)
   {
      setUnsafe(other.getX(), other.getY(), other.getZ(), other.getS());
   }

   default void set(QuaternionReadOnly<?> other)
   {
      setUnsafe(other.getX(), other.getY(), other.getZ(), other.getS());
   }

   @Override
   default void setAndNormalize(Tuple4DReadOnly<?> other)
   {
      set(other);
   }

   default void setAndConjugate(QuaternionReadOnly<?> other)
   {
      set(other);
      conjugate();
   }

   default void setAndNegate(QuaternionReadOnly<?> other)
   {
      set(other);
      negate();
   }

   default void setAndInverse(QuaternionReadOnly<?> other)
   {
      set(other);
      inverse();
   }

   default void set(AxisAngleReadOnly<?> axisAngle)
   {
      QuaternionConversion.convertAxisAngleToQuaternion(axisAngle, this);
   }

   default void set(RotationMatrixReadOnly<?> rotationMatrix)
   {
      QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, this);
   }

   default void set(Vector3DReadOnly<?> rotationVector)
   {
      QuaternionConversion.convertRotationVectorToQuaternion(rotationVector, this);
   }

   default void setYawPitchRoll(double[] yawPitchRoll)
   {
      QuaternionConversion.convertYawPitchRollToQuaternion(yawPitchRoll, this);
   }

   default void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      QuaternionConversion.convertYawPitchRollToQuaternion(yaw, pitch, roll, this);
   }

   default void setEuler(Vector3DReadOnly<?> eulerAngles)
   {
      QuaternionConversion.convertYawPitchRollToQuaternion(eulerAngles.getZ(), eulerAngles.getY(), eulerAngles.getX(), this);
   }

   default void setEuler(double rotX, double rotY, double rotZ)
   {
      QuaternionConversion.convertYawPitchRollToQuaternion(rotZ, rotY, rotX, this);
   }

   default void setToYawQuaternion(double yaw)
   {
      QuaternionConversion.computeYawQuaternion(yaw, this);
   }

   default void setToPitchQuaternion(double pitch)
   {
      QuaternionConversion.computePitchQuaternion(pitch, this);
   }

   default void setToRollQuaternion(double roll)
   {
      QuaternionConversion.computeRollQuaternion(roll, this);
   }

   default void difference(QuaternionReadOnly<?> q1, QuaternionReadOnly<?> q2)
   {
      QuaternionTools.multiplyConjugateLeft(q1, q2, this);
   }

   default void multiply(QuaternionReadOnly<?> other)
   {
      QuaternionTools.multiply(this, other, this);
   }

   default void multiply(QuaternionReadOnly<?> q1, QuaternionReadOnly<?> q2)
   {
      QuaternionTools.multiply(q1, q2, this);
   }

   default void multiply(RotationMatrixReadOnly<?> matrix)
   {
      QuaternionTools.multiply(this, matrix, this);
   }

   default void multiplyConjugateOther(QuaternionReadOnly<?> other)
   {
      QuaternionTools.multiplyConjugateRight(this, other, this);
   }

   default void multiplyConjugateThis(QuaternionReadOnly<?> other)
   {
      QuaternionTools.multiplyConjugateLeft(this, other, this);
   }

   default void preMultiply(QuaternionReadOnly<?> other)
   {
      QuaternionTools.multiply(other, this, this);
   }

   default void preMultiply(RotationMatrixReadOnly<?> matrix)
   {
      QuaternionTools.multiply(matrix, this, this);
   }

   default void preMultiplyConjugateOther(QuaternionReadOnly<?> other)
   {
      QuaternionTools.multiplyConjugateLeft(other, this, this);
   }

   default void preMultiplyConjugateThis(QuaternionReadOnly<?> other)
   {
      QuaternionTools.multiplyConjugateRight(other, this, this);
   }

   default void interpolate(QuaternionReadOnly<?> q1, double alpha)
   {
      interpolate(this, q1, alpha);
   }

   default void interpolate(QuaternionReadOnly<?> q1, QuaternionReadOnly<?> q2, double alpha)
   {
      QuaternionTools.interpolate(q1, q2, alpha, this);
   }

   @Override
   default void applyTransform(Transform transform)
   {
      transform.transform(this);
   }
}
