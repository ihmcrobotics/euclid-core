package us.ihmc.geometry.tuple4D;

import us.ihmc.geometry.exceptions.NotAMatrix2DException;
import us.ihmc.geometry.matrix.Matrix3D;
import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.matrix.RotationMatrixConversion;
import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Tuple4DReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DReadOnly;

public abstract class QuaternionTools
{
   private static final double EPS = 1.0e-12;
   public static final double EPS_NORM_FAST_SQRT = 2.107342e-08;

   public static boolean isQuaternionZOnly(double qx, double qy, double qz, double qs)
   {
      return isQuaternionZOnly(qx, qy, qz, qs, EPS);
   }

   public static boolean isQuaternionZOnly(double qx, double qy, double qz, double qs, double epsilon)
   {
      return Math.abs(qx) < epsilon && Math.abs(qy) < epsilon;
   }

   public static boolean isQuaternionZOnly(QuaternionReadOnly<?> quaternion)
   {
      return isQuaternionZOnly(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getS());
   }

   public static void checkIfQuaternionIsZOnly(double qx, double qy, double qz, double qs)
   {
      boolean isZOnly = isQuaternionZOnly(qx, qy, qz, qs);
      if (!isZOnly)
         throw new NotAMatrix2DException("The quaternion is not in XY plane: " + Tuple4DTools.toString(qx, qy, qz, qs));
   }

   public static void checkIfQuaternionIsZOnly(QuaternionReadOnly<?> quaternion)
   {
      checkIfQuaternionIsZOnly(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getS());
   }

   public static void interpolate(QuaternionReadOnly<?> q0, QuaternionReadOnly<?> qf, double alpha, QuaternionBasics<?> quaternionToPack)
   {
      double cosHalfTheta = Tuple4DTools.dot(q0, qf);
      double sign = 1.0;

      if (cosHalfTheta < 0.0)
      {
         sign = -1.0;
         cosHalfTheta = -cosHalfTheta;
      }

      double alpha0 = 1.0 - alpha;
      double alphaf = alpha;

      if (1.0 - cosHalfTheta > EPS)
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
      quaternionToPack.set(qx, qy, qz, qs);
   }

   public static void multiply(QuaternionReadOnly<?> q1, QuaternionReadOnly<?> q2, QuaternionBasics<?> quaternionToPack)
   {
      multiplyImpl(q1, false, q2, false, quaternionToPack);
   }

   public static void multiplyConjugateLeft(QuaternionReadOnly<?> q1, QuaternionReadOnly<?> q2, QuaternionBasics<?> quaternionToPack)
   {
      multiplyImpl(q1, true, q2, false, quaternionToPack);
   }

   public static void multiplyConjugateRight(QuaternionReadOnly<?> q1, QuaternionReadOnly<?> q2, QuaternionBasics<?> quaternionToPack)
   {
      multiplyImpl(q1, false, q2, true, quaternionToPack);
   }

   private static void multiplyImpl(QuaternionReadOnly<?> q1, boolean conjugateQ1, QuaternionReadOnly<?> q2, boolean conjugateQ2, QuaternionBasics<?> quaternionToPack)
   {
      multiplyImpl(q1.getX(), q1.getY(), q1.getZ(), q1.getS(), conjugateQ1, q2.getX(), q2.getY(), q2.getZ(), q2.getS(), conjugateQ2, quaternionToPack);
   }

   private static void multiplyImpl(double q1x, double q1y, double q1z, double q1s, boolean conjugateQ1, double q2x, double q2y, double q2z, double q2s,
                                    boolean conjugateq2, QuaternionBasics<?> quaternionToPack)
   {
      if (conjugateQ1)
      {
         q1x = -q1x;
         q1y = -q1y;
         q1z = -q1z;
      }

      if (conjugateq2)
      {
         q2x = -q2x;
         q2y = -q2y;
         q2z = -q2z;
      }

      double x = q1s * q2x + q1x * q2s + q1y * q2z - q1z * q2y;
      double y = q1s * q2y - q1x * q2z + q1y * q2s + q1z * q2x;
      double z = q1s * q2z + q1x * q2y - q1y * q2x + q1z * q2s;
      double s = q1s * q2s - q1x * q2x - q1y * q2y - q1z * q2z;
      quaternionToPack.set(x, y, z, s);
   }

   public static void multiply(Tuple4DReadOnly<?> t1, Tuple4DReadOnly<?> t2, Vector4DBasics<?> vectorToPack)
   {
      multiplyImpl(t1, false, t2, false, vectorToPack);
   }

   public static void multiplyConjugateLeft(Tuple4DReadOnly<?> t1, Tuple4DReadOnly<?> t2, Vector4DBasics<?> vectorToPack)
   {
      multiplyImpl(t1, true, t2, false, vectorToPack);
   }

   public static void multiplyConjugateRight(Tuple4DReadOnly<?> t1, Tuple4DReadOnly<?> t2, Vector4DBasics<?> vectorToPack)
   {
      multiplyImpl(t1, false, t2, true, vectorToPack);
   }

   private static void multiplyImpl(Tuple4DReadOnly<?> t1, boolean conjugateT1, Tuple4DReadOnly<?> t2, boolean conjugateT2, Vector4DBasics<?> vectorToPack)
   {
      multiplyImpl(t1.getX(), t1.getY(), t1.getZ(), t1.getS(), conjugateT1, t2.getX(), t2.getY(), t2.getZ(), t2.getS(), conjugateT2, vectorToPack);
   }

   private static void multiplyImpl(double x1, double y1, double z1, double s1, boolean conjugateT1, double x2, double y2, double z2, double s2,
                                    boolean conjugateT2, Vector4DBasics<?> vectorToPack)
   {
      if (conjugateT1)
      {
         x1 = -x1;
         y1 = -y1;
         z1 = -z1;
      }

      if (conjugateT2)
      {
         x2 = -x2;
         y2 = -y2;
         z2 = -z2;
      }

      double x = s1 * x2 + x1 * s2 + y1 * z2 - z1 * y2;
      double y = s1 * y2 - x1 * z2 + y1 * s2 + z1 * x2;
      double z = s1 * z2 + x1 * y2 - y1 * x2 + z1 * s2;
      double s = s1 * s2 - x1 * x2 - y1 * y2 - z1 * z2;
      vectorToPack.set(x, y, z, s);
   }

   public static void normalize(QuaternionBasics<?> q)
   {
      if (Tuple4DTools.containsNaN(q))
         return;

      double invNorm = norm(q);

      if (invNorm == 0.0)
      {
         q.setToZero();
         return;
      }

      invNorm = 1.0 / invNorm;
      double qx = q.getX() * invNorm;
      double qy = q.getY() * invNorm;
      double qz = q.getZ() * invNorm;
      double qs = q.getS() * invNorm;
      q.setUnsafe(qx, qy, qz, qs);
   }

   public static void normalizeAndLimitToPiMinusPi(QuaternionBasics<?> q)
   {
      normalize(q);

      if (q.getS() < 0.0)
         q.negate();
   }

   public static double normSquared(QuaternionReadOnly<?> quaternion)
   {
      return normSquared(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getS());
   }

   public static double normSquared(double qx, double qy, double qz, double qs)
   {
      return qx * qx + qy * qy + qz * qz + qs * qs;
   }

   public static double norm(QuaternionReadOnly<?> quaternion)
   {
      return norm(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getS());
   }

   public static double norm(double qx, double qy, double qz, double qs)
   {
      double norm = normSquared(qx, qy, qz, qs);

      if (Math.abs(1.0 - norm) < EPS_NORM_FAST_SQRT)
         norm = 0.5 * (1.0 + norm);
      else
         norm = Math.sqrt(norm);

      return norm;
   }

   public static void transform(QuaternionReadOnly<?> quaternion, Tuple3DReadOnly<?> tupleOriginal, Tuple3DBasics<?> tupleTransformed)
   {
      transformImpl(quaternion, false, tupleOriginal, tupleTransformed);
   }

   public static void inverseTransform(QuaternionReadOnly<?> quaternion, Tuple3DReadOnly<?> tupleOriginal, Tuple3DBasics<?> tupleTransformed)
   {
      transformImpl(quaternion, true, tupleOriginal, tupleTransformed);
   }

   private static void transformImpl(QuaternionReadOnly<?> quaternion, boolean conjugateQuaternion, Tuple3DReadOnly<?> tupleOriginal,
                                     Tuple3DBasics<?> tupleTransformed)
   {
      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      if (conjugateQuaternion)
      {
         qx = -qx;
         qy = -qy;
         qz = -qz;
      }

      double norm = norm(qx, qy, qz, qs);

      if (norm < EPS)
      {
         tupleTransformed.set(tupleOriginal);
         return;
      }

      norm = 1.0 / norm;
      qx *= norm;
      qy *= norm;
      qz *= norm;
      qs *= norm;

      // t = 2.0 * cross(q.xyz, v);
      // v' = v + q.s * t + cross(q.xyz, t);
      double x = tupleOriginal.getX();
      double y = tupleOriginal.getY();
      double z = tupleOriginal.getZ();

      double crossX = 2.0 * (qy * z - qz * y);
      double crossY = 2.0 * (qz * x - qx * z);
      double crossZ = 2.0 * (qx * y - qy * x);

      double crossCrossX = qy * crossZ - qz * crossY;
      double crossCrossY = qz * crossX - qx * crossZ;
      double crossCrossZ = qx * crossY - qy * crossX;

      tupleTransformed.setX(x + qs * crossX + crossCrossX);
      tupleTransformed.setY(y + qs * crossY + crossCrossY);
      tupleTransformed.setZ(z + qs * crossZ + crossCrossZ);
   }

   public static void addTransform(QuaternionReadOnly<?> quaternion, Tuple3DReadOnly<?> tupleOriginal, Tuple3DBasics<?> tupleTransformed)
   {
      addTransform(quaternion, false, tupleOriginal, tupleTransformed);
   }

   private static void addTransform(QuaternionReadOnly<?> quaternion, boolean conjugateQuaternion, Tuple3DReadOnly<?> tupleOriginal,
                                    Tuple3DBasics<?> tupleTransformed)
   {
      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      if (conjugateQuaternion)
      {
         qx = -qx;
         qy = -qy;
         qz = -qz;
      }

      double norm = norm(qx, qy, qz, qs);

      if (norm < EPS)
      {
         tupleTransformed.set(tupleOriginal);
         return;
      }

      norm = 1.0 / norm;
      qx *= norm;
      qy *= norm;
      qz *= norm;
      qs *= norm;

      // t = 2.0 * cross(q.xyz, v);
      // v' = v + q.s * t + cross(q.xyz, t);
      double x = tupleOriginal.getX();
      double y = tupleOriginal.getY();
      double z = tupleOriginal.getZ();

      double crossX = 2.0 * (qy * z - qz * y);
      double crossY = 2.0 * (qz * x - qx * z);
      double crossZ = 2.0 * (qx * y - qy * x);

      double crossCrossX = qy * crossZ - qz * crossY;
      double crossCrossY = qz * crossX - qx * crossZ;
      double crossCrossZ = qx * crossY - qy * crossX;

      tupleTransformed.setX(tupleTransformed.getX() + x + qs * crossX + crossCrossX);
      tupleTransformed.setY(tupleTransformed.getY() + y + qs * crossY + crossCrossY);
      tupleTransformed.setZ(tupleTransformed.getZ() + z + qs * crossZ + crossCrossZ);
   }

   public static void transform(QuaternionReadOnly<?> quaternion, Tuple2DReadOnly<?> tupleOriginal, Tuple2DBasics<?> tupleTransformed,
                                boolean checkIfTransformInXYPlane)
   {
      transformImpl(quaternion, false, tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   public static void inverseTransform(QuaternionReadOnly<?> quaternion, Tuple2DReadOnly<?> tupleOriginal, Tuple2DBasics<?> tupleTransformed,
                                       boolean checkIfTransformInXYPlane)
   {
      transformImpl(quaternion, true, tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   private static void transformImpl(QuaternionReadOnly<?> quaternion, boolean conjugateQuaternion, Tuple2DReadOnly<?> tupleOriginal,
                                     Tuple2DBasics<?> tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      if (conjugateQuaternion)
      {
         qx = -qx;
         qy = -qy;
         qz = -qz;
      }

      if (checkIfTransformInXYPlane)
         checkIfQuaternionIsZOnly(qx, qy, qz, qs);

      double norm = norm(qx, qy, qz, qs);

      if (norm < EPS)
      {
         tupleTransformed.set(tupleOriginal);
         return;
      }

      norm = 1.0 / norm;
      qz *= norm;
      qs *= norm;

      // t = 2.0 * cross(q.xyz, v);
      // v' = v + q.s * t + cross(q.xyz, t);
      double x = tupleOriginal.getX();
      double y = tupleOriginal.getY();

      double crossX = -2.0 * qz * y;
      double crossY = 2.0 * qz * x;

      double crossCrossX = -qz * crossY;
      double crossCrossY = qz * crossX;

      tupleTransformed.setX(x + qs * crossX + crossCrossX);
      tupleTransformed.setY(y + qs * crossY + crossCrossY);
   }

   public static void transform(QuaternionReadOnly<?> quaternion, QuaternionReadOnly<?> quaternionOriginal, QuaternionBasics<?> quaternionTransformed)
   {
      multiplyImpl(quaternion, false, quaternionOriginal, false, quaternionTransformed);
   }

   public static void inverseTransform(QuaternionReadOnly<?> quaternion, QuaternionReadOnly<?> quaternionOriginal, QuaternionBasics<?> quaternionTransformed)
   {
      multiplyImpl(quaternion, true, quaternionOriginal, false, quaternionTransformed);
   }

   public static void transform(QuaternionReadOnly<?> quaternion, Vector4DReadOnly<?> vectorOriginal, Vector4DBasics<?> vectorTransformed)
   {
      transformImpl(quaternion, false, vectorOriginal, vectorTransformed);
   }

   public static void inverseTransform(QuaternionReadOnly<?> quaternion, Vector4DReadOnly<?> vectorOriginal, Vector4DBasics<?> vectorTransformed)
   {
      transformImpl(quaternion, true, vectorOriginal, vectorTransformed);
   }

   private static void transformImpl(QuaternionReadOnly<?> quaternion, boolean conjugateQuaternion, Vector4DReadOnly<?> vectorOriginal,
                                     Vector4DBasics<?> vectorTransformed)
   {
      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      if (conjugateQuaternion)
      {
         qx = -qx;
         qy = -qy;
         qz = -qz;
      }

      // It's the same as transforming a vector 3D. The scalar of the transformed vector 4D is the same as the original.
      double norm = norm(qx, qy, qz, qs);

      if (norm < EPS)
      {
         vectorTransformed.set(vectorOriginal);
         return;
      }

      norm = 1.0 / norm;
      qx *= norm;
      qy *= norm;
      qz *= norm;
      qs *= norm;

      // t = 2.0 * cross(q.xyz, v);
      // v' = v + q.s * t + cross(q.xyz, t);
      double x = vectorOriginal.getX();
      double y = vectorOriginal.getY();
      double z = vectorOriginal.getZ();

      double crossX = 2.0 * (qy * z - qz * y);
      double crossY = 2.0 * (qz * x - qx * z);
      double crossZ = 2.0 * (qx * y - qy * x);

      double crossCrossX = qy * crossZ - qz * crossY;
      double crossCrossY = qz * crossX - qx * crossZ;
      double crossCrossZ = qx * crossY - qy * crossX;

      vectorTransformed.setX(x + qs * crossX + crossCrossX);
      vectorTransformed.setY(y + qs * crossY + crossCrossY);
      vectorTransformed.setZ(z + qs * crossZ + crossCrossZ);
      vectorTransformed.setS(vectorOriginal.getS());
   }

   public static void transform(QuaternionReadOnly<?> quaternion, Matrix3DReadOnly<?> matrixOriginal, Matrix3D matrixTransformed)
   {
      transformImpl(quaternion, false, matrixOriginal, matrixTransformed);
   }

   public static void inverseTransform(QuaternionReadOnly<?> quaternion, Matrix3DReadOnly<?> matrixOriginal, Matrix3D matrixTransformed)
   {
      transformImpl(quaternion, true, matrixOriginal, matrixTransformed);
   }

   private static void transformImpl(QuaternionReadOnly<?> quaternion, boolean conjugateQuaternion, Matrix3DReadOnly<?> matrixOriginal, Matrix3D matrixTransformed)
   {
      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      if (conjugateQuaternion)
      {
         qx = -qx;
         qy = -qy;
         qz = -qz;
      }

      double norm = norm(qx, qy, qz, qs);

      if (norm < EPS)
      {
         matrixTransformed.set(matrixOriginal);
         return;
      }

      norm = 1.0 / norm;
      qx *= norm;
      qy *= norm;
      qz *= norm;
      qs *= norm;

      double yy2 = 2.0 * qy * qy;
      double zz2 = 2.0 * qz * qz;
      double xx2 = 2.0 * qx * qx;
      double xy2 = 2.0 * qx * qy;
      double sz2 = 2.0 * qs * qz;
      double xz2 = 2.0 * qx * qz;
      double sy2 = 2.0 * qs * qy;
      double yz2 = 2.0 * qy * qz;
      double sx2 = 2.0 * qs * qx;

      double q00 = 1.0 - yy2 - zz2;
      double q01 = xy2 - sz2;
      double q02 = xz2 + sy2;
      double q10 = xy2 + sz2;
      double q11 = 1.0 - xx2 - zz2;
      double q12 = yz2 - sx2;
      double q20 = xz2 - sy2;
      double q21 = yz2 + sx2;
      double q22 = 1.0 - xx2 - yy2;

      double qM00 = q00 * matrixOriginal.getM00() + q01 * matrixOriginal.getM10() + q02 * matrixOriginal.getM20();
      double qM01 = q00 * matrixOriginal.getM01() + q01 * matrixOriginal.getM11() + q02 * matrixOriginal.getM21();
      double qM02 = q00 * matrixOriginal.getM02() + q01 * matrixOriginal.getM12() + q02 * matrixOriginal.getM22();
      double qM10 = q10 * matrixOriginal.getM00() + q11 * matrixOriginal.getM10() + q12 * matrixOriginal.getM20();
      double qM11 = q10 * matrixOriginal.getM01() + q11 * matrixOriginal.getM11() + q12 * matrixOriginal.getM21();
      double qM12 = q10 * matrixOriginal.getM02() + q11 * matrixOriginal.getM12() + q12 * matrixOriginal.getM22();
      double qM20 = q20 * matrixOriginal.getM00() + q21 * matrixOriginal.getM10() + q22 * matrixOriginal.getM20();
      double qM21 = q20 * matrixOriginal.getM01() + q21 * matrixOriginal.getM11() + q22 * matrixOriginal.getM21();
      double qM22 = q20 * matrixOriginal.getM02() + q21 * matrixOriginal.getM12() + q22 * matrixOriginal.getM22();

      double qMqt00 = qM00 * q00 + qM01 * q01 + qM02 * q02;
      double qMqt01 = qM00 * q10 + qM01 * q11 + qM02 * q12;
      double qMqt02 = qM00 * q20 + qM01 * q21 + qM02 * q22;
      double qMqt10 = qM10 * q00 + qM11 * q01 + qM12 * q02;
      double qMqt11 = qM10 * q10 + qM11 * q11 + qM12 * q12;
      double qMqt12 = qM10 * q20 + qM11 * q21 + qM12 * q22;
      double qMqt20 = qM20 * q00 + qM21 * q01 + qM22 * q02;
      double qMqt21 = qM20 * q10 + qM21 * q11 + qM22 * q12;
      double qMqt22 = qM20 * q20 + qM21 * q21 + qM22 * q22;

      matrixTransformed.set(qMqt00, qMqt01, qMqt02, qMqt10, qMqt11, qMqt12, qMqt20, qMqt21, qMqt22);
   }

   public static void transform(QuaternionReadOnly<?> quaternion, RotationMatrixReadOnly<?> rotationMatrixOriginal, RotationMatrix rotationMatrixTransformed)
   {
      multiplyImpl(quaternion, false, rotationMatrixOriginal, false, rotationMatrixTransformed);
   }

   public static void inverseTransform(QuaternionReadOnly<?> quaternion, RotationMatrixReadOnly<?> rotationMatrixOriginal,
                                       RotationMatrix rotationMatrixTransformed)
   {
      multiplyImpl(quaternion, true, rotationMatrixOriginal, false, rotationMatrixTransformed);
   }

   public static void multiply(QuaternionReadOnly<?> quaternion, RotationMatrixReadOnly<?> matrix, QuaternionBasics<?> quaternionToPack)
   {
      multiplyImpl(quaternion, false, matrix, false, quaternionToPack);
   }

   public static void multiplyConjugateQuaternion(QuaternionReadOnly<?> quaternion, RotationMatrixReadOnly<?> matrix, QuaternionBasics<?> quaternionToPack)
   {
      multiplyImpl(quaternion, true, matrix, false, quaternionToPack);
   }

   public static void multiplyTransposeMatrix(QuaternionReadOnly<?> quaternion, RotationMatrixReadOnly<?> matrix, QuaternionBasics<?> quaternionToPack)
   {
      multiplyImpl(quaternion, false, matrix, true, quaternionToPack);
   }

   public static void multiplyConjugateQuaternionTransposeMatrix(QuaternionReadOnly<?> quaternion, RotationMatrixReadOnly<?> matrix,
                                                                 QuaternionBasics<?> quaternionToPack)
   {
      multiplyImpl(quaternion, true, matrix, true, quaternionToPack);
   }

   private static void multiplyImpl(QuaternionReadOnly<?> quaternion, boolean conjugateQuaternion, RotationMatrixReadOnly<?> matrix, boolean transposeMatrix,
                                    QuaternionBasics<?> quaternionToPack)
   {
      double q1x = quaternion.getX();
      double q1y = quaternion.getY();
      double q1z = quaternion.getZ();
      double q1s = quaternion.getS();

      QuaternionConversion.convertMatrixToQuaternion(matrix, quaternionToPack);
      double q2x = quaternionToPack.getX();
      double q2y = quaternionToPack.getY();
      double q2z = quaternionToPack.getZ();
      double q2s = quaternionToPack.getS();

      multiplyImpl(q1x, q1y, q1z, q1s, conjugateQuaternion, q2x, q2y, q2z, q2s, transposeMatrix, quaternionToPack);
   }

   public static void multiply(RotationMatrixReadOnly<?> matrix, QuaternionReadOnly<?> quaternion, QuaternionBasics<?> quaternionToPack)
   {
      multiplyImpl(matrix, false, quaternion, false, quaternionToPack);
   }

   public static void multiplyConjugateQuaternion(RotationMatrixReadOnly<?> matrix, QuaternionReadOnly<?> quaternion, QuaternionBasics<?> quaternionToPack)
   {
      multiplyImpl(matrix, false, quaternion, true, quaternionToPack);
   }

   public static void multiplyTransposeMatrix(RotationMatrixReadOnly<?> matrix, QuaternionReadOnly<?> quaternion, QuaternionBasics<?> quaternionToPack)
   {
      multiplyImpl(matrix, true, quaternion, false, quaternionToPack);
   }

   public static void multiplyTransposeMatrixConjugateQuaternion(RotationMatrixReadOnly<?> matrix, QuaternionReadOnly<?> quaternion,
                                                                 QuaternionBasics<?> quaternionToPack)
   {
      multiplyImpl(matrix, true, quaternion, true, quaternionToPack);
   }

   private static void multiplyImpl(RotationMatrixReadOnly<?> matrix, boolean transposeMatrix, QuaternionReadOnly<?> quaternion, boolean conjugateQuaternion,
                                    QuaternionBasics<?> quaternionToPack)
   {
      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      if (conjugateQuaternion)
      {
         qx = -qx;
         qy = -qy;
         qz = -qz;
      }

      QuaternionConversion.convertMatrixToQuaternion(matrix, quaternionToPack);
      double q1x = quaternionToPack.getX();
      double q1y = quaternionToPack.getY();
      double q1z = quaternionToPack.getZ();
      double q1s = quaternionToPack.getS();

      if (transposeMatrix)
      {
         q1x = -q1x;
         q1y = -q1y;
         q1z = -q1z;
      }

      multiplyImpl(q1x, q1y, q1z, q1s, false, qx, qy, qz, qs, false, quaternionToPack);
   }

   public static void multiply(QuaternionReadOnly<?> quaternion, RotationMatrixReadOnly<?> matrix, RotationMatrix matrixToPack)
   {
      multiplyImpl(quaternion, false, matrix, false, matrixToPack);
   }

   public static void multiplyConjugateQuaternion(QuaternionReadOnly<?> quaternion, RotationMatrixReadOnly<?> matrix, RotationMatrix matrixToPack)
   {
      multiplyImpl(quaternion, true, matrix, false, matrixToPack);
   }

   public static void multiplyTransposeMatrix(QuaternionReadOnly<?> quaternion, RotationMatrixReadOnly<?> matrix, RotationMatrix matrixToPack)
   {
      multiplyImpl(quaternion, false, matrix, true, matrixToPack);
   }

   public static void multiplyConjugateQuaternionTransposeMatrix(QuaternionReadOnly<?> quaternion, RotationMatrixReadOnly<?> matrix, RotationMatrix matrixToPack)
   {
      multiplyImpl(quaternion, true, matrix, false, matrixToPack);
   }

   private static void multiplyImpl(QuaternionReadOnly<?> quaternion, boolean conjugateQuaternion, RotationMatrixReadOnly<?> matrix, boolean transposeMatrix,
                                    RotationMatrix matrixToPack)
   {
      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      if (conjugateQuaternion)
      {
         qx = -qx;
         qy = -qy;
         qz = -qz;
      }

      if (matrix != matrixToPack)
      {
         RotationMatrixConversion.convertQuaternionToMatrixImpl(qx, qy, qz, qs, matrixToPack);
         matrixToPack.multiply(matrix);
      }
      else
      {
         double norm = norm(qx, qy, qz, qs);

         if (norm < EPS)
         {
            matrixToPack.set(matrix);
            return;
         }

         norm = 1.0 / norm;
         qx *= norm;
         qy *= norm;
         qz *= norm;
         qs *= norm;

         double yy2 = 2.0 * qy * qy;
         double zz2 = 2.0 * qz * qz;
         double xx2 = 2.0 * qx * qx;
         double xy2 = 2.0 * qx * qy;
         double sz2 = 2.0 * qs * qz;
         double xz2 = 2.0 * qx * qz;
         double sy2 = 2.0 * qs * qy;
         double yz2 = 2.0 * qy * qz;
         double sx2 = 2.0 * qs * qx;

         double qM00 = 1.0 - yy2 - zz2;
         double qM01 = xy2 - sz2;
         double qM02 = xz2 + sy2;
         double qM10 = xy2 + sz2;
         double qM11 = 1.0 - xx2 - zz2;
         double qM12 = yz2 - sx2;
         double qM20 = xz2 - sy2;
         double qM21 = yz2 + sx2;
         double qM22 = 1.0 - xx2 - yy2;

         double m00, m01, m02, m10, m11, m12, m20, m21, m22;

         if (transposeMatrix)
         {
            m00 = qM00 * matrix.getM00() + qM01 * matrix.getM01() + qM02 * matrix.getM02();
            m01 = qM00 * matrix.getM10() + qM01 * matrix.getM11() + qM02 * matrix.getM12();
            m02 = qM00 * matrix.getM20() + qM01 * matrix.getM21() + qM02 * matrix.getM22();
            m10 = qM10 * matrix.getM00() + qM11 * matrix.getM01() + qM12 * matrix.getM02();
            m11 = qM10 * matrix.getM10() + qM11 * matrix.getM11() + qM12 * matrix.getM12();
            m12 = qM10 * matrix.getM20() + qM11 * matrix.getM21() + qM12 * matrix.getM22();
            m20 = qM20 * matrix.getM00() + qM21 * matrix.getM01() + qM22 * matrix.getM02();
            m21 = qM20 * matrix.getM10() + qM21 * matrix.getM11() + qM22 * matrix.getM12();
            m22 = qM20 * matrix.getM20() + qM21 * matrix.getM21() + qM22 * matrix.getM22();
         }
         else
         {
            m00 = qM00 * matrix.getM00() + qM01 * matrix.getM10() + qM02 * matrix.getM20();
            m01 = qM00 * matrix.getM01() + qM01 * matrix.getM11() + qM02 * matrix.getM21();
            m02 = qM00 * matrix.getM02() + qM01 * matrix.getM12() + qM02 * matrix.getM22();
            m10 = qM10 * matrix.getM00() + qM11 * matrix.getM10() + qM12 * matrix.getM20();
            m11 = qM10 * matrix.getM01() + qM11 * matrix.getM11() + qM12 * matrix.getM21();
            m12 = qM10 * matrix.getM02() + qM11 * matrix.getM12() + qM12 * matrix.getM22();
            m20 = qM20 * matrix.getM00() + qM21 * matrix.getM10() + qM22 * matrix.getM20();
            m21 = qM20 * matrix.getM01() + qM21 * matrix.getM11() + qM22 * matrix.getM21();
            m22 = qM20 * matrix.getM02() + qM21 * matrix.getM12() + qM22 * matrix.getM22();
         }
         matrixToPack.setAndNormalize(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      }
   }

   public static void multiply(RotationMatrixReadOnly<?> matrix, QuaternionReadOnly<?> quaternion, RotationMatrix matrixToPack)
   {
      multiplyImpl(matrix, false, quaternion, false, matrixToPack);
   }

   public static void multiplyConjugateQuaternion(RotationMatrixReadOnly<?> matrix, QuaternionReadOnly<?> quaternion, RotationMatrix matrixToPack)
   {
      multiplyImpl(matrix, false, quaternion, true, matrixToPack);
   }

   public static void multiplyTransposeMatrix(RotationMatrixReadOnly<?> matrix, QuaternionReadOnly<?> quaternion, RotationMatrix matrixToPack)
   {
      multiplyImpl(matrix, true, quaternion, false, matrixToPack);
   }

   public static void multiplyTransposeMatrixConjugateQuaternion(RotationMatrixReadOnly<?> matrix, QuaternionReadOnly<?> quaternion, RotationMatrix matrixToPack)
   {
      multiplyImpl(matrix, true, quaternion, true, matrixToPack);
   }

   private static void multiplyImpl(RotationMatrixReadOnly<?> matrix, boolean transposeMatrix, QuaternionReadOnly<?> quaternion, boolean conjugateQuaternion,
                                    RotationMatrix matrixToPack)
   {
      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      if (conjugateQuaternion)
      {
         qx = -qx;
         qy = -qy;
         qz = -qz;
      }

      if (matrix != matrixToPack)
      {
         RotationMatrixConversion.convertQuaternionToMatrixImpl(qx, qy, qz, qs, matrixToPack);
         matrixToPack.preMultiply(matrix);
      }
      else
      {
         double norm = norm(qx, qy, qz, qs);

         if (norm < EPS)
         {
            matrixToPack.set(matrix);
            return;
         }

         norm = 1.0 / norm;
         qx *= norm;
         qy *= norm;
         qz *= norm;
         qs *= norm;

         double yy2 = 2.0 * qy * qy;
         double zz2 = 2.0 * qz * qz;
         double xx2 = 2.0 * qx * qx;
         double xy2 = 2.0 * qx * qy;
         double sz2 = 2.0 * qs * qz;
         double xz2 = 2.0 * qx * qz;
         double sy2 = 2.0 * qs * qy;
         double yz2 = 2.0 * qy * qz;
         double sx2 = 2.0 * qs * qx;

         double qM00 = 1.0 - yy2 - zz2;
         double qM01 = xy2 - sz2;
         double qM02 = xz2 + sy2;
         double qM10 = xy2 + sz2;
         double qM11 = 1.0 - xx2 - zz2;
         double qM12 = yz2 - sx2;
         double qM20 = xz2 - sy2;
         double qM21 = yz2 + sx2;
         double qM22 = 1.0 - xx2 - yy2;

         double m00, m01, m02, m10, m11, m12, m20, m21, m22;
         if (transposeMatrix)
         {
            m00 = matrix.getM00() * qM00 + matrix.getM10() * qM10 + matrix.getM20() * qM20;
            m01 = matrix.getM00() * qM01 + matrix.getM10() * qM11 + matrix.getM20() * qM21;
            m02 = matrix.getM00() * qM02 + matrix.getM10() * qM12 + matrix.getM20() * qM22;
            m10 = matrix.getM01() * qM00 + matrix.getM11() * qM10 + matrix.getM21() * qM20;
            m11 = matrix.getM01() * qM01 + matrix.getM11() * qM11 + matrix.getM21() * qM21;
            m12 = matrix.getM01() * qM02 + matrix.getM11() * qM12 + matrix.getM21() * qM22;
            m20 = matrix.getM02() * qM00 + matrix.getM12() * qM10 + matrix.getM22() * qM20;
            m21 = matrix.getM02() * qM01 + matrix.getM12() * qM11 + matrix.getM22() * qM21;
            m22 = matrix.getM02() * qM02 + matrix.getM12() * qM12 + matrix.getM22() * qM22;
         }
         else
         {
            m00 = matrix.getM00() * qM00 + matrix.getM01() * qM10 + matrix.getM02() * qM20;
            m01 = matrix.getM00() * qM01 + matrix.getM01() * qM11 + matrix.getM02() * qM21;
            m02 = matrix.getM00() * qM02 + matrix.getM01() * qM12 + matrix.getM02() * qM22;
            m10 = matrix.getM10() * qM00 + matrix.getM11() * qM10 + matrix.getM12() * qM20;
            m11 = matrix.getM10() * qM01 + matrix.getM11() * qM11 + matrix.getM12() * qM21;
            m12 = matrix.getM10() * qM02 + matrix.getM11() * qM12 + matrix.getM12() * qM22;
            m20 = matrix.getM20() * qM00 + matrix.getM21() * qM10 + matrix.getM22() * qM20;
            m21 = matrix.getM20() * qM01 + matrix.getM21() * qM11 + matrix.getM22() * qM21;
            m22 = matrix.getM20() * qM02 + matrix.getM21() * qM12 + matrix.getM22() * qM22;
         }
         matrixToPack.setAndNormalize(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      }
   }
}
