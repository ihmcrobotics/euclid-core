package us.ihmc.geometry.testingTools;

import java.util.Random;

import us.ihmc.geometry.axisAngle.AxisAngle;
import us.ihmc.geometry.axisAngle.AxisAngle32;
import us.ihmc.geometry.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.geometry.matrix.Matrix3D;
import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.matrix.RotationScaleMatrix;
import us.ihmc.geometry.transform.AffineTransform;
import us.ihmc.geometry.transform.QuaternionBasedTransform;
import us.ihmc.geometry.transform.RigidBodyTransform;
import us.ihmc.geometry.tuple2D.Point2D;
import us.ihmc.geometry.tuple2D.Point2D32;
import us.ihmc.geometry.tuple2D.Vector2D;
import us.ihmc.geometry.tuple2D.Vector2D32;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple3D.Point3D;
import us.ihmc.geometry.tuple3D.Point3D32;
import us.ihmc.geometry.tuple3D.Vector3D;
import us.ihmc.geometry.tuple3D.Vector3D32;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.geometry.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.tuple4D.Quaternion;
import us.ihmc.geometry.tuple4D.Quaternion32;
import us.ihmc.geometry.tuple4D.Vector4D;
import us.ihmc.geometry.tuple4D.Vector4D32;
import us.ihmc.geometry.yawPitchRoll.YawPitchRollConversion;

/**
 * This class provides random generators to generate random geometry objects.
 * <p>
 * The main application is for writing JUnit Tests.
 * </p>
 *
 * @author Sylvain Bertrand
 *
 */
public abstract class EuclidCoreRandomTools
{
   /**
    * Generates random yaw-pitch-roll angles and returns it in an array.
    * <p>
    * <ul>
    * <li>yaw &in; [-<i>pi</i>; <i>pi</i>],
    * <li>pitch &in; [-<i>pi</i>/2.0; <i>pi</i>/2.0],
    * <li>roll &in; [-<i>pi</i>; <i>pi</i>],
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return an array containing the random yaw-pitch-roll angles.
    */
   public static double[] generateRandomYawPitchRoll(Random random)
   {
      return generateRandomYawPitchRoll(random, Math.PI, YawPitchRollConversion.MAX_PITCH_ANGLE, Math.PI);
   }

   /**
    * Generates random yaw-pitch-roll angles and returns it in an array.
    * <p>
    * <ul>
    * <li>yaw &in; [-{@code minMaxYaw}; {@code minMaxYaw}],
    * <li>pitch &in; [-{@code minMaxPitch}; {@code minMaxPitch}],
    * <li>roll &in; [-{@code minMaxRoll}; {@code minMaxRoll}],
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @param minMaxYaw the maximum absolute angle for the generated yaw angle.
    * @param minMaxPitch the maximum absolute angle for the generated pitch angle.
    * @param minMaxRoll the maximum absolute angle for the generated roll angle.
    * @return an array containing the random yaw-pitch-roll angles.
    * @throws RuntimeException if {@code minMaxYaw < 0}, {@code minMaxPitch < 0},
    *            {@code minMaxRoll < 0}.
    */
   public static double[] generateRandomYawPitchRoll(Random random, double minMaxYaw, double minMaxPitch, double minMaxRoll)
   {
      double yaw = generateRandomDouble(random, minMaxYaw);
      double pitch = generateRandomDouble(random, minMaxPitch);
      double roll = generateRandomDouble(random, minMaxRoll);
      double[] yawPitchRoll = {yaw, pitch, roll};
      return yawPitchRoll;
   }

   /**
    * Generates a random rotation vector.
    * <p>
    * {@code rotationVector.length()} &in; [-<i>pi</i>; <i>pi</i>].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random rotation vector.
    */
   public static Vector3D generateRandomRotationVector(Random random)
   {
      return generateRandomRotationVector(random, Math.PI);
   }

   /**
    * Generates a random rotation vector.
    * <p>
    * {@code rotationVector.length()} &in; [-{@code minMaxAngle}; {@code minMaxAngle}].
    * </p>
    *
    * @param random the random generator to use.
    * @param minMaxAngle the maximum length of the generated rotation vector.
    * @return the random rotation vector.
    * @throws RuntimeException if {@code minMaxAngle < 0}.
    */
   public static Vector3D generateRandomRotationVector(Random random, double minMaxAngle)
   {
      Vector3D rotationVector = new Vector3D();
      AxisAngle axisAngle = generateRandomAxisAngle(random, minMaxAngle);
      axisAngle.getRotationVector(rotationVector);
      return rotationVector;
   }

   /**
    * Generates a random axis-angle.
    * <p>
    * {@code axisAngle.getAngle()} &in; [-<i>pi</i>; <i>pi</i>].
    * </p>
    * <p>
    * The approach used here generates uniformly a point on a sphere to create uniformly distributed
    * random axes, <a href="http://mathworld.wolfram.com/SpherePointPicking.html"> see link</a>. The
    * angle is then generated as a normal bounded random double.
    * </p>
    *
    * @param random the random generator to use.
    * @return the random axis-angle.
    */
   public static AxisAngle generateRandomAxisAngle(Random random)
   {
      AxisAngle axisAngle = new AxisAngle();
      randomizeAxisAngle(random, axisAngle);
      return axisAngle;
   }

   /**
    * Generates a random axis-angle.
    * <p>
    * {@code axisAngle.getAngle()} &in; [-{@code minMaxAngle}; {@code minMaxAngle}].
    * </p>
    * <p>
    * The approach used here generates uniformly a point on a sphere to create uniformly distributed
    * random axes, <a href="http://mathworld.wolfram.com/SpherePointPicking.html"> see link</a>. The
    * angle is then generated as a normal bounded random double.
    * </p>
    *
    * @param random the random generator to use.
    * @param minMaxAngle the maximum absolute angle value.
    * @return the random axis-angle.
    * @throws RuntimeException if {@code minMaxAngle < 0}.
    */
   public static AxisAngle generateRandomAxisAngle(Random random, double minMaxAngle)
   {
      AxisAngle axisAngle = new AxisAngle();
      randomizeAxisAngle(random, minMaxAngle, axisAngle);
      return axisAngle;
   }

   /**
    * Generates a random axis-angle.
    * <p>
    * {@code axisAngle.getAngle()} &in; [-<i>pi</i>; <i>pi</i>].
    * </p>
    * <p>
    * The approach used here generates uniformly a point on a sphere to create uniformly distributed
    * random axes, <a href="http://mathworld.wolfram.com/SpherePointPicking.html"> see link</a>. The
    * angle is then generated as a normal bounded random double.
    * </p>
    *
    * @param random the random generator to use.
    * @return the random axis-angle.
    */
   public static AxisAngle32 generateRandomAxisAngle32(Random random)
   {
      AxisAngle32 axisAngle = new AxisAngle32();
      randomizeAxisAngle(random, axisAngle);
      return axisAngle;
   }

   /**
    * Generates a random axis-angle.
    * <p>
    * {@code axisAngle.getAngle()} &in; [-{@code minMaxAngle}; {@code minMaxAngle}].
    * </p>
    * <p>
    * The approach used here generates uniformly a point on a sphere to create uniformly distributed
    * random axes, <a href="http://mathworld.wolfram.com/SpherePointPicking.html"> see link</a>. The
    * angle is then generated as a normal bounded random double.
    * </p>
    *
    * @param random the random generator to use.
    * @param minMaxAngle the maximum absolute angle value.
    * @return the random axis-angle.
    * @throws RuntimeException if {@code minMaxAngle < 0}.
    */
   public static AxisAngle32 generateRandomAxisAngle32(Random random, double minMaxAngle)
   {
      AxisAngle32 axisAngle = new AxisAngle32();
      randomizeAxisAngle(random, minMaxAngle, axisAngle);
      return axisAngle;
   }

   /**
    * Generates a random diagonal 3-by-3 matrix.
    * <p>
    * <ul>
    * <li>{@code matrix.getM00()} &in; [-1.0; 1.0].
    * <li>{@code matrix.getM11()} &in; [-1.0; 1.0].
    * <li>{@code matrix.getM22()} &in; [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random diagonal matrix.
    */
   public static Matrix3D generateRandomDiagonalMatrix3D(Random random)
   {
      return generateRandomMatrix3D(random, 1.0);
   }

   /**
    * Generates a random diagonal 3-by-3 matrix.
    * <p>
    * <ul>
    * <li>{@code matrix.getM00()} &in; [-{@code minMaxValue}; {@code minMaxValue}].
    * <li>{@code matrix.getM11()} &in; [-{@code minMaxValue}; {@code minMaxValue}].
    * <li>{@code matrix.getM22()} &in; [-{@code minMaxValue}; {@code minMaxValue}].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @param minMaxValue the maximum absolute value for each diagonal element.
    * @return the random diagonal matrix.
    * @throws RuntimeException if {@code minMaxValue < 0}.
    */
   public static Matrix3D generateRandomDiagonalMatrix3D(Random random, double minMaxValue)
   {
      return generateRandomMatrix3D(random, -minMaxValue, minMaxValue);
   }

   /**
    * Generates a random diagonal 3-by-3 matrix.
    * <p>
    * <ul>
    * <li>{@code matrix.getM00()} &in; [{@code minValue}; {@code maxValue}].
    * <li>{@code matrix.getM11()} &in; [{@code minValue}; {@code maxValue}].
    * <li>{@code matrix.getM22()} &in; [{@code minValue}; {@code maxValue}].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @param minValue the minimum value of each diagonal element.
    * @param maxValue the maximum value of each diagonal element.
    * @return the random diagonal matrix.
    * @throws RuntimeException if {@code minValue > maxValue}.
    */
   public static Matrix3D generateRandomDiagonalMatrix3D(Random random, double minValue, double maxValue)
   {
      Matrix3D matrix3D = new Matrix3D();
      for (int i = 0; i < 3; i++)
         matrix3D.setElement(i, i, generateRandomDouble(random, minValue, maxValue));
      return matrix3D;
   }

   /**
    * Generates a random double &in; [-1.0; 1.0].
    *
    * @param random the random generator to use.
    * @return the random double.
    */
   public static double generateRandomDouble(Random random)
   {
      return generateRandomDouble(random, 1.0);
   }

   /**
    * Generates a random double &in; [-{@code minMax}; {@code minMax}].
    *
    * @param random the random generator to use.
    * @param minMaxValue the maximum absolute value of the generated double.
    * @return the random double.
    * @throws RuntimeException if {@code minMaxValue < 0}.
    */
   public static double generateRandomDouble(Random random, double minMaxValue)
   {
      return generateRandomDouble(random, -minMaxValue, minMaxValue);
   }

   /**
    * Generates a random double &in; [-{@code minValue}; {@code maxValue}].
    *
    * @param random the random generator to use.
    * @param minValue the minimum value of the generated double.
    * @param maxValue the maximum value of the generated double.
    * @return the random double.
    * @throws RuntimeException if {@code minValue > maxValue}.
    */
   public static double generateRandomDouble(Random random, double minValue, double maxValue)
   {
      if (minValue > maxValue)
         throw new RuntimeException("Min is greater than max: min = " + minValue + ", max = " + maxValue);

      return minValue + random.nextDouble() * (maxValue - minValue);
   }

   /**
    * Generates a random 3-by-3 matrix.
    * <p>
    * {@code matrix}<sub>ij</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random matrix.
    */
   public static Matrix3D generateRandomMatrix3D(Random random)
   {
      return generateRandomMatrix3D(random, 1.0);
   }

   /**
    * Generates a random 3-by-3 matrix.
    * <p>
    * {@code matrix}<sub>ij</sub> &in; [-{@code minMaxValue}; {@code minMaxValue}].
    * </p>
    *
    * @param random the random generator to use.
    * @param minMaxValue the maximum absolute value for each element.
    * @return the random matrix.
    * @throws RuntimeException if {@code minMaxValue < 0}.
    */
   public static Matrix3D generateRandomMatrix3D(Random random, double minMaxValue)
   {
      return generateRandomMatrix3D(random, -minMaxValue, minMaxValue);
   }

   /**
    * Generates a random 3-by-3 matrix.
    * <p>
    * {@code matrix}<sub>ij</sub> &in; [{@code minValue}; {@code maxValue}].
    * </p>
    *
    * @param random the random generator to use.
    * @param minValue the minimum value for each element.
    * @param maxValue the maximum value for each element.
    * @return the random matrix.
    * @throws RuntimeException if {@code minValue > maxValue}.
    */
   public static Matrix3D generateRandomMatrix3D(Random random, double minValue, double maxValue)
   {
      Matrix3D matrix3D = new Matrix3D();
      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            matrix3D.setElement(row, column, generateRandomDouble(random, minValue, maxValue));
         }
      }
      return matrix3D;
   }

   /**
    * Generates a random quaternion uniformly distributed on the unit-sphere.
    * <p>
    * The rotation magnitude described by the generated quaternion is in [-<i>pi</i>; <i>pi</i>].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random quaternion.
    */
   public static Quaternion generateRandomQuaternion(Random random)
   {
      return new Quaternion(generateRandomAxisAngle(random));
   }

   /**
    * Generates a random quaternion uniformly distributed on the unit-sphere.
    * <p>
    * The rotation magnitude described by the generated quaternion is in [-{@code minMaxAngle};
    * {@code minMaxAngle}].
    * </p>
    *
    * @param random the random generator to use.
    * @param minMaxAngle the maximum absolute angle described by the generated quaternion.
    * @return the random quaternion.
    * @throws RuntimeException if {@code minMaxAngle < 0}.
    */
   public static Quaternion generateRandomQuaternion(Random random, double minMaxAngle)
   {
      return new Quaternion(generateRandomAxisAngle(random, minMaxAngle));
   }

   /**
    * Generates a random quaternion uniformly distributed on the unit-sphere.
    * <p>
    * The rotation magnitude described by the generated quaternion is in [-<i>pi</i>; <i>pi</i>].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random quaternion.
    */
   public static Quaternion32 generateRandomQuaternion32(Random random)
   {
      return new Quaternion32(generateRandomAxisAngle(random));
   }

   /**
    * Generates a random quaternion uniformly distributed on the unit-sphere.
    * <p>
    * The rotation magnitude described by the generated quaternion is in [-{@code minMaxAngle};
    * {@code minMaxAngle}].
    * </p>
    *
    * @param random the random generator to use.
    * @param minMaxAngle the maximum absolute angle described by the generated quaternion.
    * @return the random quaternion.
    * @throws RuntimeException if {@code minMaxAngle < 0}.
    */
   public static Quaternion32 generateRandomQuaternion32(Random random, double minMaxAngleRange)
   {
      return new Quaternion32(generateRandomAxisAngle(random, minMaxAngleRange));
   }

   /**
    * Generates a random rigid-body transform.
    * <p>
    * <ul>
    * <li>The rotation part is uniformly distributed on the unit sphere and describes an rotation
    * angle in [-<i>pi</i>; <i>pi</i>].
    * <li>Each component of the translation part is in [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random rigid-body transform.
    */
   public static RigidBodyTransform generateRandomRigidBodyTransform(Random random)
   {
      return new RigidBodyTransform(generateRandomAxisAngle(random), generateRandomVector3D(random));
   }

   /**
    * Generates a random quaternion-based transform.
    * <p>
    * <ul>
    * <li>The rotation part is uniformly distributed on the unit sphere and describes an rotation
    * angle in [-<i>pi</i>; <i>pi</i>].
    * <li>Each component of the translation part is in [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random quaternion-based transform.
    */
   public static QuaternionBasedTransform generateRandomQuaternionBasedTransform(Random random)
   {
      return new QuaternionBasedTransform(generateRandomQuaternion(random), generateRandomVector3D(random));
   }

   /**
    * Generates a random affine transform.
    * <p>
    * <ul>
    * <li>The rotation part is uniformly distributed on the unit sphere and describes an rotation
    * angle in [-<i>pi</i>; <i>pi</i>].
    * <li>Each scale factor is in ]0.0; 10.0].
    * <li>Each component of the translation part is in [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @return the random affine transform.
    */
   public static AffineTransform generateRandomAffineTransform(Random random)
   {
      return new AffineTransform(generateRandomRotationScaleMatrix(random, 10.0), generateRandomVector3D(random));
   }

   /**
    * Generates a random rotation matrix uniformly distributed on the unit sphere and describes an
    * rotation angle in [-<i>pi</i>; <i>pi</i>].
    *
    * @param random the random generator to use.
    * @return the random rotation matrix.
    */
   public static RotationMatrix generateRandomRotationMatrix(Random random)
   {
      return generateRandomRotationMatrix(random, Math.PI);
   }

   /**
    * Generates a random rotation matrix uniformly distributed on the unit sphere and describes an
    * rotation angle in [-{@code minMaxAngle}; {@code minMaxAngle}].
    *
    * @param random the random generator to use.
    * @param minMaxAngle the maximum absolute angle described by the generated rotation matrix.
    * @return the random rotation matrix.
    */
   public static RotationMatrix generateRandomRotationMatrix(Random random, double minMaxAngle)
   {
      AxisAngle randomRotation = generateRandomAxisAngle(random, minMaxAngle);
      return new RotationMatrix(randomRotation);
   }

   /**
    * Generates a random rotation-scale matrix.
    * <p>
    * <ul>
    * <li>The rotation part is uniformly distributed on the unit sphere and describes an rotation
    * angle in [-<i>pi</i>; <i>pi</i>].
    * <li>Each scale factor is in ]0.0; {@code maxScale}].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @param maxScale the maximum scale value used for each scale factor.
    * @return the random rotation-scale matrix.
    * @throws RuntimeException if {@code maxScale < 0}.
    */
   public static RotationScaleMatrix generateRandomRotationScaleMatrix(Random random, double maxScale)
   {
      return generateRandomRotationScaleMatrix(random, Math.PI, maxScale);
   }

   /**
    * Generates a random rotation-scale matrix.
    * <p>
    * <ul>
    * <li>The rotation part is uniformly distributed on the unit sphere and describes an rotation
    * angle in [-{@code minMaxAngle}; {@code minMaxAngle}].
    * <li>Each scale factor is in ]0.0; {@code maxScale}].
    * </ul>
    * </p>
    *
    * @param random the random generator to use.
    * @param minMaxAngle the maximum absolute angle value that describes the generated
    *           rotation-scale matrix.
    * @param maxScale the maximum scale value used for each scale factor.
    * @return the random rotation-scale matrix.
    * @throws RuntimeException if {@code minMaxAngle < 0}.
    * @throws RuntimeException if {@code maxScale < 0}.
    */
   public static RotationScaleMatrix generateRandomRotationScaleMatrix(Random random, double minMaxAngle, double maxScale)
   {
      AxisAngle randomRotation = generateRandomAxisAngle(random, minMaxAngle);
      Vector3D randomScales = generateRandomVector3D(random, new Vector3D(0.0, 0.0, 0.0), new Vector3D(maxScale, maxScale, maxScale));
      return new RotationScaleMatrix(randomRotation, randomScales);
   }

   /**
    * Generates a random vector.
    * <p>
    * {@code vector}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random vector.
    */
   public static Vector3D generateRandomVector3D(Random random)
   {
      Vector3D vector = new Vector3D();
      randomizeTuple3D(random, vector);
      return vector;
   }

   /**
    * Generates a random point.
    * <p>
    * {@code point}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random point.
    */
   public static Point3D generateRandomPoint3D(Random random)
   {
      Point3D point = new Point3D();
      randomizeTuple3D(random, point);
      return point;
   }

   /**
    * Generates a random point.
    * <p>
    * {@code point.x} &in; [-maxAbsoluteX; maxAbsoluteX]. <br>
    * {@code point.y} &in; [-maxAbsoluteY; maxAbsoluteY]. <br>
    * {@code point.z} &in; [-maxAbsoluteZ; maxAbsoluteZ]. <br>
    * </p>
    * 
    * @param random the random generator to use.
    * @param maxAbsoluteX the maximum absolute value for the x-coordinate.
    * @param maxAbsoluteY the maximum absolute value for the y-coordinate.
    * @param maxAbsoluteZ the maximum absolute value for the z-coordinate.
    * @return the random point.
    * @throws RuntimeException if {@code maxAbsoluteX < 0}, {@code maxAbsoluteY < 0},
    *            {@code maxAbsoluteZ < 0}.
    */
   public static Point3D generateRandomPoint3D(Random random, double maxAbsoluteX, double maxAbsoluteY, double maxAbsoluteZ)
   {
      double x = generateRandomDouble(random, -maxAbsoluteX, maxAbsoluteX);
      double y = generateRandomDouble(random, -maxAbsoluteY, maxAbsoluteY);
      double z = generateRandomDouble(random, -maxAbsoluteZ, maxAbsoluteZ);

      return new Point3D(x, y, z);
   }

   /**
    * Generates a random vector given its length {@code length}.
    *
    * @param random the random generator to use.
    * @param length the length of the generated vector.
    * @return the random vector.
    */
   public static Vector3D generateRandomVector3DWithFixedLength(Random random, double length)
   {
      Vector3D vector = generateRandomVector3D(random);
      vector.normalize();
      vector.scale(length);
      return vector;
   }

   /**
    * Generates a random vector that is perpendicular to {@code vectorToBeOrthogonalTo}.
    *
    * @param random the random generator to use.
    * @param vectorToBeOrthogonalTo the vector to be orthogonal to. Not modified.
    * @param normalize whether to normalize the generated vector or not.
    * @return the random vector.
    */
   public static Vector3D generateRandomOrthogonalVector3d(Random random, Vector3DReadOnly vectorToBeOrthogonalTo, boolean normalize)
   {
      Vector3D v1 = new Vector3D(vectorToBeOrthogonalTo.getY(), -vectorToBeOrthogonalTo.getX(), 0.0);
      Vector3D v2 = new Vector3D(-vectorToBeOrthogonalTo.getZ(), 0.0, vectorToBeOrthogonalTo.getX());

      Vector3D randomPerpendicular = new Vector3D();
      double a = generateRandomDouble(random, 1.0);
      double b = generateRandomDouble(random, 1.0);
      randomPerpendicular.scaleAdd(a, v1, randomPerpendicular);
      randomPerpendicular.scaleAdd(b, v2, randomPerpendicular);

      if (normalize)
         randomPerpendicular.normalize();

      return randomPerpendicular;
   }

   /**
    * Generates a random vector.
    * <p>
    * {@code vector}<sub>i</sub> &in; [-{@code minMax}<sub>i</sub>; {@code minMax}<sub>i</sub>].
    * </p>
    *
    * @param random the random generator to use.
    * @param minMax tuple used to bound the maximum absolute value of each component of the
    *           generated vector. Not modified.
    * @return the random vector.
    * @throws RuntimeException if any component of {@code minMax} is negative.
    */
   public static Vector3D generateRandomVector3D(Random random, Tuple3DReadOnly minMax)
   {
      Vector3D vector = new Vector3D();
      randomizeTuple3D(random, minMax, vector);
      return vector;
   }

   /**
    * Generates a random vector.
    * <p>
    * {@code vector}<sub>i</sub> &in; [{@code min}<sub>i</sub>; {@code max}<sub>i</sub>].
    * </p>
    *
    * @param random the random generator to use.
    * @param min tuple used as upper-bound for each component of the generated vector. Not modified.
    * @param max tuple used as lower-bound for each component of the generated vector. Not modified.
    * @return the random vector.
    * @throws RuntimeException if {@code min}<sub>i</sub> > {@code max}<sub>i</sub>.
    */
   public static Vector3D generateRandomVector3D(Random random, Tuple3DReadOnly min, Tuple3DReadOnly max)
   {
      Vector3D vector = new Vector3D();
      randomizeTuple3D(random, min, max, vector);
      return vector;
   }

   /**
    * Generates a random vector.
    * <p>
    * {@code vector}<sub>i</sub> &in; [{@code min}; {@code max}].
    * </p>
    *
    * @param random the random generator to use.
    * @param min upper-bound for each component of the generated vector. Not modified.
    * @param max lower-bound for each component of the generated vector. Not modified.
    * @return the random vector.
    * @throws RuntimeException if {@code min > max}.
    */
   public static Vector3D generateRandomVector3D(Random random, double min, double max)
   {
      Vector3D vector = new Vector3D();
      randomizeTuple3D(random, new Point3D(min, min, min), new Point3D(max, max, max), vector);
      return vector;
   }

   /**
    * Generates a random 2D vector.
    * <p>
    * {@code vector}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random 2D vector.
    */
   public static Vector2D generateRandomVector2D(Random random)
   {
      Vector2D vector = new Vector2D();
      randomizeTuple2D(random, vector);
      return vector;
   }

   /**
    * Generates a random 2D point.
    * <p>
    * {@code point}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random 2D point.
    */
   public static Point2D generateRandomPoint2D(Random random)
   {
      Point2D point = new Point2D();
      randomizeTuple2D(random, point);
      return point;
   }

   /**
    * Generates a random 2D vector given its length {@code length}.
    *
    * @param random the random generator to use.
    * @param length the length of the generated 2D vector.
    * @return the random 2D vector.
    */
   public static Vector2D generateRandomVector2DWithFixedLength(Random random, double length)
   {
      Vector2D vector = generateRandomVector2D(random);
      vector.normalize();
      vector.scale(length);
      return vector;
   }

   /**
    * Generates a random 2D vector.
    * <p>
    * {@code vector}<sub>i</sub> &in; [-{@code minMax}<sub>i</sub>; {@code minMax}<sub>i</sub>].
    * </p>
    *
    * @param random the random generator to use.
    * @param minMax tuple used to bound the maximum absolute value of each component of the
    *           generated 2D vector. Not modified.
    * @return the random 2D vector.
    * @throws RuntimeException if any component of {@code minMax} is negative.
    */
   public static Vector2D generateRandomVector2D(Random random, Tuple2DReadOnly minMax)
   {
      Vector2D vector = new Vector2D();
      randomizeTuple2D(random, minMax, vector);
      return vector;
   }

   /**
    * Generates a random 2D vector.
    * <p>
    * {@code vector}<sub>i</sub> &in; [{@code min}<sub>i</sub>; {@code max}<sub>i</sub>].
    * </p>
    *
    * @param random the random generator to use.
    * @param min tuple used as upper-bound for each component of the generated 2D vector. Not
    *           modified.
    * @param max tuple used as lower-bound for each component of the generated 2D vector. Not
    *           modified.
    * @return the random 2D vector.
    * @throws RuntimeException if {@code min}<sub>i</sub> > {@code max}<sub>i</sub>.
    */
   public static Vector2D generateRandomVector2D(Random random, Tuple2DReadOnly min, Tuple2DReadOnly max)
   {
      Vector2D vector = new Vector2D();
      randomizeTuple2D(random, min, max, vector);
      return vector;
   }

   /**
    * Generates a random 4D vector.
    * <p>
    * {@code vector}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random 4D vector.
    */
   public static Vector4D generateRandomVector4D(Random random)
   {
      Vector4D vector = new Vector4D();
      for (int i = 0; i < 4; i++)
         vector.set(i, generateRandomDouble(random));
      return vector;
   }

   /**
    * Generates a random vector.
    * <p>
    * {@code vector}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random vector.
    */
   public static Vector3D32 generateRandomVector3D32(Random random)
   {
      Vector3D32 vector = new Vector3D32();
      randomizeTuple3D(random, vector);
      return vector;
   }

   /**
    * Generates a random point.
    * <p>
    * {@code point}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random point.
    */
   public static Point3D32 generateRandomPoint3D32(Random random)
   {
      Point3D32 point = new Point3D32();
      randomizeTuple3D(random, point);
      return point;
   }

   /**
    * Generates a random 2D vector.
    * <p>
    * {@code vector}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random 2D vector.
    */
   public static Vector2D32 generateRandomVector2D32(Random random)
   {
      Vector2D32 vector = new Vector2D32();
      randomizeTuple2D(random, vector);
      return vector;
   }

   /**
    * Generates a random 2D point.
    * <p>
    * {@code point}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random 2D point.
    */
   public static Point2D32 generateRandomPoint2D32(Random random)
   {
      Point2D32 point = new Point2D32();
      randomizeTuple2D(random, point);
      return point;
   }

   /**
    * Generates a random 4D vector.
    * <p>
    * {@code vector}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @return the random 4D vector.
    */
   public static Vector4D32 generateRandomVector4D32(Random random)
   {
      Vector4D32 vector = new Vector4D32();
      for (int i = 0; i < 4; i++)
         vector.set(i, generateRandomDouble(random));
      return vector;
   }

   /**
    * Randomizes the given axis-angle.
    * <p>
    * {@code axisAngle.getAngle()} &in; [-<i>pi</i>; <i>pi</i>].
    * </p>
    * <p>
    * The approach used here generates uniformly a point on a sphere to create uniformly distributed
    * random axes, <a href="http://mathworld.wolfram.com/SpherePointPicking.html"> see link</a>. The
    * angle is then generated as a normal bounded random double.
    * </p>
    *
    * @param random the random generator to use.
    * @param axisAngleToRandomize the axis-angle to randomize. Modified.
    */
   public static void randomizeAxisAngle(Random random, AxisAngleBasics axisAngleToRandomize)
   {
      randomizeAxisAngle(random, Math.PI, axisAngleToRandomize);
   }

   /**
    * Randomizes the given axis-angle.
    * <p>
    * {@code axisAngle.getAngle()} &in; [-{@code minMaxAngle}; {@code minMaxAngle}].
    * </p>
    * <p>
    * The approach used here generates uniformly a point on a sphere to create uniformly distributed
    * random axes, <a href="http://mathworld.wolfram.com/SpherePointPicking.html"> see link</a>. The
    * angle is then generated as a normal bounded random double.
    * </p>
    *
    * @param random the random generator to use.
    * @param minMaxAngle the maximum absolute angle value.
    * @param axisAngleToRandomize the axis-angle to randomize. Modified.
    * @throws RuntimeException if {@code minMaxAngle < 0}.
    */
   public static void randomizeAxisAngle(Random random, double minMaxAngle, AxisAngleBasics axisAngleToRandomize)
   {
      // Generate uniformly random point on unit sphere (based on http://mathworld.wolfram.com/SpherePointPicking.html )
      double height = 2.0 * random.nextDouble() - 1.0;
      double angle = generateRandomDouble(random, minMaxAngle);
      double radius = Math.sqrt(1.0 - height * height);
      axisAngleToRandomize.set(radius * Math.cos(angle), radius * Math.sin(angle), height, angle);
   }

   /**
    * Randomizes a tuple.
    * <p>
    * {@code tuple}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @param tupleToRandomize the tuple to randomize. Modified.
    */
   public static void randomizeTuple3D(Random random, Tuple3DBasics tupleToRandomize)
   {
      randomizeTuple3D(random, new Point3D(1.0, 1.0, 1.0), tupleToRandomize);
   }

   /**
    * Randomizes a tuple.
    * <p>
    * {@code tuple}<sub>i</sub> &in; [-{@code minMax}<sub>i</sub>; {@code minMax}<sub>i</sub>].
    * </p>
    *
    * @param random the random generator to use.
    * @param minMax tuple used to bound the maximum absolute value of each component of the
    *           generated vector. Not modified.
    * @param tupleToRandomize the tuple to randomize. Modified.
    * @throws RuntimeException if any component of {@code minMax} is negative.
    */
   public static void randomizeTuple3D(Random random, Tuple3DReadOnly minMax, Tuple3DBasics tupleToRandomize)
   {
      for (int i = 0; i < 3; i++)
         tupleToRandomize.set(i, generateRandomDouble(random, minMax.get(i)));
   }

   /**
    * Randomizes a tuple.
    * <p>
    * {@code vector}<sub>i</sub> &in; [{@code min}<sub>i</sub>; {@code max}<sub>i</sub>].
    * </p>
    *
    * @param random the random generator to use.
    * @param min tuple used as upper-bound for each component of the generated vector. Not modified.
    * @param max tuple used as lower-bound for each component of the generated vector. Not modified.
    * @param tupleToRandomize the tuple to randomize. Modified.
    * @throws RuntimeException if {@code min}<sub>i</sub> > {@code max}<sub>i</sub>.
    */
   public static void randomizeTuple3D(Random random, Tuple3DReadOnly min, Tuple3DReadOnly max, Tuple3DBasics tupleToRandomize)
   {
      for (int i = 0; i < 3; i++)
         tupleToRandomize.set(i, generateRandomDouble(random, min.get(i), max.get(i)));
   }

   /**
    * Randomizes a 2D tuple.
    * <p>
    * {@code tuple}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random the random generator to use.
    * @param tupleToRandomize the 2D tuple to randomize. Modified.
    */
   public static void randomizeTuple2D(Random random, Tuple2DBasics tupleToRandomize)
   {
      randomizeTuple2D(random, new Point2D(1.0, 1.0), tupleToRandomize);
   }

   /**
    * Randomizes a 2D tuple.
    * <p>
    * {@code tuple}<sub>i</sub> &in; [-{@code minMax}<sub>i</sub>; {@code minMax}<sub>i</sub>].
    * </p>
    *
    * @param random the random generator to use.
    * @param minMax tuple used to bound the maximum absolute value of each component of the
    *           generated vector. Not modified.
    * @param tupleToRandomize the 2D tuple to randomize. Modified.
    * @throws RuntimeException if any component of {@code minMax} is negative.
    */
   public static void randomizeTuple2D(Random random, Tuple2DReadOnly minMax, Tuple2DBasics tupleToRandomize)
   {
      for (int i = 0; i < 2; i++)
         tupleToRandomize.set(i, generateRandomDouble(random, minMax.get(i)));
   }

   /**
    * Randomizes a 2D tuple.
    * <p>
    * {@code vector}<sub>i</sub> &in; [{@code min}<sub>i</sub>; {@code max}<sub>i</sub>].
    * </p>
    *
    * @param random the random generator to use.
    * @param min tuple used as upper-bound for each component of the generated vector. Not modified.
    * @param max tuple used as lower-bound for each component of the generated vector. Not modified.
    * @param tupleToRandomize the 2D tuple to randomize. Modified.
    * @throws RuntimeException if {@code min}<sub>i</sub> > {@code max}<sub>i</sub>.
    */
   public static void randomizeTuple2D(Random random, Tuple2DReadOnly min, Tuple2DReadOnly max, Tuple2DBasics tupleToRandomize)
   {
      for (int i = 0; i < 2; i++)
         tupleToRandomize.set(i, generateRandomDouble(random, min.get(i), max.get(i)));
   }
}
