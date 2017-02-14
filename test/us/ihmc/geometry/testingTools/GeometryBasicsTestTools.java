package us.ihmc.geometry.testingTools;

import static us.ihmc.geometry.GeometryBasicsIOTools.*;

import java.util.Arrays;

import us.ihmc.geometry.TupleTools;
import us.ihmc.geometry.axisAngle.AxisAngle;
import us.ihmc.geometry.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.geometry.matrix.Matrix3DFeatures;
import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.geometry.transform.AffineTransform;
import us.ihmc.geometry.transform.QuaternionBasedTransform;
import us.ihmc.geometry.transform.RigidBodyTransform;
import us.ihmc.geometry.tuple2D.Vector2D;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple3D.Vector3D;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.geometry.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.tuple4D.Quaternion;
import us.ihmc.geometry.tuple4D.Vector4D;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Tuple4DReadOnly;

public abstract class GeometryBasicsTestTools
{
   private static final String DEFAULT_FORMAT = getStringFormat(15, 12);

   /**
    * Asserts that the two given angles are equal to an {@code epsilon}.
    * <p>
    * The method returns {@code true} for angles such as:
    * {@code actualAngle = expectedAngle +/- 2.0 * Math.PI}.
    * </p>
    *
    * @param expectedAngle the expected angle.
    * @param actualAngle the actual angle.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two angles are not equal.
    */
   public static void assertAngleEquals(double expectedAngle, double actualAngle, double epsilon)
   {
      assertAngleEquals(null, expectedAngle, actualAngle, epsilon);
   }

   /**
    * Asserts that the two given angles are equal to an {@code epsilon}.
    * <p>
    * The method returns {@code true} for angles such as:
    * {@code actualAngle = expectedAngle +/- 2.0 * Math.PI}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expectedAngle the expected angle.
    * @param actualAngle the actual angle.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two angles are not equal.
    */
   public static void assertAngleEquals(String messagePrefix, double expectedAngle, double actualAngle, double epsilon)
   {
      double differenceAngle = Math.abs(expectedAngle - actualAngle);
      differenceAngle = (differenceAngle + Math.PI) % (2.0 * Math.PI) - Math.PI;

      if (Math.abs(differenceAngle) > epsilon)
         throwNotEqualAssertionError(messagePrefix, Double.toString(expectedAngle), Double.toString(actualAngle));
   }

   /**
    * Asserts on a per component basis that the two sets of yaw-pitch-roll angles are equal to an
    * {@code epsilon}.
    * <p>
    * The method returns {@code true} for angles such as:
    * {@code actualAngle = expectedAngle +/- 2.0 * Math.PI}.
    * </p>
    *
    * @param expectedYawPitchRoll the expected set of yaw-pitch-roll angles. Not modified.
    * @param actualYawPitchRoll the actual set of yaw-pitch-roll angles. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two sets of yaw-pitch-roll angles are not equal.
    */
   public static void assertYawPitchRollEquals(double[] expectedYawPitchRoll, double[] actualYawPitchRoll, double epsilon)
   {
      assertYawPitchRollEquals(null, expectedYawPitchRoll, actualYawPitchRoll, epsilon);
   }

   /**
    * Asserts on a per component basis that the two sets of yaw-pitch-roll angles are equal to an
    * {@code epsilon}.
    * <p>
    * The method returns {@code true} for angles such as:
    * {@code actualAngle = expectedAngle +/- 2.0 * Math.PI}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expectedYawPitchRoll the expected set of yaw-pitch-roll angles. Not modified.
    * @param actualYawPitchRoll the actual set of yaw-pitch-roll angles. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two sets of yaw-pitch-roll angles are not equal.
    */
   public static void assertYawPitchRollEquals(String messagePrefix, double[] expectedYawPitchRoll, double[] actualYawPitchRoll, double epsilon)
   {
      try
      {
         assertAngleEquals(expectedYawPitchRoll[0], actualYawPitchRoll[0], epsilon);
         assertAngleEquals(expectedYawPitchRoll[1], actualYawPitchRoll[1], epsilon);
         assertAngleEquals(expectedYawPitchRoll[2], actualYawPitchRoll[2], epsilon);
      }
      catch (AssertionError e)
      {
         throwNotEqualAssertionError(messagePrefix, Arrays.toString(expectedYawPitchRoll), Arrays.toString(actualYawPitchRoll));
      }
   }

   /**
    * Asserts on a per component basis that the two rotation vectors are equal to an
    * {@code epsilon}.
    * <p>
    * The method returns {@code true} for angles such as:
    * {@code actualAngle = expectedAngle +/- 2.0 * Math.PI}.
    * </p>
    *
    * @param expectedRotationVector the expected rotation vector. Not modified.
    * @param actualRotationVector the actual rotation vector. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two rotation vectors are not equal.
    */
   public static void assertRotationVectorEquals(Vector3DReadOnly expectedRotationVector, Vector3DReadOnly actualRotationVector, double epsilon)
   {
      assertRotationVectorEquals(null, expectedRotationVector, actualRotationVector, epsilon);
   }

   /**
    * Asserts on a per component basis that the two rotation vectors are equal to an
    * {@code epsilon}.
    * <p>
    * The method returns {@code true} for angles such as:
    * {@code actualAngle = expectedAngle +/- 2.0 * Math.PI}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expectedRotationVector the expected rotation vector. Not modified.
    * @param actualRotationVector the actual rotation vector. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two rotation vectors are not equal.
    */
   public static void assertRotationVectorEquals(String messagePrefix, Vector3DReadOnly expectedRotationVector, Vector3DReadOnly actualRotationVector,
                                                 double epsilon)
   {
      assertRotationVectorEquals(messagePrefix, expectedRotationVector, actualRotationVector, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two rotation vectors are equal to an
    * {@code epsilon}.
    * <p>
    * The method returns {@code true} for angles such as:
    * {@code actualAngle = expectedAngle +/- 2.0 * Math.PI}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expectedRotationVector the expected rotation vector. Not modified.
    * @param actualRotationVector the actual rotation vector. Not modified.
    * @param epsilon the tolerance to use.
    * @param format the format to use for printing each component when an {@code AssertionError} is
    *           thrown.
    * @throws AssertionError if the two rotation vectors are not equal.
    */
   public static void assertRotationVectorEquals(String messagePrefix, Vector3DReadOnly expectedRotationVector, Vector3DReadOnly actualRotationVector,
                                                 double epsilon, String format)
   {
      try
      {
         assertAxisAngleEqualsSmart(new AxisAngle(expectedRotationVector), new AxisAngle(actualRotationVector), epsilon);
         // More reliable by going through the axis-angle.
         //         assertAngleEquals(expectedRotationVector.getX(), actualRotationVector.getX(), epsilon);
         //         assertAngleEquals(expectedRotationVector.getY(), actualRotationVector.getY(), epsilon);
         //         assertAngleEquals(expectedRotationVector.getZ(), actualRotationVector.getZ(), epsilon);
      }
      catch (AssertionError e)
      {
         throwNotEqualAssertionError(messagePrefix, expectedRotationVector, actualRotationVector, format);
      }
   }

   /**
    * Asserts on a per component basis that the two tuples are equal to an {@code epsilon}.
    *
    * @param expected the expected tuple.
    * @param actual the actual tuple.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two tuples are not equal.
    */
   public static void assertTuple2DEquals(Tuple2DReadOnly expected, Tuple2DReadOnly actual, double epsilon)
   {
      assertTuple2DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two tuples are equal to an {@code epsilon}.
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected the expected tuple.
    * @param actual the actual tuple.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two tuples are not equal.
    */
   public static void assertTuple2DEquals(String messagePrefix, Tuple2DReadOnly expected, Tuple2DReadOnly actual, double epsilon)
   {
      assertTuple2DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two tuples are equal to an {@code epsilon}.
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected the expected tuple.
    * @param actual the actual tuple.
    * @param epsilon the tolerance to use.
    * @param format the format to use for printing each component when an {@code AssertionError} is
    *           thrown.
    * @throws AssertionError if the two tuples are not equal.
    */
   public static void assertTuple2DEquals(String messagePrefix, Tuple2DReadOnly expected, Tuple2DReadOnly actual, double epsilon, String format)
   {
      if (!TupleTools.epsilonEquals(expected, actual, epsilon))
      {
         Vector2D difference = new Vector2D(actual);
         difference.sub(expected);
         throwNotEqualAssertionError(messagePrefix, expected, actual, difference.length(), format);
      }
   }

   /**
    * Asserts on a per component basis that the two tuples are equal to an {@code epsilon}.
    *
    * @param expected the expected tuple.
    * @param actual the actual tuple.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two tuples are not equal.
    */
   public static void assertTuple3DEquals(Tuple3DReadOnly expected, Tuple3DReadOnly actual, double epsilon)
   {
      assertTuple3DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two tuples are equal to an {@code epsilon}.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected the expected tuple.
    * @param actual the actual tuple.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two tuples are not equal.
    */
   public static void assertTuple3DEquals(String messagePrefix, Tuple3DReadOnly expected, Tuple3DReadOnly actual, double epsilon)
   {
      assertTuple3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two tuples are equal to an {@code epsilon}.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected the expected tuple.
    * @param actual the actual tuple.
    * @param epsilon the tolerance to use.
    * @param format the format to use for printing each component when an {@code AssertionError} is
    *           thrown.
    * @throws AssertionError if the two tuples are not equal.
    */
   public static void assertTuple3DEquals(String messagePrefix, Tuple3DReadOnly expected, Tuple3DReadOnly actual, double epsilon, String format)
   {
      if (!TupleTools.epsilonEquals(expected, actual, epsilon))
      {
         Vector3D difference = new Vector3D(actual);
         difference.sub(expected);
         throwNotEqualAssertionError(messagePrefix, expected, actual, difference.length(), format);
      }
   }

   /**
    * Asserts on a per component basis that the two tuples are equal to an {@code epsilon}.
    *
    * @param expected the expected tuple.
    * @param actual the actual tuple.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two tuples are not equal.
    */
   public static void assertTuple4DEquals(Tuple4DReadOnly expected, Tuple4DReadOnly actual, double epsilon)
   {
      assertTuple4DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two tuples are equal to an {@code epsilon}.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected the expected tuple.
    * @param actual the actual tuple.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two tuples are not equal.
    */
   public static void assertTuple4DEquals(String messagePrefix, Tuple4DReadOnly expected, Tuple4DReadOnly actual, double epsilon)
   {
      assertTuple4DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two tuples are equal to an {@code epsilon}.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected the expected tuple.
    * @param actual the actual tuple.
    * @param epsilon the tolerance to use.
    * @param format the format to use for printing each component when an {@code AssertionError} is
    *           thrown.
    * @throws AssertionError if the two tuples are not equal.
    */
   public static void assertTuple4DEquals(String messagePrefix, Tuple4DReadOnly expected, Tuple4DReadOnly actual, double epsilon, String format)
   {
      if (!TupleTools.epsilonEquals(expected, actual, epsilon))
      {
         Vector4D difference = new Vector4D(actual);
         difference.sub(expected);
         throwNotEqualAssertionError(messagePrefix, expected, actual, difference.norm(), format);
      }
   }

   /**
    * Asserts on a per component basis that the two matrices are equal to an {@code epsilon}.
    *
    * @param expected the expected matrix.
    * @param actual the actual matrix.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two matrices are not equal.
    */
   public static void assertMatrix3DEquals(Matrix3DReadOnly expected, Matrix3DReadOnly actual, double epsilon)
   {
      assertMatrix3DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two matrices are equal to an {@code epsilon}.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected the expected matrix.
    * @param actual the actual matrix.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two matrices are not equal.
    */
   public static void assertMatrix3DEquals(String messagePrefix, Matrix3DReadOnly expected, Matrix3DReadOnly actual, double epsilon)
   {
      assertMatrix3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two matrices are equal to an {@code epsilon}.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected the expected matrix.
    * @param actual the actual matrix.
    * @param epsilon the tolerance to use.
    * @param format the format to use for printing each component when an {@code AssertionError} is
    *           thrown.
    * @throws AssertionError if the two matrices are not equal.
    */
   public static void assertMatrix3DEquals(String messagePrefix, Matrix3DReadOnly expected, Matrix3DReadOnly actual, double epsilon, String format)
   {
      if (!Matrix3DFeatures.epsilonEquals(expected, actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts that the given matrix is skew-symmetric:
    *
    * <pre>
    *     |  0 -z  y |
    * m = |  z  0 -x |
    *     | -y  x  0 |
    * </pre>
    * <p>
    * This matrix is considered to be skew symmetric if:
    * <ul>
    * <li>each diagonal coefficient is equal to 0.0 +/- {@code epsilon},
    * <li>the sums of each pair of cross-diagonal coefficients ({@code m10}, {@code m01}),
    * ({@code m12}, {@code m21}), and ({@code m20}, {@code m02}) are equal to 0.0 +/-
    * {@code epsilon}.
    * </ul>
    * </p>
    *
    * @param matrix the query. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the matrix is not skew-symmetric.
    */
   public static void assertSkewSymmetric(Matrix3DReadOnly matrix, double epsilon)
   {
      assertSkewSymmetric(null, matrix, epsilon);
   }

   /**
    * Asserts that the given matrix is skew-symmetric:
    *
    * <pre>
    *     |  0 -z  y |
    * m = |  z  0 -x |
    *     | -y  x  0 |
    * </pre>
    * <p>
    * This matrix is considered to be skew symmetric if:
    * <ul>
    * <li>each diagonal coefficient is equal to 0.0 +/- {@code epsilon},
    * <li>the sums of each pair of cross-diagonal coefficients ({@code m10}, {@code m01}),
    * ({@code m12}, {@code m21}), and ({@code m20}, {@code m02}) are equal to 0.0 +/-
    * {@code epsilon}.
    * </ul>
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param matrix the query. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the matrix is not skew-symmetric.
    */
   public static void assertSkewSymmetric(String messagePrefix, Matrix3DReadOnly matrix, double epsilon)
   {
      assertSkewSymmetric(messagePrefix, matrix, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the given matrix is skew-symmetric:
    *
    * <pre>
    *     |  0 -z  y |
    * m = |  z  0 -x |
    *     | -y  x  0 |
    * </pre>
    * <p>
    * This matrix is considered to be skew symmetric if:
    * <ul>
    * <li>each diagonal coefficient is equal to 0.0 +/- {@code epsilon},
    * <li>the sums of each pair of cross-diagonal coefficients ({@code m10}, {@code m01}),
    * ({@code m12}, {@code m21}), and ({@code m20}, {@code m02}) are equal to 0.0 +/-
    * {@code epsilon}.
    * </ul>
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param matrix the query. Not modified.
    * @param epsilon the tolerance to use.
    * @param format the format to use for printing each component when an {@code AssertionError} is
    *           thrown.
    * @throws AssertionError if the matrix is not skew-symmetric.
    */
   public static void assertSkewSymmetric(String messagePrefix, Matrix3DReadOnly matrix, double epsilon, String format)
   {
      if (!matrix.isMatrixSkewSymmetric(epsilon))
      {
         String errorMessage = "The matrix is not skew-symmetric:\n" + getMatrixString(DEFAULT_FORMAT, matrix);
         throw new AssertionError(addPrefixToMessage(messagePrefix, errorMessage));
      }
   }

   /**
    * Asserts that this matrix is a rotation matrix.
    * <p>
    * This matrix is a rotation matrix if:
    * <ul>
    * <li>the length of each row vector is equal to 1.0 +/- {@code epsilon},
    * <li>the dot product of each pair of row vectors is equal to 0.0 +/- {@code epsilon},
    * <li>the determinant of the matrix is equal to 1.0 +/- {@code epsilon}.
    * </ul>
    * </p>
    *
    * @param matrix the query. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the matrix is not a rotation matrix.
    */
   public static void assertRotationMatrix(Matrix3DReadOnly matrix, double epsilon)
   {
      assertRotationMatrix(null, matrix, epsilon);
   }

   /**
    * Asserts that this matrix is a rotation matrix.
    * <p>
    * This matrix is a rotation matrix if:
    * <ul>
    * <li>the length of each row vector is equal to 1.0 +/- {@code epsilon},
    * <li>the dot product of each pair of row vectors is equal to 0.0 +/- {@code epsilon},
    * <li>the determinant of the matrix is equal to 1.0 +/- {@code epsilon}.
    * </ul>
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param matrix the query. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the matrix is not a rotation matrix.
    */
   public static void assertRotationMatrix(String messagePrefix, Matrix3DReadOnly matrix, double epsilon)
   {
      assertRotationMatrix(messagePrefix, matrix, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that this matrix is a rotation matrix.
    * <p>
    * This matrix is a rotation matrix if:
    * <ul>
    * <li>the length of each row vector is equal to 1.0 +/- {@code epsilon},
    * <li>the dot product of each pair of row vectors is equal to 0.0 +/- {@code epsilon},
    * <li>the determinant of the matrix is equal to 1.0 +/- {@code epsilon}.
    * </ul>
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param matrix the query. Not modified.
    * @param epsilon the tolerance to use.
    * @param format the format to use for printing each component when an {@code AssertionError} is
    *           thrown.
    * @throws AssertionError if the matrix is not a rotation matrix.
    */
   public static void assertRotationMatrix(String messagePrefix, Matrix3DReadOnly matrix, double epsilon, String format)
   {
      if (!matrix.isRotationMatrix(epsilon))
      {
         String errorMessage = "This is not a rotation matrix:\n" + getMatrixString(format, matrix);
         throw new AssertionError(addPrefixToMessage(messagePrefix, errorMessage));
      }
   }

   /**
    * Asserts on a per coefficient basis that this matrix is equal to identity to an
    * {@code epsilon}.
    *
    * @param matrix the query. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the matrix is not identity.
    */
   public static void assertIdentity(Matrix3DReadOnly matrix, double epsilon)
   {
      assertIdentity(null, matrix, epsilon);
   }

   /**
    * Asserts on a per coefficient basis that this matrix is equal to identity to an
    * {@code epsilon}.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param matrix the query. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the matrix is not identity.
    */
   public static void assertIdentity(String messagePrefix, Matrix3DReadOnly matrix, double epsilon)
   {
      assertIdentity(messagePrefix, matrix, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per coefficient basis that this matrix is equal to identity to an
    * {@code epsilon}.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param matrix the query. Not modified.
    * @param epsilon the tolerance to use.
    * @param format the format to use for printing each component when an {@code AssertionError} is
    *           thrown.
    * @throws AssertionError if the matrix is not identity.
    */
   public static void assertIdentity(String messagePrefix, Matrix3DReadOnly matrix, double epsilon, String format)
   {
      if (!matrix.isIdentity(epsilon))
      {
         String errorMessage = "The matrix is not identity:\n" + getMatrixString(DEFAULT_FORMAT, matrix);
         throw new AssertionError(addPrefixToMessage(messagePrefix, errorMessage));
      }
   }

   /**
    * Asserts that the given matrix contains on {@link Double#NaN}.
    *
    * @param matrix the query. Not modified.
    * @throws AssertionError if the matrix does not only contain {@link Double#NaN}.
    */
   public static void assertMatrix3DContainsOnlyNaN(Matrix3DReadOnly matrix)
   {
      assertMatrix3DContainsOnlyNaN(null, matrix);
   }

   /**
    * Asserts that the given matrix contains on {@link Double#NaN}.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param matrix the query. Not modified.
    * @throws AssertionError if the matrix does not only contain {@link Double#NaN}.
    */
   public static void assertMatrix3DContainsOnlyNaN(String messagePrefix, Matrix3DReadOnly matrix)
   {
      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            if (!Double.isNaN(matrix.getElement(row, column)))
            {
               String errorMessage = "The matrix does not contain only NaN:\n" + getMatrixString(DEFAULT_FORMAT, matrix);
               throw new AssertionError(addPrefixToMessage(messagePrefix, errorMessage));
            }
         }
      }
   }

   /**
    * Asserts on a per component basis that the two quaternions are equal to an {@code epsilon}.
    *
    * @param expectedQuaternion the expected quaternion. Not modified.
    * @param actualQuaternion the actual quaternion. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two quaternions are not equal.
    */
   public static void assertQuaternionEquals(QuaternionReadOnly expectedQuaternion, QuaternionReadOnly actualQuaternion, double epsilon)
   {
      assertTuple4DEquals(expectedQuaternion, actualQuaternion, epsilon);
   }

   /**
    * Asserts on a per component basis that the two quaternions are equal to an {@code epsilon}.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expectedQuaternion the expected quaternion. Not modified.
    * @param actualQuaternion the actual quaternion. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two quaternions are not equal.
    */
   public static void assertQuaternionEquals(String messagePrefix, QuaternionReadOnly expectedQuaternion, QuaternionReadOnly actualQuaternion, double epsilon)
   {
      assertTuple4DEquals(messagePrefix, expectedQuaternion, actualQuaternion, epsilon);
   }

   /**
    * Asserts on a per component basis that the two quaternions are equal to an {@code epsilon}.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expectedQuaternion the expected quaternion. Not modified.
    * @param actualQuaternion the actual quaternion. Not modified.
    * @param epsilon the tolerance to use.
    * @param format the format to use for printing each component when an {@code AssertionError} is
    *           thrown.
    * @throws AssertionError if the two quaternions are not equal.
    */
   public static void assertQuaternionEquals(String messagePrefix, QuaternionReadOnly expectedQuaternion, QuaternionReadOnly actualQuaternion, double epsilon,
                                             String format)
   {
      assertTuple4DEquals(messagePrefix, expectedQuaternion, actualQuaternion, epsilon, format);
   }

   /**
    * Asserts on a per component basis that the two quaternions are equal to an {@code epsilon}.
    * <p>
    * This method changes the sign of one of the two quaternions when comparing if their dot product
    * is negative.
    * </p>
    *
    * @param expectedQuaternion the expected quaternion. Not modified.
    * @param actualQuaternion the actual quaternion. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two quaternions are not equal.
    */
   public static void assertQuaternionEqualsSmart(QuaternionReadOnly expectedQuaternion, QuaternionReadOnly actualQuaternion, double epsilon)
   {
      assertQuaternionEqualsSmart(null, expectedQuaternion, actualQuaternion, epsilon);
   }

   /**
    * Asserts on a per component basis that the two quaternions are equal to an {@code epsilon}.
    * <p>
    * This method changes the sign of one of the two quaternions when comparing if their dot product
    * is negative.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expectedQuaternion the expected quaternion. Not modified.
    * @param actualQuaternion the actual quaternion. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two quaternions are not equal.
    */
   public static void assertQuaternionEqualsSmart(String messagePrefix, QuaternionReadOnly expectedQuaternion, QuaternionReadOnly actualQuaternion,
                                                  double epsilon)
   {
      assertQuaternionEqualsSmart(messagePrefix, expectedQuaternion, actualQuaternion, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two quaternions are equal to an {@code epsilon}.
    * <p>
    * This method changes the sign of one of the two quaternions when comparing if their dot product
    * is negative.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expectedQuaternion the expected quaternion. Not modified.
    * @param actualQuaternion the actual quaternion. Not modified.
    * @param epsilon the tolerance to use.
    * @param format the format to use for printing each component when an {@code AssertionError} is
    *           thrown.
    * @throws AssertionError if the two quaternions are not equal.
    */
   public static void assertQuaternionEqualsSmart(String messagePrefix, QuaternionReadOnly expectedQuaternion, QuaternionReadOnly actualQuaternion,
                                                  double epsilon, String format)
   {
      if (expectedQuaternion.dot(actualQuaternion) < 0.0)
      {
         Quaternion quaternion = new Quaternion();
         quaternion.setAndNegate(actualQuaternion);
         actualQuaternion = quaternion;
      }
      assertQuaternionEquals(messagePrefix, expectedQuaternion, actualQuaternion, epsilon, format);
   }

   /**
    * Asserts that the two given quaternions represents the same orientation to an {@code epsilon}
    * by calculating the magnitude of their difference.
    *
    * @param expectedQuaternion the expected quaternion. Not modified.
    * @param actualQuaternion the actual quaternion. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two quaternions do not represent the same orientation.
    */
   public static void assertQuaternionEqualsUsingDifference(QuaternionReadOnly expectedQuaternion, QuaternionReadOnly actualQuaternion, double epsilon)
   {
      assertQuaternionEqualsUsingDifference(null, expectedQuaternion, actualQuaternion, epsilon);
   }

   /**
    * Asserts that the two given quaternions represents the same orientation to an {@code epsilon}
    * by calculating the magnitude of their difference.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expectedQuaternion the expected quaternion. Not modified.
    * @param actualQuaternion the actual quaternion. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two quaternions do not represent the same orientation.
    */
   public static void assertQuaternionEqualsUsingDifference(String messagePrefix, QuaternionReadOnly expectedQuaternion, QuaternionReadOnly actualQuaternion,
                                                            double epsilon)
   {
      assertQuaternionEqualsUsingDifference(messagePrefix, expectedQuaternion, actualQuaternion, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the two given quaternions represents the same orientation to an {@code epsilon}
    * by calculating the magnitude of their difference.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expectedQuaternion the expected quaternion. Not modified.
    * @param actualQuaternion the actual quaternion. Not modified.
    * @param epsilon the tolerance to use.
    * @param format the format to use for printing each component when an {@code AssertionError} is
    *           thrown.
    * @throws AssertionError if the two quaternions do not represent the same orientation.
    */
   public static void assertQuaternionEqualsUsingDifference(String messagePrefix, QuaternionReadOnly expectedQuaternion, QuaternionReadOnly actualQuaternion,
                                                            double epsilon, String format)
   {
      Quaternion qDifference = new Quaternion(expectedQuaternion);
      qDifference.multiplyConjugateOther(actualQuaternion);
      double angleDifference = qDifference.getAngle();
      if (Math.abs(angleDifference) > epsilon)
      {
         throwNotEqualAssertionError(messagePrefix, expectedQuaternion, actualQuaternion, angleDifference, format);
      }
   }

   /**
    * Asserts on a per component basis if the two axis-angles are equal to an {@code epsilon}.
    *
    * @param expectedAxisAngle the expected axis-angle. Not modified.
    * @param actualAxisAngle the actual axis-angle. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two axis-angles are not equal.
    */
   public static void assertAxisAngleEquals(AxisAngleReadOnly expectedAxisAngle, AxisAngleReadOnly actualAxisAngle, double epsilon)
   {
      assertAxisAngleEquals(null, expectedAxisAngle, actualAxisAngle, epsilon);
   }

   /**
    * Asserts on a per component basis if the two axis-angles are equal to an {@code epsilon}.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expectedAxisAngle the expected axis-angle. Not modified.
    * @param actualAxisAngle the actual axis-angle. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two axis-angles are not equal.
    */
   public static void assertAxisAngleEquals(String messagePrefix, AxisAngleReadOnly expectedAxisAngle, AxisAngleReadOnly actualAxisAngle, double epsilon)
   {
      assertAxisAngleEquals(messagePrefix, expectedAxisAngle, actualAxisAngle, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis if the two axis-angles are equal to an {@code epsilon}.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expectedAxisAngle the expected axis-angle. Not modified.
    * @param actualAxisAngle the actual axis-angle. Not modified.
    * @param epsilon the tolerance to use.
    * @param format the format to use for printing each component when an {@code AssertionError} is
    *           thrown.
    * @throws AssertionError if the two axis-angles are not equal.
    */
   public static void assertAxisAngleEquals(String messagePrefix, AxisAngleReadOnly expectedAxisAngle, AxisAngleReadOnly actualAxisAngle, double epsilon,
                                            String format)
   {
      if (!expectedAxisAngle.epsilonEquals(actualAxisAngle, epsilon))
         throwNotEqualAssertionError(messagePrefix, expectedAxisAngle, actualAxisAngle, format);
   }

   /**
    * Asserts on a per component basis if the two axis-angles are equal to an {@code epsilon}.
    * <p>
    * This method changes the sign of one of the two axis-angles when comparing if the dot product
    * of their axis is negative.
    * </p>
    *
    * @param expectedAxisAngle the expected axis-angle. Not modified.
    * @param actualAxisAngle the actual axis-angle. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two axis-angles are not equal.
    */
   public static void assertAxisAngleEqualsSmart(AxisAngleReadOnly expectedAxisAngle, AxisAngleReadOnly actualAxisAngle, double epsilon)
   {
      assertAxisAngleEqualsSmart(null, expectedAxisAngle, actualAxisAngle, epsilon);
   }

   /**
    * Asserts on a per component basis if the two axis-angles are equal to an {@code epsilon}.
    * <p>
    * This method changes the sign of one of the two axis-angles when comparing if the dot product
    * of their axis is negative.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expectedAxisAngle the expected axis-angle. Not modified.
    * @param actualAxisAngle the actual axis-angle. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two axis-angles are not equal.
    */
   public static void assertAxisAngleEqualsSmart(String messagePrefix, AxisAngleReadOnly expectedAxisAngle, AxisAngleReadOnly actualAxisAngle, double epsilon)
   {
      assertAxisAngleEqualsSmart(messagePrefix, expectedAxisAngle, actualAxisAngle, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis if the two axis-angles are equal to an {@code epsilon}.
    * <p>
    * This method changes the sign of one of the two axis-angles when comparing if the dot product
    * of their axis is negative.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expectedAxisAngle the expected axis-angle. Not modified.
    * @param actualAxisAngle the actual axis-angle. Not modified.
    * @param epsilon the tolerance to use.
    * @param format the format to use for printing each component when an {@code AssertionError} is
    *           thrown.
    * @throws AssertionError if the two axis-angles are not equal.
    */
   public static void assertAxisAngleEqualsSmart(String messagePrefix, AxisAngleReadOnly expectedAxisAngle, AxisAngleReadOnly actualAxisAngle, double epsilon,
                                                 String format)
   {
      double expectedX = expectedAxisAngle.getX();
      double expectedY = expectedAxisAngle.getY();
      double expectedZ = expectedAxisAngle.getZ();
      double actualX = actualAxisAngle.getX();
      double actualY = actualAxisAngle.getY();
      double actualZ = actualAxisAngle.getZ();
      double actualAngle = actualAxisAngle.getAngle();

      AxisAngleReadOnly actualAxisAngleOriginal = actualAxisAngle;

      if (expectedX * actualX + expectedY * actualY + expectedZ * actualZ < 0.0)
      {
         actualAxisAngle = new AxisAngle(-actualX, -actualY, -actualZ, -actualAngle);
      }

      for (int index = 0; index < 3; index++)
      {
         double diff = expectedAxisAngle.get(index) - actualAxisAngle.get(index);
         if (Math.abs(diff) > epsilon)
            throwNotEqualAssertionError(messagePrefix, expectedAxisAngle, actualAxisAngleOriginal, format);
      }

      try
      {
         assertAngleEquals(expectedAxisAngle.getAngle(), actualAxisAngle.getAngle(), epsilon);
      }
      catch (AssertionError e)
      {
         throwNotEqualAssertionError(messagePrefix, expectedAxisAngle, actualAxisAngleOriginal, format);
      }
   }

   /**
    * Asserts that the given axis-angle contains only {@link Double#NaN}.
    *
    * @param axisAngleToAssert the query. Not modified.
    * @throws AssertionError if the axis-angle does not only contain {@link Double#NaN}.
    */
   public static void assertAxisAngleContainsOnlyNaN(AxisAngleReadOnly axisAngleToAssert)
   {
      assertAxisAngleContainsOnlyNaN(null, axisAngleToAssert);
   }

   /**
    * Asserts that the given axis-angle contains only {@link Double#NaN}.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param axisAngleToAssert the query. Not modified.
    * @throws AssertionError if the axis-angle does not only contain {@link Double#NaN}.
    */
   public static void assertAxisAngleContainsOnlyNaN(String messagePrefix, AxisAngleReadOnly axisAngleToAssert)
   {
      for (int index = 0; index < 4; index++)
      {
         if (!Double.isNaN(axisAngleToAssert.get(index)))
         {
            String errorMessage = "The axis-angle does not contain only NaN:\n" + getAxisAngleString(DEFAULT_FORMAT, axisAngleToAssert);
            throw new AssertionError(addPrefixToMessage(messagePrefix, errorMessage));
         }
      }
   }

   /**
    * Assert that {@link AxisAngleBasics#setToZero()} has just been called on the given axis-angle.
    *
    * @param axisAngleToAssert the query. Not modified.
    * @throws AssertionError if the axis-angle has not been set to zero.
    */
   public static void assertAxisAngleIsSetToZero(AxisAngleReadOnly axisAngleToAssert)
   {
      assertAxisAngleIsSetToZero(null, axisAngleToAssert);
   }

   /**
    * Assert that {@link AxisAngleBasics#setToZero()} has just been called on the given axis-angle.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param axisAngleToAssert the query. Not modified.
    * @throws AssertionError if the axis-angle has not been set to zero.
    */
   public static void assertAxisAngleIsSetToZero(String messagePrefix, AxisAngleReadOnly axisAngleToAssert)
   {
      AxisAngle expected = new AxisAngle(1.0, 0.0, 0.0, 0.0);
      if (!expected.equals(axisAngleToAssert))
      {
         String errorMessage = "The axis-angle has not been set to zero:\n" + getAxisAngleString(DEFAULT_FORMAT, axisAngleToAssert);
         throw new AssertionError(addPrefixToMessage(messagePrefix, errorMessage));
      }
   }

   /**
    * Asserts that the length of the axis of the axis-angle is equal to {@code 1.0 +/- epsilon}.
    *
    * @param axisAngleToAssert the query. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the axis is not unitary.
    */
   public static void assertAxisUnitary(AxisAngleReadOnly axisAngleToAssert, double epsilon)
   {
      assertAxisUnitary(null, axisAngleToAssert, epsilon);
   }

   /**
    * Asserts that the length of the axis of the axis-angle is equal to {@code 1.0 +/- epsilon}.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param axisAngleToAssert the query. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the axis is not unitary.
    */
   public static void assertAxisUnitary(String messagePrefix, AxisAngleReadOnly axisAngleToAssert, double epsilon)
   {
      assertAxisUnitary(messagePrefix, axisAngleToAssert, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the length of the axis of the axis-angle is equal to {@code 1.0 +/- epsilon}.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param axisAngleToAssert the query. Not modified.
    * @param epsilon the tolerance to use.
    * @param format the format to use for printing each component when an {@code AssertionError} is
    *           thrown.
    * @throws AssertionError if the axis is not unitary.
    */
   public static void assertAxisUnitary(String messagePrefix, AxisAngleReadOnly axisAngleToAssert, double epsilon, String format)
   {
      if (!axisAngleToAssert.isAxisUnitary(epsilon))
      {
         String errorMessage = "The axis of the given axis-angle is not unitary: " + getAxisAngleString(format, axisAngleToAssert);
         throw new AssertionError(addPrefixToMessage(messagePrefix, errorMessage));
      }
   }

   /**
    * Assert that {@link QuaternionBasics#setToZero()} has just been called on the given quaternion.
    *
    * @param quaternionToAssert the query. Not modified.
    * @throws AssertionError if the quaternion has not been set to zero.
    */
   public static void assertQuaternionIsSetToZero(QuaternionReadOnly quaternionToAssert)
   {
      assertQuaternionIsSetToZero(null, quaternionToAssert);
   }

   /**
    * Assert that {@link QuaternionBasics#setToZero()} has just been called on the given quaternion.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param quaternionToAssert the query. Not modified.
    * @throws AssertionError if the quaternion has not been set to zero.
    */
   public static void assertQuaternionIsSetToZero(String messagePrefix, QuaternionReadOnly quaternionToAssert)
   {
      assertQuaternionIsSetToZero(messagePrefix, quaternionToAssert, DEFAULT_FORMAT);
   }

   /**
    * Assert that {@link QuaternionBasics#setToZero()} has just been called on the given quaternion.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param quaternionToAssert the query. Not modified.
    * @param format the format to use for printing each component when an {@code AssertionError} is
    *           thrown.
    * @throws AssertionError if the quaternion has not been set to zero.
    */
   public static void assertQuaternionIsSetToZero(String messagePrefix, QuaternionReadOnly quaternionToAssert, String format)
   {
      Quaternion expected = new Quaternion(0.0, 0.0, 0.0, 1.0);

      if (!expected.equals(quaternionToAssert))
      {
         String errorMessage = "The axis-angle has not been set to zero:\n" + getTuple4DString(format, quaternionToAssert);
         throw new AssertionError(addPrefixToMessage(messagePrefix, errorMessage));
      }
   }

   /**
    * Asserts that the norm of the given quaternion is equal to {@code 1.0 +/- epsilon}.
    *
    * @param quaternionToAssert the query. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the quaternion is not a unit-quaternion.
    */
   public static void assertQuaternionIsUnitary(QuaternionReadOnly quaternionToAssert, double epsilon)
   {
      assertQuaternionIsUnitary(null, quaternionToAssert, epsilon);
   }

   /**
    * Asserts that the norm of the given quaternion is equal to {@code 1.0 +/- epsilon}.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param quaternionToAssert the query. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the quaternion is not a unit-quaternion.
    */
   public static void assertQuaternionIsUnitary(String messagePrefix, QuaternionReadOnly quaternionToAssert, double epsilon)
   {
      assertQuaternionIsUnitary(messagePrefix, quaternionToAssert, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the norm of the given quaternion is equal to {@code 1.0 +/- epsilon}.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param quaternionToAssert the query. Not modified.
    * @param epsilon the tolerance to use.
    * @param format the format to use for printing each component when an {@code AssertionError} is
    *           thrown.
    * @throws AssertionError if the quaternion is not a unit-quaternion.
    */
   public static void assertQuaternionIsUnitary(String messagePrefix, QuaternionReadOnly quaternionToAssert, double epsilon, String format)
   {
      if (!quaternionToAssert.isUnitary(epsilon))
      {
         String errorMessage = "The quaternion is not unitary: " + getTuple4DString(format, quaternionToAssert);
         throw new AssertionError(addPrefixToMessage(messagePrefix, errorMessage));
      }
   }

   /**
    * Asserts that the given tuple contains only {@link Double#NaN}.
    *
    * @param tupleToAssert the query. Not modified.
    * @throws AssertionError if the tuple does not only contain {@link Double#NaN}.
    */
   public static void assertTuple2DContainsOnlyNaN(Tuple2DReadOnly tupleToAssert)
   {
      assertTuple2DContainsOnlyNaN(null, tupleToAssert);
   }

   /**
    * Asserts that the given tuple contains only {@link Double#NaN}.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param tupleToAssert the query. Not modified.
    * @throws AssertionError if the tuple does not only contain {@link Double#NaN}.
    */
   public static void assertTuple2DContainsOnlyNaN(String messagePrefix, Tuple2DReadOnly tupleToAssert)
   {
      for (int index = 0; index < 2; index++)
      {
         if (!Double.isNaN(tupleToAssert.get(index)))
         {
            String errorMessage = "The tuple does not contain only NaN:\n" + getTuple2DString(DEFAULT_FORMAT, tupleToAssert);
            throw new AssertionError(addPrefixToMessage(messagePrefix, errorMessage));
         }
      }
   }

   /**
    * Assert that {@link Tuple2DBasics#setToZero()} has just been called on the given tuple.
    *
    * @param tupleToAssert the query. Not modified.
    * @throws AssertionError if the tuple has not been set to zero.
    */
   public static void assertTuple2DIsSetToZero(Tuple2DReadOnly tupleToAssert)
   {
      assertTuple2DIsSetToZero(null, tupleToAssert);
   }

   /**
    * Assert that {@link Tuple2DBasics#setToZero()} has just been called on the given tuple.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param tupleToAssert the query. Not modified.
    * @throws AssertionError if the tuple has not been set to zero.
    */
   public static void assertTuple2DIsSetToZero(String messagePrefix, Tuple2DReadOnly tupleToAssert)
   {
      assertTuple2DIsSetToZero(messagePrefix, tupleToAssert, DEFAULT_FORMAT);
   }

   /**
    * Assert that {@link Tuple2DBasics#setToZero()} has just been called on the given tuple.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param tupleToAssert the query. Not modified.
    * @param format the format to use for printing each component when an {@code AssertionError} is
    *           thrown.
    * @throws AssertionError if the tuple has not been set to zero.
    */
   public static void assertTuple2DIsSetToZero(String messagePrefix, Tuple2DReadOnly tupleToAssert, String format)
   {
      for (int index = 0; index < 2; index++)
      {
         if (tupleToAssert.get(index) != 0.0)
         {
            String errorMessage = "The tuple has not been set to zero:\n " + getTuple2DString(format, tupleToAssert);
            throw new AssertionError(addPrefixToMessage(messagePrefix, errorMessage));
         }
      }
   }

   /**
    * Asserts that the given tuple contains only {@link Double#NaN}.
    *
    * @param tupleToAssert the query. Not modified.
    * @throws AssertionError if the tuple does not only contain {@link Double#NaN}.
    */
   public static void assertTuple3DContainsOnlyNaN(Tuple3DReadOnly tupleToAssert)
   {
      assertTuple3DContainsOnlyNaN(null, tupleToAssert);
   }

   /**
    * Asserts that the given tuple contains only {@link Double#NaN}.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param tupleToAssert the query. Not modified.
    * @throws AssertionError if the tuple does not only contain {@link Double#NaN}.
    */
   public static void assertTuple3DContainsOnlyNaN(String messagePrefix, Tuple3DReadOnly tupleToAssert)
   {
      for (int index = 0; index < 3; index++)
      {
         if (!Double.isNaN(tupleToAssert.get(index)))
         {
            String errorMessage = "The tuple does not contain only NaN:\n" + getTuple3DString(DEFAULT_FORMAT, tupleToAssert);
            throw new AssertionError(addPrefixToMessage(messagePrefix, errorMessage));
         }
      }
   }

   /**
    * Assert that {@link Tuple3DBasics#setToZero()} has just been called on the given tuple.
    *
    * @param tupleToAssert the query. Not modified.
    * @throws AssertionError if the tuple has not been set to zero.
    */
   public static void assertTuple3DIsSetToZero(Tuple3DReadOnly tupleToAssert)
   {
      assertTuple3DIsSetToZero(null, tupleToAssert);
   }

   /**
    * Assert that {@link Tuple3DBasics#setToZero()} has just been called on the given tuple.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param tupleToAssert the query. Not modified.
    * @throws AssertionError if the tuple has not been set to zero.
    */
   public static void assertTuple3DIsSetToZero(String messagePrefix, Tuple3DReadOnly tupleToAssert)
   {
      assertTuple3DIsSetToZero(messagePrefix, tupleToAssert, DEFAULT_FORMAT);
   }

   /**
    * Assert that {@link Tuple3DBasics#setToZero()} has just been called on the given tuple.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param tupleToAssert the query. Not modified.
    * @param format the format to use for printing each component when an {@code AssertionError} is
    *           thrown.
    * @throws AssertionError if the tuple has not been set to zero.
    */
   public static void assertTuple3DIsSetToZero(String messagePrefix, Tuple3DReadOnly tupleToAssert, String format)
   {
      for (int index = 0; index < 3; index++)
      {
         if (tupleToAssert.get(index) != 0.0)
         {
            String errorMessage = "The tuple has not been set to zero:\n " + getTuple3DString(format, tupleToAssert);
            throw new AssertionError(addPrefixToMessage(messagePrefix, errorMessage));
         }
      }
   }

   /**
    * Asserts that the given tuple contains only {@link Double#NaN}.
    *
    * @param quaternionToAssert the query. Not modified.
    * @throws AssertionError if the quaternion does not only contain {@link Double#NaN}.
    */
   public static void assertTuple4DContainsOnlyNaN(Tuple4DReadOnly quaternionToAssert)
   {
      assertTuple4DContainsOnlyNaN(null, quaternionToAssert);
   }

   /**
    * Asserts that the given quaternion contains only {@link Double#NaN}.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param quaternionToAssert the query. Not modified.
    * @throws AssertionError if the quaternion does not only contain {@link Double#NaN}.
    */
   public static void assertTuple4DContainsOnlyNaN(String messagePrefix, Tuple4DReadOnly quaternionToAssert)
   {
      for (int index = 0; index < 4; index++)
      {
         if (!Double.isNaN(quaternionToAssert.get(index)))
         {
            String errorMessage = "The tuple does not contain only NaN:\n" + getTuple4DString(DEFAULT_FORMAT, quaternionToAssert);
            throw new AssertionError(addPrefixToMessage(messagePrefix, errorMessage));
         }
      }
   }

   /**
    * Asserts on a per component basis that the two given rigid-body transform are equal to an
    * {@code epsilon}.
    *
    * @param expected the expected rigid-body transform. Not modified.
    * @param actual the actual rigid-body transform. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two rigid-body transforms are not equal.
    */
   public static void assertRigidBodyTransformEquals(RigidBodyTransform expected, RigidBodyTransform actual, double epsilon)
   {
      assertRigidBodyTransformEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two given rigid-body transform are equal to an
    * {@code epsilon}.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected the expected rigid-body transform. Not modified.
    * @param actual the actual rigid-body transform. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two rigid-body transforms are not equal.
    */
   public static void assertRigidBodyTransformEquals(String messagePrefix, RigidBodyTransform expected, RigidBodyTransform actual, double epsilon)
   {
      assertRigidBodyTransformEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two given rigid-body transform are equal to an
    * {@code epsilon}.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected the expected rigid-body transform. Not modified.
    * @param actual the actual rigid-body transform. Not modified.
    * @param epsilon the tolerance to use.
    * @param format the format to use for printing each component when an {@code AssertionError} is
    *           thrown.
    * @throws AssertionError if the two rigid-body transforms are not equal.
    */
   public static void assertRigidBodyTransformEquals(String messagePrefix, RigidBodyTransform expected, RigidBodyTransform actual, double epsilon,
                                                     String format)
   {
      if (!expected.epsilonEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts on a per component basis that the two quaternion-based transforms are equal to an
    * {@code epsilon}.
    *
    * @param expected the expected quaternion-based transform. Not modified.
    * @param actual the actual quaternion-based transform. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two quaternion-based transforms are not equal.
    */
   public static void assertQuaternionBasedTransformEquals(QuaternionBasedTransform expected, QuaternionBasedTransform actual, double epsilon)
   {
      assertQuaternionBasedTransformEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two quaternion-based transforms are equal to an
    * {@code epsilon}.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected the expected quaternion-based transform. Not modified.
    * @param actual the actual quaternion-based transform. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two quaternion-based transforms are not equal.
    */
   public static void assertQuaternionBasedTransformEquals(String messagePrefix, QuaternionBasedTransform expected, QuaternionBasedTransform actual,
                                                           double epsilon)
   {
      assertQuaternionBasedTransformEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two quaternion-based transforms are equal to an
    * {@code epsilon}.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected the expected quaternion-based transform. Not modified.
    * @param actual the actual quaternion-based transform. Not modified.
    * @param epsilon the tolerance to use.
    * @param format the format to use for printing each component when an {@code AssertionError} is
    *           thrown.
    * @throws AssertionError if the two quaternion-based transforms are not equal.
    */
   public static void assertQuaternionBasedTransformEquals(String messagePrefix, QuaternionBasedTransform expected, QuaternionBasedTransform actual,
                                                           double epsilon, String format)
   {
      if (!expected.epsilonEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts on a per component basis that the two quaternion-based transforms are equal to an
    * {@code epsilon}.
    * <p>
    * This method compares the quaternions using
    * {@link #assertQuaternionEqualsSmart(QuaternionReadOnly, QuaternionReadOnly, double)}.
    * </p>
    *
    * @param expected the expected quaternion-based transform. Not modified.
    * @param actual the actual quaternion-based transform. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two quaternion-based transforms are not equal.
    */
   public static void assertQuaternionBasedTransformEqualsSmart(QuaternionBasedTransform expected, QuaternionBasedTransform actual, double epsilon)
   {
      assertQuaternionBasedTransformEqualsSmart(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two quaternion-based transforms are equal to an
    * {@code epsilon}.
    * <p>
    * This method compares the quaternions using
    * {@link #assertQuaternionEqualsSmart(QuaternionReadOnly, QuaternionReadOnly, double)}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected the expected quaternion-based transform. Not modified.
    * @param actual the actual quaternion-based transform. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two quaternion-based transforms are not equal.
    */
   public static void assertQuaternionBasedTransformEqualsSmart(String messagePrefix, QuaternionBasedTransform expected, QuaternionBasedTransform actual,
                                                                double epsilon)
   {
      assertQuaternionBasedTransformEqualsSmart(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two quaternion-based transforms are equal to an
    * {@code epsilon}.
    * <p>
    * This method compares the quaternions using
    * {@link #assertQuaternionEqualsSmart(QuaternionReadOnly, QuaternionReadOnly, double)}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected the expected quaternion-based transform. Not modified.
    * @param actual the actual quaternion-based transform. Not modified.
    * @param epsilon the tolerance to use.
    * @param format the format to use for printing each component when an {@code AssertionError} is
    *           thrown.
    * @throws AssertionError if the two quaternion-based transforms are not equal.
    */
   public static void assertQuaternionBasedTransformEqualsSmart(String messagePrefix, QuaternionBasedTransform expected, QuaternionBasedTransform actual,
                                                                double epsilon, String format)
   {
      try
      {
         assertTuple3DEquals(expected.getTranslationVector(), actual.getTranslationVector(), epsilon);
         assertQuaternionEqualsSmart(expected.getQuaternion(), actual.getQuaternion(), epsilon);
      }
      catch (AssertionError e)
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts on a per component basis that the two given affine transforms are equal to an
    * {@code epsilon}.
    *
    * @param expected the expected affine transform. Not modified.
    * @param actual the actual affine transform. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two affine transforms are not equal.
    */
   public static void assertAffineTransformEquals(AffineTransform expected, AffineTransform actual, double epsilon)
   {
      assertAffineTransformEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two given affine transforms are equal to an
    * {@code epsilon}.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected the expected affine transform. Not modified.
    * @param actual the actual affine transform. Not modified.
    * @param epsilon the tolerance to use.
    * @param format the format to use for printing each component when an {@code AssertionError} is
    *           thrown.
    * @throws AssertionError if the two affine transforms are not equal.
    */
   public static void assertAffineTransformEquals(String messagePrefix, AffineTransform expected, AffineTransform actual, double epsilon)
   {
      assertAffineTransformEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two given affine transforms are equal to an
    * {@code epsilon}.
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected the expected affine transform. Not modified.
    * @param actual the actual affine transform. Not modified.
    * @param epsilon the tolerance to use.
    * @param format the format to use for printing each component when an {@code AssertionError} is
    *           thrown.
    * @throws AssertionError if the two affine transforms are not equal.
    */
   public static void assertAffineTransformEquals(String messagePrefix, AffineTransform expected, AffineTransform actual, double epsilon, String format)
   {
      if (!expected.epsilonEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Tuple2DReadOnly expected, Tuple2DReadOnly actual, double difference, String format)
   {
      String expectedAsString = getTuple2DString(format, expected);
      String actualAsString = getTuple2DString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString, Double.toString(difference));
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Tuple3DReadOnly expected, Tuple3DReadOnly actual, String format)
   {
      String expectedAsString = getTuple3DString(format, expected);
      String actualAsString = getTuple3DString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Tuple3DReadOnly expected, Tuple3DReadOnly actual, double difference, String format)
   {
      String expectedAsString = getTuple3DString(format, expected);
      String actualAsString = getTuple3DString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString, Double.toString(difference));
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Tuple4DReadOnly expected, Tuple4DReadOnly actual, double difference, String format)
   {
      String expectedAsString = getTuple4DString(format, expected);
      String actualAsString = getTuple4DString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString, Double.toString(difference));
   }

   private static void throwNotEqualAssertionError(String messagePrefix, AxisAngleReadOnly expected, AxisAngleReadOnly actual, String format)
   {
      String expectedAsString = getAxisAngleString(format, expected);
      String actualAsString = getAxisAngleString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Matrix3DReadOnly expected, Matrix3DReadOnly actual, String format)
   {
      String expectedAsString = getMatrixString(format, expected);
      String actualAsString = getMatrixString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, AffineTransform expected, AffineTransform actual, String format)
   {
      String expectedAsString = getAffineTransformString(format, expected);
      String actualAsString = getAffineTransformString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, RigidBodyTransform expected, RigidBodyTransform actual, String format)
   {
      String expectedAsString = getRigidBodyTransformString(format, expected);
      String actualAsString = getRigidBodyTransformString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, QuaternionBasedTransform expected, QuaternionBasedTransform actual, String format)
   {
      String expectedAsString = getQuaternionBasedTransformString(format, expected);
      String actualAsString = getQuaternionBasedTransformString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, String expectedAsString, String actualAsString)
   {
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString, null);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, String expectedAsString, String actualAsString, String differenceAsString)
   {
      String errorMessage = addPrefixToMessage(messagePrefix, "expected:\n" + expectedAsString + "\n but was:\n" + actualAsString);
      if (differenceAsString != null)
         errorMessage += "\nDifference of: " + differenceAsString;

      throw new AssertionError(errorMessage);
   }

   private static String addPrefixToMessage(String prefix, String message)
   {
      if (prefix != null && !prefix.isEmpty())
         return prefix + " " + message;
      else
         return message;
   }
}
