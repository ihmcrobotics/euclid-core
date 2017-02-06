package us.ihmc.geometry.testingTools;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Arrays;

import us.ihmc.geometry.axisAngle.AxisAngle;
import us.ihmc.geometry.axisAngle.AxisAngleTools;
import us.ihmc.geometry.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.geometry.transform.AffineTransform;
import us.ihmc.geometry.transform.QuaternionBasedTransform;
import us.ihmc.geometry.transform.RigidBodyTransform;
import us.ihmc.geometry.tuple.TupleTools;
import us.ihmc.geometry.tuple.Vector;
import us.ihmc.geometry.tuple.interfaces.Tuple3DReadOnly;
import us.ihmc.geometry.tuple.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.tuple2D.Vector2D;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple4D.Quaternion;
import us.ihmc.geometry.tuple4D.Tuple4DTools;
import us.ihmc.geometry.tuple4D.Vector4D;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Tuple4DReadOnly;

public abstract class GeometryBasicsTestTools
{
   /**
    * Asserts that the two given angles are equal to an {@code epsilon}.
    * <p>
    * The method returns {@code true} for angles such as: {@code actualAngle = expectedAngle +/- 2.0 * Math.PI}.
    * </p>
    * 
    * @param expectedAngle the expected angle.
    * @param actualAngle the actual angle.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two angles are not equal.
    */
   public static void assertAngleEquals(double expectedAngle, double actualAngle, double epsilon)
   {
      double differenceAngle = Math.abs(expectedAngle - actualAngle);
      differenceAngle = ((differenceAngle + Math.PI) % (2.0 * Math.PI)) - Math.PI;
      try
      {
         assertEquals(0.0, differenceAngle, epsilon);
      }
      catch (AssertionError e)
      {
         throw new AssertionError("expected:<" + expectedAngle + "> but was:<" + actualAngle + ">");
      }
   }

   /**
    * Asserts on a per component basis that the two sets of yaw-pitch-roll angles are equal to an {@code epsilon}.
    * <p>
    * The method returns {@code true} for angles such as: {@code actualAngle = expectedAngle +/- 2.0 * Math.PI}.
    * </p>
    * 
    * @param expectedYawPitchRoll the expected set of yaw-pitch-roll angles. Not modified.
    * @param actualYawPitchRoll the actual set of yaw-pitch-roll angles. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two sets of yaw-pitch-roll angles are not equal.
    */
   public static void assertYawPitchRollEquals(double[] expectedYawPitchRoll, double[] actualYawPitchRoll, double epsilon)
   {
      try
      {
         assertAngleEquals(expectedYawPitchRoll[0], actualYawPitchRoll[0], epsilon);
         assertAngleEquals(expectedYawPitchRoll[1], actualYawPitchRoll[1], epsilon);
         assertAngleEquals(expectedYawPitchRoll[2], actualYawPitchRoll[2], epsilon);
      }
      catch (AssertionError e)
      {
         throw new AssertionError("expected:<" + Arrays.toString(expectedYawPitchRoll) + "> but was:<" + Arrays.toString(actualYawPitchRoll) + ">");
      }
   }

   /**
    * Asserts on a per component basis that the two rotation vectors are equal to an {@code epsilon}.
    * <p>
    * The method returns {@code true} for angles such as: {@code actualAngle = expectedAngle +/- 2.0 * Math.PI}.
    * </p>
    * 
    * @param expectedRotationVector the expected rotation vector. Not modified.
    * @param actualRotationVector the actual rotation vector. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two rotation vectors are not equal.
    */
   public static void assertRotationVectorEquals(Vector3DReadOnly expectedRotationVector, Vector3DReadOnly actualRotationVector, double epsilon)
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
         throw new AssertionError("expected:" + expectedRotationVector + " but was:" + actualRotationVector);
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
   public static void assertTupleEquals(Tuple3DReadOnly expected, Tuple3DReadOnly actual, double epsilon)
   {
      assertTupleEquals("", expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two tuples are equal to an {@code epsilon}.
    * 
    * @param message prefix to add to the automated message.
    * @param expected the expected tuple.
    * @param actual the actual tuple.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two tuples are not equal.
    */
   public static void assertTupleEquals(String message, Tuple3DReadOnly expected, Tuple3DReadOnly actual, double epsilon)
   {
      boolean areEqual = TupleTools.epsilonEquals(expected, actual, epsilon);

      if (message.equals(""))
      {
         message = expected.getClass().getSimpleName() + " is not equal to " + actual.getClass().getSimpleName() + "!";
      }

      if (!areEqual)
      {
         Vector difference = new Vector(actual);
         difference.sub(expected);
         fail(message + " Expected = " + expected + ", actual = " + actual + ". Difference magnitude = " + difference.length());
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
      assertTuple2DEquals("", expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two tuples are equal to an {@code epsilon}.
    * 
    * @param message prefix to add to the automated message.
    * @param expected the expected tuple.
    * @param actual the actual tuple.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two tuples are not equal.
    */
   public static void assertTuple2DEquals(String message, Tuple2DReadOnly expected, Tuple2DReadOnly actual, double epsilon)
   {
      boolean areEqual = TupleTools.epsilonEquals(expected, actual, epsilon);

      if (message.equals(""))
      {
         message = expected.getClass().getSimpleName() + " is not equal to " + actual.getClass().getSimpleName() + "!";
      }

      if (!areEqual)
      {
         Vector2D difference = new Vector2D(actual);
         difference.sub(expected);
         fail(message + " Expected = " + expected + ", actual = " + actual + ". Difference magnitude = " + difference.length());
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
      assertTuple4DEquals("", expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two tuples are equal to an {@code epsilon}.
    * 
    * @param message prefix to add to the automated message.
    * @param expected the expected tuple.
    * @param actual the actual tuple.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two tuples are not equal.
    */
   public static void assertTuple4DEquals(String message, Tuple4DReadOnly expected, Tuple4DReadOnly actual, double epsilon)
   {
      boolean areEqual = Tuple4DTools.epsilonEquals(expected, actual, epsilon);

      if (message.equals(""))
      {
         message = expected.getClass().getSimpleName() + " is not equal to " + actual.getClass().getSimpleName() + "!";
      }

      if (!areEqual)
      {
         Vector4D difference = new Vector4D(actual);
         difference.sub(expected);
         fail(message + " Expected = " + expected + ", actual = " + actual + ". Difference magnitude = " + difference.length());
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
   public static void assertMatrix3DEquals(Matrix3DReadOnly<?> expected, Matrix3DReadOnly<?> actual, double epsilon)
   {
      assertMatrix3DEquals("", expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two matrices are equal to an {@code epsilon}.
    * 
    * @param message prefix to add to the automated message.
    * @param expected the expected matrix.
    * @param actual the actual matrix.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two matrices are not equal.
    */
   public static <T extends Matrix3DReadOnly<T>> void assertMatrix3DEquals(String message, Matrix3DReadOnly<?> expected, Matrix3DReadOnly<?> actual,
                                                                           double epsilon)
   {
      if (!expected.epsilonEquals(actual, epsilon))
      {
         fail(message + " Expected =\n" + expected + "\nActual =\n" + actual);
      }
   }

   /**
    * Asserts that the given matrix is skew-symmetric:
    * <pre>
    *     |  0 -z  y |
    * m = |  z  0 -x |
    *     | -y  x  0 |
    * </pre>
    * <p>
    * This matrix is considered to be skew symmetric if:
    * <ul>
    *    <li> each diagonal coefficient is equal to 0.0 +/- {@code epsilon},
    *    <li> the sums of each pair of cross-diagonal coefficients
    *     ({@code m10}, {@code m01}), ({@code m12}, {@code m21}), and ({@code m20}, {@code m02})
    *     are equal to 0.0 +/- {@code epsilon}.
    * </ul>
    * </p>
    *
    * @param matrix the query. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the matrix is not skew-symmetric.
    */
   public static void assertSkewSymmetric(Matrix3DReadOnly<?> matrix, double epsilon)
   {
      if (!matrix.isMatrixSkewSymmetric(epsilon))
         fail("The matrix is not skew-symmetric:\n" + matrix);
   }

   /**
    * Asserts that this matrix is a rotation matrix.
    * <p>
    * This matrix is a rotation matrix if:
    * <ul>
    *    <li> the length of each row vector is equal to 1.0 +/- {@code epsilon},
    *    <li> the dot product of each pair of row vectors is equal to 0.0 +/- {@code epsilon},
    *    <li> the determinant of the matrix is equal to 1.0 +/- {@code epsilon}.
    * </ul>
    * </p>
    * 
    * @param matrix the query. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the matrix is not a rotation matrix.
    */
   public static void assertRotationMatrix(Matrix3DReadOnly<?> matrix, double epsilon)
   {
      if (!matrix.isRotationMatrix(epsilon))
         fail("This is not a rotation matrix:\n" + matrix);
   }

   /**
    * Asserts on a per coefficient basis that this matrix is equal to identity to an {@code epsilon}.
    * 
    * @param matrix the query. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the matrix is not identity.
    */
   public static void assertIdentity(Matrix3DReadOnly<?> matrix, double epsilon)
   {
      if (!matrix.isIdentity(epsilon))
         fail("The matrix is not identity:\n" + matrix);
   }

   /**
    * Asserts that the given matrix contains on {@link Double#NaN}.
    * 
    * @param matrix the query. Not modified.
    * @throws AssertionError if the matrix does not only contain {@link Double#NaN}.
    */
   public static void assertMatrix3DContainsOnlyNaN(Matrix3DReadOnly<?> matrix)
   {
      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            if (!Double.isNaN(matrix.getElement(row, column)))
               fail("The matrix does not contain only NaN:\n" + matrix);
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
      try
      {
         assertEquals(expectedQuaternion.getS(), actualQuaternion.getS(), epsilon);
         assertEquals(expectedQuaternion.getX(), actualQuaternion.getX(), epsilon);
         assertEquals(expectedQuaternion.getY(), actualQuaternion.getY(), epsilon);
         assertEquals(expectedQuaternion.getZ(), actualQuaternion.getZ(), epsilon);
      }
      catch (AssertionError e)
      {
         throw new AssertionError("expected:\n<" + expectedQuaternion + ">\n but was:\n<" + actualQuaternion + ">");
      }
   }

   /**
    * Asserts on a per component basis that the two quaternions are equal to an {@code epsilon}.
    * <p>
    * This method changes the sign of one of the two quaternions when comparing if their dot product is negative.
    * </p>
    * 
    * @param expectedQuaternion the expected quaternion. Not modified.
    * @param actualQuaternion the actual quaternion. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two quaternions are not equal.
    */
   public static void assertQuaternionEqualsSmart(QuaternionReadOnly expectedQuaternion, QuaternionReadOnly actualQuaternion, double epsilon)
   {
      try
      {
         double sign = Math.signum(Tuple4DTools.dot(expectedQuaternion, actualQuaternion));
         assertEquals(expectedQuaternion.getS(), sign * actualQuaternion.getS(), epsilon);
         assertEquals(expectedQuaternion.getX(), sign * actualQuaternion.getX(), epsilon);
         assertEquals(expectedQuaternion.getY(), sign * actualQuaternion.getY(), epsilon);
         assertEquals(expectedQuaternion.getZ(), sign * actualQuaternion.getZ(), epsilon);
      }
      catch (AssertionError e)
      {
         throw new AssertionError("expected:\n<" + expectedQuaternion + ">\n but was:\n<" + actualQuaternion + ">");
      }
   }

   /**
    * Asserts that the two given quaternions represents the same orientation to an {@code epsilon} by calculating the magnitude of their difference.
    * 
    * @param expectedQuaternion the expected quaternion. Not modified.
    * @param actualQuaternion the actual quaternion. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two quaternions do not represent the same orientation.
    */
   public static void assertQuaternionEqualsUsingDifference(QuaternionReadOnly expectedQuaternion, QuaternionReadOnly actualQuaternion, double epsilon)
   {
      try
      {
         Quaternion qDifference = new Quaternion(expectedQuaternion);
         qDifference.multiplyConjugateOther(actualQuaternion);
         AxisAngle axisAngle = new AxisAngle();
         axisAngle.set(qDifference);
         assertEquals(0.0, axisAngle.getAngle(), epsilon);
      }
      catch (AssertionError e)
      {
         throw new AssertionError("expected:\n<" + expectedQuaternion + ">\n but was:\n<" + actualQuaternion + ">");
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
   public static void assertAxisAngleEquals(AxisAngleReadOnly<?> expectedAxisAngle, AxisAngleReadOnly<?> actualAxisAngle, double epsilon)
   {
      try
      {
         assertEquals(expectedAxisAngle.getAngle(), actualAxisAngle.getAngle(), epsilon);
         assertEquals(expectedAxisAngle.getX(), actualAxisAngle.getX(), epsilon);
         assertEquals(expectedAxisAngle.getY(), actualAxisAngle.getY(), epsilon);
         assertEquals(expectedAxisAngle.getZ(), actualAxisAngle.getZ(), epsilon);
      }
      catch (AssertionError e)
      {
         throw new AssertionError("expected:\n<" + expectedAxisAngle + ">\n but was:\n<" + actualAxisAngle + ">");
      }
   }

   /**
    * Asserts on a per component basis if the two axis-angles are equal to an {@code epsilon}.
    * <p>
    * This method changes the sign of one of the two axis-angles when comparing if the dot product of their axis is negative.
    * </p>
    * 
    * @param expectedAxisAngle the expected axis-angle. Not modified.
    * @param actualAxisAngle the actual axis-angle. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two axis-angles are not equal.
    */
   public static void assertAxisAngleEqualsSmart(AxisAngleReadOnly<?> expectedAxisAngle, AxisAngleReadOnly<?> actualAxisAngle, double epsilon)
   {
      try
      {
         double expectedX = expectedAxisAngle.getX();
         double expectedY = expectedAxisAngle.getY();
         double expectedZ = expectedAxisAngle.getZ();
         double expectedAngle = expectedAxisAngle.getAngle();
         double actualX = actualAxisAngle.getX();
         double actualY = actualAxisAngle.getY();
         double actualZ = actualAxisAngle.getZ();
         double actualAngle = actualAxisAngle.getAngle();

         if (expectedX * actualX + expectedY * actualY + expectedZ * actualZ < 0.0)
         {
            actualX *= -1.0;
            actualY *= -1.0;
            actualZ *= -1.0;
            actualAngle *= -1.0;
         }

         assertEquals(expectedX, actualX, epsilon);
         assertEquals(expectedY, actualY, epsilon);
         assertEquals(expectedZ, actualZ, epsilon);
         assertAngleEquals(expectedAngle, actualAngle, epsilon);
      }
      catch (AssertionError e)
      {
         throw new AssertionError("expected:\n<" + expectedAxisAngle + ">\n but was:\n<" + actualAxisAngle + ">");
      }
   }

   /**
    * Asserts that the given axis-angle contains only {@link Double#NaN}.
    * 
    * @param axisAngleToAssert the query. Not modified.
    * @throws AssertionError if the axis-angle does not only contain {@link Double#NaN}.
    */
   public static void assertAxisAngleContainsOnlyNaN(AxisAngleReadOnly<?> axisAngleToAssert)
   {
      assertTrue(Double.isNaN(axisAngleToAssert.getX()));
      assertTrue(Double.isNaN(axisAngleToAssert.getY()));
      assertTrue(Double.isNaN(axisAngleToAssert.getZ()));
      assertTrue(Double.isNaN(axisAngleToAssert.getAngle()));
   }

   /**
    * Assert that {@link AxisAngleBasics#setToZero()} has just been called on the given axis-angle.
    * 
    * @param axisAngleToAssert the query. Not modified.
    * @throws AssertionError if the axis-angle has not been set to zero.
    */
   public static void assertAxisAngleIsSetToZero(AxisAngleReadOnly<?> axisAngleToAssert)
   {
      assertTrue(axisAngleToAssert.getX() == 1.0);
      assertTrue(axisAngleToAssert.getY() == 0.0);
      assertTrue(axisAngleToAssert.getZ() == 0.0);
      assertTrue(axisAngleToAssert.getAngle() == 0.0);
   }

   /**
    * Asserts that the length of the axis of the axis-angle is equal to {@code 1.0 +/- epsilon}. 
    * 
    * @param axisAngleToAssert the query. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the axis is not unitary.
    */
   public static void assertAxisUnitary(AxisAngleReadOnly<?> axisAngleToAssert, double epsilon)
   {
      if (!AxisAngleTools.isAxisUnitary(axisAngleToAssert, epsilon))
         fail("The axis of the given AxisAngle is not unitary: " + axisAngleToAssert);
   }

   /**
    * Asserts that the given quaternion contains only {@link Double#NaN}.
    * 
    * @param quaternionToAssert the query. Not modified.
    * @throws AssertionError if the quaternion does not only contain {@link Double#NaN}.
    */
   public static void assertQuaternionContainsOnlyNaN(QuaternionReadOnly quaternionToAssert)
   {
      assertTrue(Double.isNaN(quaternionToAssert.getX()));
      assertTrue(Double.isNaN(quaternionToAssert.getY()));
      assertTrue(Double.isNaN(quaternionToAssert.getZ()));
      assertTrue(Double.isNaN(quaternionToAssert.getS()));
   }

   /**
    * Assert that {@link QuaternionBasics#setToZero()} has just been called on the given quaternion.
    * 
    * @param quaternionToAssert the query. Not modified.
    * @throws AssertionError if the quaternion has not been set to zero.
    */
   public static void assertQuaternionIsSetToZero(QuaternionReadOnly quaternionToAssert)
   {
      assertTrue(quaternionToAssert.getX() == 0.0);
      assertTrue(quaternionToAssert.getY() == 0.0);
      assertTrue(quaternionToAssert.getZ() == 0.0);
      assertTrue(quaternionToAssert.getS() == 1.0);
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
      if (!Tuple4DTools.isUnitary(quaternionToAssert, epsilon))
         fail("The axis of the given Quaternion is not unitary: " + quaternionToAssert);
   }

   /**
    * Asserts that the given tuple contains only {@link Double#NaN}.
    * 
    * @param tupleToAssert the query. Not modified.
    * @throws AssertionError if the tuple does not only contain {@link Double#NaN}.
    */
   public static void assertTupleContainsOnlyNaN(Tuple3DReadOnly tupleToAssert)
   {
      assertTrue(Double.isNaN(tupleToAssert.getX()));
      assertTrue(Double.isNaN(tupleToAssert.getY()));
      assertTrue(Double.isNaN(tupleToAssert.getZ()));
   }

   /**
    * Assert that {@link TupleBasics#setToZero()} has just been called on the given tuple.
    * 
    * @param tupleToAssert the query. Not modified.
    * @throws AssertionError if the tuple has not been set to zero.
    */
   public static void assertTupleIsSetToZero(Tuple3DReadOnly tupleToAssert)
   {
      assertTrue(tupleToAssert.getX() == 0.0);
      assertTrue(tupleToAssert.getY() == 0.0);
      assertTrue(tupleToAssert.getZ() == 0.0);
   }

   /**
    * Asserts on a per component basis that the two given rigid-body transform are equal to an {@code epsilon}.
    * 
    * @param expected the expected rigid-body transform. Not modified.
    * @param actual the actual rigid-body transform. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two rigid-body transforms are not equal.
    */
   public static void assertRigidBodyTransformEquals(RigidBodyTransform expected, RigidBodyTransform actual, double epsilon)
   {
      if (!expected.epsilonEquals(actual, epsilon))
      {
         throw new AssertionError("expected:\n<" + expected + ">\n but was:\n<" + actual + ">");
      }
   }

   /**
    * Asserts on a per component basis that the two quaternion-based transforms are equal to an {@code epsilon}.
    * 
    * @param expected the expected quaternion-based transform. Not modified.
    * @param actual the actual quaternion-based transform. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two quaternion-based transforms are not equal.
    */
   public static void assertQuaternionBasedTransformEquals(QuaternionBasedTransform expected, QuaternionBasedTransform actual, double epsilon)
   {
      if (!expected.epsilonEquals(actual, epsilon))
      {
         throw new AssertionError("expected:\n<" + expected + ">\n but was:\n<" + actual + ">");
      }
   }

   /**
    * Asserts on a per component basis that the two quaternion-based transforms are equal to an {@code epsilon}.
    * <p>
    * This method compares the quaternions using {@link #assertQuaternionEqualsSmart(QuaternionReadOnly, QuaternionReadOnly, double)}.
    * </p>
    * 
    * @param expected the expected quaternion-based transform. Not modified.
    * @param actual the actual quaternion-based transform. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two quaternion-based transforms are not equal.
    */
   public static void assertQuaternionBasedTransformEqualsSmart(QuaternionBasedTransform expected, QuaternionBasedTransform actual, double epsilon)
   {
      try
      {
         assertTupleEquals(expected.getTranslationVector(), actual.getTranslationVector(), epsilon);
         assertQuaternionEqualsSmart(expected.getQuaternion(), actual.getQuaternion(), epsilon);
      }
      catch (AssertionError e)
      {
         throw new AssertionError("expected:\n<" + expected + ">\n but was:\n<" + actual + ">");
      }
   }

   /**
    * Asserts on a per component basis that the two given affine transforms are equal to an {@code epsilon}.
    * 
    * @param expected the expected affine transform. Not modified.
    * @param actual the actual affine transform. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two affine transforms are not equal.
    */
   public static void assertAffineTransformEquals(AffineTransform expected, AffineTransform actual, double epsilon)
   {
      if (!expected.epsilonEquals(actual, epsilon))
      {
         throw new AssertionError("expected:\n<" + expected + ">\n but was:\n<" + actual + ">");
      }
   }
}
