package us.ihmc.euclid.orientation.interfaces;

import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public abstract class Orientation3DBasicsTest
{
   public static final int ITERATIONS = 1000;

   public abstract Orientation3DBasics createEmptyOrientation3DBasics();

   public abstract double getEpsilon();

   @Test
   public void testAppend() throws Exception
   {
      Random random = new Random(54);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against Quaternion multiplication
         Quaternion expected = EuclidCoreRandomTools.nextQuaternion(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         QuaternionReadOnly appended = EuclidCoreRandomTools.nextQuaternion(random);

         expected.multiply(appended);
         actualOrientation.append(appended);

         Quaternion actual = new Quaternion();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against AxisAngle multiplication
         AxisAngle expected = EuclidCoreRandomTools.nextAxisAngle(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         AxisAngleReadOnly appended = EuclidCoreRandomTools.nextAxisAngle(random);

         expected.multiply(appended);
         actualOrientation.append(appended);

         AxisAngle actual = new AxisAngle();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertAxisAngleGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against RotationMatrix multiplication
         RotationMatrix expected = EuclidCoreRandomTools.nextRotationMatrix(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         RotationMatrixReadOnly appended = EuclidCoreRandomTools.nextRotationMatrix(random);

         expected.multiply(appended);
         actualOrientation.append(appended);

         RotationMatrix actual = new RotationMatrix();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertRotationMatrixGeometricallyEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testAppendInvertThis() throws Exception
   {
      Random random = new Random(54);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against Quaternion multiplication
         Quaternion expected = EuclidCoreRandomTools.nextQuaternion(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         QuaternionReadOnly appended = EuclidCoreRandomTools.nextQuaternion(random);

         expected.multiplyConjugateThis(appended);
         actualOrientation.appendInvertThis(appended);

         Quaternion actual = new Quaternion();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against AxisAngle multiplication
         AxisAngle expected = EuclidCoreRandomTools.nextAxisAngle(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         AxisAngleReadOnly appended = EuclidCoreRandomTools.nextAxisAngle(random);

         expected.multiplyInvertThis(appended);
         actualOrientation.appendInvertThis(appended);

         AxisAngle actual = new AxisAngle();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertAxisAngleGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against RotationMatrix multiplication
         RotationMatrix expected = EuclidCoreRandomTools.nextRotationMatrix(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         RotationMatrixReadOnly appended = EuclidCoreRandomTools.nextRotationMatrix(random);

         expected.multiplyTransposeThis(appended);
         actualOrientation.appendInvertThis(appended);

         RotationMatrix actual = new RotationMatrix();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertRotationMatrixGeometricallyEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testAppendInvertOther() throws Exception
   {
      Random random = new Random(54);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against Quaternion multiplication
         Quaternion expected = EuclidCoreRandomTools.nextQuaternion(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         QuaternionReadOnly appended = EuclidCoreRandomTools.nextQuaternion(random);

         expected.multiplyConjugateOther(appended);
         actualOrientation.appendInvertOther(appended);

         Quaternion actual = new Quaternion();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against AxisAngle multiplication
         AxisAngle expected = EuclidCoreRandomTools.nextAxisAngle(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         AxisAngleReadOnly appended = EuclidCoreRandomTools.nextAxisAngle(random);

         expected.multiplyInvertOther(appended);
         actualOrientation.appendInvertOther(appended);

         AxisAngle actual = new AxisAngle();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertAxisAngleGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against RotationMatrix multiplication
         RotationMatrix expected = EuclidCoreRandomTools.nextRotationMatrix(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         RotationMatrixReadOnly appended = EuclidCoreRandomTools.nextRotationMatrix(random);

         expected.multiplyTransposeOther(appended);
         actualOrientation.appendInvertOther(appended);

         RotationMatrix actual = new RotationMatrix();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertRotationMatrixGeometricallyEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testAppendInvertBoth() throws Exception
   {
      Random random = new Random(54);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against Quaternion multiplication
         Quaternion expected = EuclidCoreRandomTools.nextQuaternion(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         QuaternionReadOnly appended = EuclidCoreRandomTools.nextQuaternion(random);

         expected.multiplyConjugateBoth(appended);
         actualOrientation.appendInvertBoth(appended);

         Quaternion actual = new Quaternion();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against AxisAngle multiplication
         AxisAngle expected = EuclidCoreRandomTools.nextAxisAngle(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         AxisAngleReadOnly appended = EuclidCoreRandomTools.nextAxisAngle(random);

         expected.multiplyInvertBoth(appended);
         actualOrientation.appendInvertBoth(appended);

         AxisAngle actual = new AxisAngle();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertAxisAngleGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against RotationMatrix multiplication
         RotationMatrix expected = EuclidCoreRandomTools.nextRotationMatrix(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         RotationMatrixReadOnly appended = EuclidCoreRandomTools.nextRotationMatrix(random);

         expected.multiplyTransposeBoth(appended);
         actualOrientation.appendInvertBoth(appended);

         RotationMatrix actual = new RotationMatrix();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertRotationMatrixGeometricallyEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testPrepend() throws Exception
   {
      Random random = new Random(54);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against Quaternion multiplication
         Quaternion expected = EuclidCoreRandomTools.nextQuaternion(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         QuaternionReadOnly prepended = EuclidCoreRandomTools.nextQuaternion(random);

         expected.preMultiply(prepended);
         actualOrientation.prepend(prepended);

         Quaternion actual = new Quaternion();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against AxisAngle multiplication
         AxisAngle expected = EuclidCoreRandomTools.nextAxisAngle(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         AxisAngleReadOnly prepended = EuclidCoreRandomTools.nextAxisAngle(random);

         expected.preMultiply(prepended);
         actualOrientation.prepend(prepended);

         AxisAngle actual = new AxisAngle();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertAxisAngleGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against RotationMatrix multiplication
         RotationMatrix expected = EuclidCoreRandomTools.nextRotationMatrix(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         RotationMatrixReadOnly prepended = EuclidCoreRandomTools.nextRotationMatrix(random);

         expected.preMultiply(prepended);
         actualOrientation.prepend(prepended);

         RotationMatrix actual = new RotationMatrix();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertRotationMatrixGeometricallyEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testPrependInvertThis() throws Exception
   {
      Random random = new Random(54);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against Quaternion multiplication
         Quaternion expected = EuclidCoreRandomTools.nextQuaternion(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         QuaternionReadOnly prepended = EuclidCoreRandomTools.nextQuaternion(random);

         expected.preMultiplyConjugateThis(prepended);
         actualOrientation.prependInvertThis(prepended);

         Quaternion actual = new Quaternion();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against AxisAngle multiplication
         AxisAngle expected = EuclidCoreRandomTools.nextAxisAngle(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         AxisAngleReadOnly prepended = EuclidCoreRandomTools.nextAxisAngle(random);

         expected.preMultiplyInvertThis(prepended);
         actualOrientation.prependInvertThis(prepended);

         AxisAngle actual = new AxisAngle();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertAxisAngleGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against RotationMatrix multiplication
         RotationMatrix expected = EuclidCoreRandomTools.nextRotationMatrix(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         RotationMatrixReadOnly prepended = EuclidCoreRandomTools.nextRotationMatrix(random);

         expected.preMultiplyTransposeThis(prepended);
         actualOrientation.prependInvertThis(prepended);

         RotationMatrix actual = new RotationMatrix();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertRotationMatrixGeometricallyEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testPrependInvertOther() throws Exception
   {
      Random random = new Random(54);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against Quaternion multiplication
         Quaternion expected = EuclidCoreRandomTools.nextQuaternion(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         QuaternionReadOnly prepended = EuclidCoreRandomTools.nextQuaternion(random);

         expected.preMultiplyConjugateOther(prepended);
         actualOrientation.prependInvertOther(prepended);

         Quaternion actual = new Quaternion();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against AxisAngle multiplication
         AxisAngle expected = EuclidCoreRandomTools.nextAxisAngle(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         AxisAngleReadOnly prepended = EuclidCoreRandomTools.nextAxisAngle(random);

         expected.preMultiplyInvertOther(prepended);
         actualOrientation.prependInvertOther(prepended);

         AxisAngle actual = new AxisAngle();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertAxisAngleGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against RotationMatrix multiplication
         RotationMatrix expected = EuclidCoreRandomTools.nextRotationMatrix(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         RotationMatrixReadOnly prepended = EuclidCoreRandomTools.nextRotationMatrix(random);

         expected.preMultiplyTransposeOther(prepended);
         actualOrientation.prependInvertOther(prepended);

         RotationMatrix actual = new RotationMatrix();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertRotationMatrixGeometricallyEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testPrependInvertBoth() throws Exception
   {
      Random random = new Random(54);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against Quaternion multiplication
         Quaternion expected = EuclidCoreRandomTools.nextQuaternion(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         QuaternionReadOnly prepended = EuclidCoreRandomTools.nextQuaternion(random);

         expected.preMultiplyConjugateBoth(prepended);
         actualOrientation.prependInvertBoth(prepended);

         Quaternion actual = new Quaternion();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against AxisAngle multiplication
         AxisAngle expected = EuclidCoreRandomTools.nextAxisAngle(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         AxisAngleReadOnly prepended = EuclidCoreRandomTools.nextAxisAngle(random);

         expected.preMultiplyInvertBoth(prepended);
         actualOrientation.prependInvertBoth(prepended);

         AxisAngle actual = new AxisAngle();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertAxisAngleGeometricallyEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against RotationMatrix multiplication
         RotationMatrix expected = EuclidCoreRandomTools.nextRotationMatrix(random);
         Orientation3DBasics actualOrientation = createEmptyOrientation3DBasics();
         actualOrientation.set(expected);

         RotationMatrixReadOnly prepended = EuclidCoreRandomTools.nextRotationMatrix(random);

         expected.preMultiplyTransposeBoth(prepended);
         actualOrientation.prependInvertBoth(prepended);

         RotationMatrix actual = new RotationMatrix();
         actual.set(actualOrientation);
         EuclidCoreTestTools.assertRotationMatrixGeometricallyEquals(expected, actual, getEpsilon());
      }
   }

}
