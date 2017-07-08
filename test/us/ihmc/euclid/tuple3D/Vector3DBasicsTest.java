package us.ihmc.euclid.tuple3D;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public abstract class Vector3DBasicsTest<T extends Vector3DBasics> extends Tuple3DBasicsTest<T>
{
   // Read-only part
   @Test
   public void testLength()
   {
      Random random = new Random(312310L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         T vector1 = createRandomTuple(random);
         double length1 = vector1.length();
         T vector2 = createEmptyTuple();
         double scalar = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);
         vector2.setAndScale(scalar, vector1);
         double expectedLength2 = scalar * length1;
         double actualLength2 = vector2.length();
         assertEquals(expectedLength2, actualLength2, 5.0 * getEpsilon());
      }
   }

   @Test
   public void testLengthSquared()
   {
      Random random = new Random(312310L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         T vector1 = createRandomTuple(random);
         double length1 = vector1.length();
         T vector2 = createEmptyTuple();
         double scalar = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);
         vector2.setAndScale(scalar, vector1);
         double expectedLength2 = scalar * length1;
         double actualLength2 = vector2.lengthSquared();
         assertEquals(expectedLength2, Math.sqrt(actualLength2), 5.0 * getEpsilon());
      }
   }

   @Test
   public void testDot()
   {
      Random random = new Random(5461L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         T vector1 = createRandomTuple(random);
         vector1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 2.0));
         Vector3DBasics axis = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, vector1, true);
         double angle = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);

         T vector2 = createEmptyTuple();
         RotationMatrix rotationMatrix = new RotationMatrix(new AxisAngle(axis, angle));
         rotationMatrix.transform(vector1, vector2);
         vector2.scale(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 2.0));

         double expectedDot = vector1.length() * vector2.length() * Math.cos(angle);
         double actualDot = vector1.dot(vector2);
         assertEquals(expectedDot, actualDot, 10.0 * getEpsilon());
      }
   }

   @Test
   public void testAngle()
   {
      Random random = new Random(56461L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         T vector1 = createRandomTuple(random);
         vector1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 2.0));
         Vector3DBasics axis = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, vector1, true);
         double expectedAngle = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);

         T vector2 = createEmptyTuple();
         RotationMatrix rotationMatrix = new RotationMatrix(new AxisAngle(axis, expectedAngle));
         rotationMatrix.transform(vector1, vector2);
         vector2.scale(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 2.0));

         double actualAngle = vector1.angle(vector2);
         assertEquals(Math.abs(expectedAngle), actualAngle, 20.0 * getEpsilon());
      }
   }

   @Test
   public void testCross()
   {
      Random random = new Random(56461L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // cross(Vector3DReadOnly other)
         T vector1 = createRandomTuple(random);
         vector1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 2.0));
         Vector3DBasics axis = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, vector1, true);
         double angle = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, Math.PI);

         T vector2 = createEmptyTuple();
         RotationMatrix rotationMatrix = new RotationMatrix(new AxisAngle(axis, angle));
         rotationMatrix.transform(vector1, vector2);
         vector2.scale(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 2.0));

         T vector3 = createEmptyTuple();
         vector3.cross(vector1, vector2);

         double expectedCrossMagnitude = vector1.length() * vector2.length() * Math.sin(angle);
         double actualCrossMagnitude = vector3.length();
         assertEquals(expectedCrossMagnitude, actualCrossMagnitude, 10.0 * getEpsilon());

         assertEquals(0.0, vector1.dot(vector3), 10.0 * getEpsilon());
         assertEquals(0.0, vector2.dot(vector3), 10.0 * getEpsilon());
      }
   }

   @Test
   public void testClipToMaxLength() throws Exception
   {
      Random random = new Random(234234);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test with maxLength > EPS_MAX_LENGTH
         double maxLength = EuclidCoreRandomTools.generateRandomDouble(random, Vector3DBasics.EPS_MAX_LENGTH, 10.0);
         double vectorLength = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);
         T expectedVector = createTuple(1.0, 0.0, 0.0);
         RotationMatrix rotationMatrix = EuclidCoreRandomTools.generateRandomRotationMatrix(random);
         rotationMatrix.transform(expectedVector, expectedVector);
         T actualVector = createEmptyTuple();
         actualVector.setAndScale(vectorLength, expectedVector);

         if (maxLength > vectorLength)
            expectedVector.scale(vectorLength);
         else
            expectedVector.scale(maxLength);

         actualVector.clipToMaxLength(maxLength);

         EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i + ", maxLength: " + maxLength, expectedVector, actualVector, 5.0 * getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test with maxLength < EPS_MAX_LENGTH
         double maxLength = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, Vector3DBasics.EPS_MAX_LENGTH);
         double vectorLength = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);
         T actualVector = createTuple(vectorLength, 0.0, 0.0);
         RotationMatrix rotationMatrix = EuclidCoreRandomTools.generateRandomRotationMatrix(random);
         rotationMatrix.transform(actualVector, actualVector);
         
         actualVector.clipToMaxLength(maxLength);
         
         EuclidCoreTestTools.assertTuple3DIsSetToZero("Iteration: " + i + ", maxLength: " + maxLength, actualVector);
      }
   }

   // Basics part
   @Test
   public void testNormalize()
   {
      Random random = new Random(312310L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test normalize()
         T vector1 = createRandomTuple(random);
         vector1.normalize();

         double expectedLength = 1.0;
         double actualLength = vector1.length();
         assertEquals(expectedLength, actualLength, getEpsilon());

         T vector2 = createRandomTuple(random);
         vector2.normalize();
         vector1.setAndScale(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), vector2);
         vector1.normalize();
         EuclidCoreTestTools.assertTuple3DEquals(vector1, vector2, getEpsilon());

         vector1.setToNaN();
         vector1.normalize();
         for (int index = 0; index < 3; index++)
            assertTrue(Double.isNaN(vector1.getElement(index)));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test setAndNormalize(Tuple3DReadOnly other)
         T vector1 = createRandomTuple(random);
         T vector2 = createEmptyTuple();

         vector2.setAndNormalize(vector1);

         double expectedLength = 1.0;
         double actualLength = vector2.length();
         assertEquals(expectedLength, actualLength, getEpsilon());

         vector2 = createRandomTuple(random);
         vector2.normalize();
         T vector3 = createEmptyTuple();
         vector3.setAndScale(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), vector2);
         vector1.setAndNormalize(vector3);
         EuclidCoreTestTools.assertTuple3DEquals(vector1, vector2, getEpsilon());

         vector3.setToNaN();
         vector1.setToZero();
         vector1.setAndNormalize(vector3);
         for (int index = 0; index < 3; index++)
            assertTrue(Double.isNaN(vector1.getElement(index)));
      }
   }

   @Test
   public void testApplyTransform() throws Exception
   {
      Random random = new Random(23523L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         transform.transform(expected);
         actual.set(original);
         actual.applyTransform(transform);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         QuaternionBasedTransform transform = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         transform.transform(expected);
         actual.set(original);
         actual.applyTransform(transform);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         AffineTransform transform = EuclidCoreRandomTools.generateRandomAffineTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         transform.transform(expected);
         actual.set(original);
         actual.applyTransform(transform);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, 10.0 * getEpsilon());
      }
   }

   @Test
   public void testApplyInverseTransform() throws Exception
   {
      Random random = new Random(23523L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         actual.set(original);
         actual.applyTransform(transform);
         actual.applyInverseTransform(transform);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, 10.0 * getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         QuaternionBasedTransform transform = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         actual.set(original);
         actual.applyTransform(transform);
         actual.applyInverseTransform(transform);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, 10.0 * getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         AffineTransform transform = EuclidCoreRandomTools.generateRandomAffineTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         actual.set(original);
         actual.applyTransform(transform);
         actual.applyInverseTransform(transform);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, 100.0 * getEpsilon());
      }
   }
}
