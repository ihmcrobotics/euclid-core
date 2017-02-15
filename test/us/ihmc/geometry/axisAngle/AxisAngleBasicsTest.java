package us.ihmc.geometry.axisAngle;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

import us.ihmc.geometry.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.tools.AxisAngleTools;
import us.ihmc.geometry.tools.EuclidCoreRandomTools;
import us.ihmc.geometry.tools.EuclidCoreTestTools;
import us.ihmc.geometry.tools.QuaternionTools;
import us.ihmc.geometry.tuple3D.Vector3D;
import us.ihmc.geometry.tuple4D.Quaternion;

public abstract class AxisAngleBasicsTest<T extends AxisAngleBasics> extends AxisAngleReadOnlyTest<T>
{
   @Test
   public void testSetAngle()
   {
      Random random = new Random(5646541L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         T axisAngle = createAxisAngle(0.0, 0.0, 0.0, 0.0);
         double expectedAngle = random.nextInt(100);
         axisAngle.setAngle(expectedAngle);

         assertTrue(axisAngle.getX() == 0.0);
         assertTrue(axisAngle.getY() == 0.0);
         assertTrue(axisAngle.getZ() == 0.0);
         assertTrue(axisAngle.getAngle() == expectedAngle);
      }
   }

   @Test
   public void testSetX()
   {
      Random random = new Random(5646541L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         T axisAngle = createAxisAngle(0.0, 0.0, 0.0, 0.0);
         double expectedX = random.nextInt(100);
         axisAngle.setX(expectedX);

         assertTrue(axisAngle.getX() == expectedX);
         assertTrue(axisAngle.getY() == 0.0);
         assertTrue(axisAngle.getZ() == 0.0);
         assertTrue(axisAngle.getAngle() == 0.0);
      }
   }

   @Test
   public void testSetY()
   {
      Random random = new Random(5646541L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         T axisAngle = createAxisAngle(0.0, 0.0, 0.0, 0.0);
         double expectedY = random.nextInt(100);
         axisAngle.setY(expectedY);

         assertTrue(axisAngle.getX() == 0.0); // Set to default
         assertTrue(axisAngle.getY() == expectedY);
         assertTrue(axisAngle.getZ() == 0.0);
         assertTrue(axisAngle.getAngle() == 0.0);
      }
   }

   @Test
   public void testSetZ()
   {
      Random random = new Random(5646541L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         T axisAngle = createAxisAngle(0.0, 0.0, 0.0, 0.0);
         double expectedZ = random.nextInt(100);
         axisAngle.setZ(expectedZ);

         assertTrue(axisAngle.getX() == 0.0);
         assertTrue(axisAngle.getY() == 0.0);
         assertTrue(axisAngle.getZ() == expectedZ);
         assertTrue(axisAngle.getAngle() == 0.0);
      }
   }

   @Test
   public void testSetToZero()
   {
      T axisAngle = createAxisAngle(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(axisAngle);
      axisAngle.setToZero();
      assertTrue(axisAngle.getX() == 1.0); // Set to default (x = 1.0, not 0.0)
      assertTrue(axisAngle.getY() == 0.0);
      assertTrue(axisAngle.getZ() == 0.0);
      assertTrue(axisAngle.getAngle() == 0.0);
   }

   @Test
   public void testSetToNaN()
   {
      T axisAngle = createAxisAngle(0.0, 0.0, 0.0, 0.0);
      assertTrue(axisAngle.getX() == 0.0);
      assertTrue(axisAngle.getY() == 0.0);
      assertTrue(axisAngle.getZ() == 0.0);
      assertTrue(axisAngle.getAngle() == 0.0);
      axisAngle.setToNaN();
      EuclidCoreTestTools.assertAxisAngleContainsOnlyNaN(axisAngle);
      assertTrue(Double.isNaN(axisAngle.getX()));
      assertTrue(Double.isNaN(axisAngle.getY()));
      assertTrue(Double.isNaN(axisAngle.getZ()));
      assertTrue(Double.isNaN(axisAngle.getAngle()));
   }

   @Test
   public void testNegate()
   {
      Random random = new Random(5646541L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double x = EuclidCoreRandomTools.generateRandomDouble(random);
         double y = EuclidCoreRandomTools.generateRandomDouble(random);
         double z = EuclidCoreRandomTools.generateRandomDouble(random);
         double angle = EuclidCoreRandomTools.generateRandomDouble(random);
         T expectedAxisAngle = createAxisAngle(-x, -y, -z, -angle);
         T actualAxisAngle = createAxisAngle(x, y, z, angle);
         actualAxisAngle.negate();
         assertTrue(expectedAxisAngle.equals(actualAxisAngle));
      }
   }

   @Test
   public void testAbsolute()
   {
      Random random = new Random(5646541L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double x = EuclidCoreRandomTools.generateRandomDouble(random);
         double y = EuclidCoreRandomTools.generateRandomDouble(random);
         double z = EuclidCoreRandomTools.generateRandomDouble(random);
         double angle = EuclidCoreRandomTools.generateRandomDouble(random);
         T expectedAxisAngle = createAxisAngle(Math.abs(x), Math.abs(y), Math.abs(z), Math.abs(angle));
         T actualAxisAngle = createAxisAngle(x, y, z, angle);
         actualAxisAngle.absolute();
         assertTrue(expectedAxisAngle.equals(actualAxisAngle));
      }
   }

   @Test
   public void testNormalizeAxis() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Vector3D randomAxis = EuclidCoreRandomTools.generateRandomRotationVector(random);
         double randomAngle = EuclidCoreRandomTools.generateRandomDouble(random);

         T actualAxisAngle = createAxisAngle(randomAxis.getX(), randomAxis.getY(), randomAxis.getZ(), randomAngle);
         actualAxisAngle.normalizeAxis();
         randomAxis.normalize();
         T expectedAxisAngle = createAxisAngle(randomAxis.getX(), randomAxis.getY(), randomAxis.getZ(), randomAngle);
         EuclidCoreTestTools.assertAxisAngleEquals(expectedAxisAngle, actualAxisAngle, getEpsilon());
      }

      T axisAngle = createAxisAngle(Double.NaN, 0.0, 0.0, 0.0);
      axisAngle.normalizeAxis();
      assertTrue(Double.isNaN(axisAngle.getX()));
      assertFalse(Double.isNaN(axisAngle.getY()));
      assertFalse(Double.isNaN(axisAngle.getZ()));
      assertFalse(Double.isNaN(axisAngle.getAngle()));
      
      axisAngle = createAxisAngle(0.0, Double.NaN, 0.0, 0.0);
      axisAngle.normalizeAxis();
      assertFalse(Double.isNaN(axisAngle.getX()));
      assertTrue(Double.isNaN(axisAngle.getY()));
      assertFalse(Double.isNaN(axisAngle.getZ()));
      assertFalse(Double.isNaN(axisAngle.getAngle()));
      
      axisAngle = createAxisAngle(0.0, 0.0, Double.NaN, 0.0);
      axisAngle.normalizeAxis();
      assertFalse(Double.isNaN(axisAngle.getX()));
      assertFalse(Double.isNaN(axisAngle.getY()));
      assertTrue(Double.isNaN(axisAngle.getZ()));
      assertFalse(Double.isNaN(axisAngle.getAngle()));

      axisAngle = createAxisAngle(0.0, 0.0, 0.0, Double.NaN);
      axisAngle.normalizeAxis();
      assertFalse(Double.isNaN(axisAngle.getX()));
      assertFalse(Double.isNaN(axisAngle.getY()));
      assertFalse(Double.isNaN(axisAngle.getZ()));
      assertTrue(Double.isNaN(axisAngle.getAngle()));
   }

   @Test
   public void testInverse() throws Exception
   {
      Random random = new Random(23423L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         T axisAngle = createRandomAxisAngle(random);
         T inverseAxisAngle = createEmptyAxisAngle();
         inverseAxisAngle.set(axisAngle);
         inverseAxisAngle.inverse();

         RotationMatrix rotationMatrix = new RotationMatrix(axisAngle);
         RotationMatrix inverseRotationMatrix = new RotationMatrix(inverseAxisAngle);
         RotationMatrix identityExpected = new RotationMatrix();
         identityExpected.set(rotationMatrix);
         identityExpected.multiply(inverseRotationMatrix);
         EuclidCoreTestTools.assertIdentity(identityExpected, getEpsilon());

         T zeroExpected = createEmptyAxisAngle();
         zeroExpected.multiply(axisAngle, inverseAxisAngle);
         EuclidCoreTestTools.assertAngleEquals(0.0, zeroExpected.getAngle(), getEpsilon());
      }
   }

   @Test
   public void testSetWithDoubles()
   {
      Random random = new Random(5646541L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         T expectedAxisAngle = createRandomAxisAngle(random);
         T actualAxisAngle = createEmptyAxisAngle();
         actualAxisAngle.set(expectedAxisAngle.getX(), expectedAxisAngle.getY(), expectedAxisAngle.getZ(), expectedAxisAngle.getAngle());
         assertTrue(expectedAxisAngle.equals(actualAxisAngle));
      }
   }

   @Test
   public void testSet()
   {
      T actualAxisAngle = createEmptyAxisAngle();
      T expectedAxisAngle = createEmptyAxisAngle();
      Random random = new Random(64654L);

      { // Test set(VectorBasics axis, double angle)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            Vector3D vectorAxis = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, 1.0);
            double angle = random.nextDouble();

            actualAxisAngle.set(vectorAxis, angle);

            assertEquals(actualAxisAngle.getX(), vectorAxis.getX(), getEpsilon());
            assertEquals(actualAxisAngle.getY(), vectorAxis.getY(), getEpsilon());
            assertEquals(actualAxisAngle.getZ(), vectorAxis.getZ(), getEpsilon());
            assertEquals(actualAxisAngle.getAngle(), angle, getEpsilon());
         }
      }

      { // Test set(T other)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            expectedAxisAngle = createRandomAxisAngle(random);
            actualAxisAngle.set(expectedAxisAngle);
            EuclidCoreTestTools.assertAxisAngleEquals(actualAxisAngle, expectedAxisAngle, getEpsilon());
         }
      }

      { // Test set(AxisAngleReadOnly other)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            expectedAxisAngle = createRandomAxisAngle(random);
            actualAxisAngle.set((AxisAngleReadOnly) expectedAxisAngle);
            EuclidCoreTestTools.assertAxisAngleEquals(actualAxisAngle, expectedAxisAngle, getEpsilon());
         }
      }

      { // Test set(double[] axisAngleArray)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            expectedAxisAngle = createRandomAxisAngle(random);
            double[] axisAngleArray = new double[] {expectedAxisAngle.getX(), expectedAxisAngle.getY(), expectedAxisAngle.getZ(), expectedAxisAngle.getAngle()};
            actualAxisAngle.set(axisAngleArray);
            EuclidCoreTestTools.assertAxisAngleEquals(expectedAxisAngle, actualAxisAngle, getEpsilon());
         }
      }

      { // Test set(double[] axisAngleArray, int startIndex)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            expectedAxisAngle = createRandomAxisAngle(random);
            int startIndex = random.nextInt(10);
            double[] axisAngleArray = new double[4 + startIndex + random.nextInt(10)];
            expectedAxisAngle.get(startIndex, axisAngleArray);
            actualAxisAngle.set(startIndex, axisAngleArray);
            EuclidCoreTestTools.assertAxisAngleEquals(expectedAxisAngle, actualAxisAngle, getEpsilon());
         }
      }

      { // Test set(float[] axisAngleArray)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            expectedAxisAngle = createRandomAxisAngle(random);
            float[] axisAngleArray = new float[] {expectedAxisAngle.getX32(), expectedAxisAngle.getY32(), expectedAxisAngle.getZ32(),
                  expectedAxisAngle.getAngle32()};
            actualAxisAngle.set(axisAngleArray);
            assertTrue(expectedAxisAngle.getX32() == actualAxisAngle.getX32());
            assertTrue(expectedAxisAngle.getY32() == actualAxisAngle.getY32());
            assertTrue(expectedAxisAngle.getZ32() == actualAxisAngle.getZ32());
            assertTrue(expectedAxisAngle.getAngle32() == actualAxisAngle.getAngle32());
         }
      }

      { // Test set(float[] axisAngleArray, int startIndex)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            expectedAxisAngle = createRandomAxisAngle(random);
            int startIndex = random.nextInt(10);
            float[] axisAngleArray = new float[4 + startIndex + random.nextInt(10)];
            expectedAxisAngle.get(startIndex, axisAngleArray);
            actualAxisAngle.set(startIndex, axisAngleArray);
            assertTrue(expectedAxisAngle.getX32() == actualAxisAngle.getX32());
            assertTrue(expectedAxisAngle.getY32() == actualAxisAngle.getY32());
            assertTrue(expectedAxisAngle.getZ32() == actualAxisAngle.getZ32());
            assertTrue(expectedAxisAngle.getAngle32() == actualAxisAngle.getAngle32());
         }
      }

      { // Test set(QuaternionReadOnly quaternion)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            Quaternion quaternion = EuclidCoreRandomTools.generateRandomQuaternion(random);
            actualAxisAngle.set(quaternion);
            AxisAngleConversion.convertQuaternionToAxisAngle(quaternion, expectedAxisAngle);
            EuclidCoreTestTools.assertAxisAngleEquals(expectedAxisAngle, actualAxisAngle, getEpsilon());
         }
      }

      { // Test set(RotationMatrixReadOnly rotationMatrix)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            RotationMatrix matrix = EuclidCoreRandomTools.generateRandomRotationMatrix(random);
            actualAxisAngle.set(matrix);
            AxisAngleConversion.convertMatrixToAxisAngle(matrix, expectedAxisAngle);
            EuclidCoreTestTools.assertAxisAngleEquals(expectedAxisAngle, actualAxisAngle, getEpsilon());
         }
      }

      { // Test set(Vector3DReadOnly rotationVector)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            Vector3D rotationVector = EuclidCoreRandomTools.generateRandomRotationVector(random);
            actualAxisAngle.set(rotationVector);
            AxisAngleConversion.convertRotationVectorToAxisAngle(rotationVector, expectedAxisAngle);
            EuclidCoreTestTools.assertAxisAngleEquals(expectedAxisAngle, actualAxisAngle, getEpsilon());
         }
      }

      { // Test setYawPitchRoll(double[] yawPitchRoll)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            double[] yawPitchRoll = EuclidCoreRandomTools.generateRandomYawPitchRoll(random);
            actualAxisAngle.setYawPitchRoll(yawPitchRoll);
            AxisAngleConversion.convertYawPitchRollToAxisAngle(yawPitchRoll, expectedAxisAngle);
            EuclidCoreTestTools.assertAxisAngleEquals(expectedAxisAngle, actualAxisAngle, getEpsilon());
         }
      }

      { // Test setYawPitchRoll(double yaw, double pitch, double roll)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            double yaw = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);
            double pitch = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI / 2.0);
            double roll = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);
            actualAxisAngle.setYawPitchRoll(yaw, pitch, roll);
            AxisAngleConversion.convertYawPitchRollToAxisAngle(yaw, pitch, roll, expectedAxisAngle);
            EuclidCoreTestTools.assertAxisAngleEquals(expectedAxisAngle, actualAxisAngle, getEpsilon());
         }
      }

      { // Test set(int index, double value)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            actualAxisAngle = createRandomAxisAngle(random);

            for (int index = 0; index < 4; index++)
            {
               double expectedValue = random.nextDouble();
               actualAxisAngle.set(index, expectedValue);
               double actualValue = actualAxisAngle.get(index);
               assertEquals(expectedValue, actualValue, getEpsilon());
            }
         }
      }
   }

   @Test
   public void testMultiply()
   {
      Random random = new Random(65445L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         T aaOther1 = createRandomAxisAngle(random);
         T aaOther2 = createRandomAxisAngle(random);
         T aaActual = createRandomAxisAngle(random);
         T aaExpected = createEmptyAxisAngle();

         Quaternion qOther1 = new Quaternion(aaOther1);
         Quaternion qOther2 = new Quaternion(aaOther2);
         Quaternion qExpected = new Quaternion();

         { // Test multiplyConjugateThis(AxisAngleReadOnly other)
            aaActual.set(aaOther1);
            qExpected.set(aaOther1);
            aaActual.multiply(aaOther2);

            AxisAngleTools.multiply(aaOther1, aaOther2, aaExpected);
            EuclidCoreTestTools.assertAxisAngleEquals(aaActual, aaExpected, getEpsilon());

            QuaternionTools.multiply(qOther1, qOther2, qExpected);
            aaExpected.set(qExpected);
            EuclidCoreTestTools.assertAxisAngleEquals(aaActual, aaExpected, getEpsilon());

            // Corrupt axis of aaActual
            aaActual.set(aaOther1);
            qExpected.set(aaOther1);
            double scale = 0.5 + random.nextDouble();
            aaActual.setX(scale * aaActual.getX());
            aaActual.setY(scale * aaActual.getY());
            aaActual.setZ(scale * aaActual.getZ());

            aaActual.multiply(aaOther2);
            QuaternionTools.multiply(qOther1, qOther2, qExpected);
            aaExpected.set(qExpected);
            EuclidCoreTestTools.assertAxisAngleEquals(aaActual, aaExpected, getEpsilon());

            // Corrupt axis of aaOther2
            aaActual.set(aaOther1);
            qExpected.set(aaOther1);
            QuaternionTools.multiply(qOther1, qOther2, qExpected);

            aaOther2.setX(scale * aaOther2.getX());
            aaOther2.setY(scale * aaOther2.getY());
            aaOther2.setZ(scale * aaOther2.getZ());

            aaActual.multiply(aaOther2);
            aaExpected.set(qExpected);
            EuclidCoreTestTools.assertAxisAngleEquals(aaActual, aaExpected, getEpsilon());

            // Check that it doesn't anything for bad axis-angles
            aaActual.multiply(createAxisAngle(0.0, 0.0, 0.0, random.nextDouble()));
            EuclidCoreTestTools.assertAxisAngleEquals(aaActual, aaExpected, getEpsilon());

            aaExpected.set(0.0, 0.0, 0.0, random.nextDouble());
            aaActual.set(aaExpected);
            aaActual.multiply(aaOther2);
            EuclidCoreTestTools.assertAxisAngleEquals(aaActual, aaExpected, getEpsilon());
         }
      }
   }

   @Test
   public void testMultiplyInvert()
   {
      Random random = new Random(65445L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         T aaOther1 = createRandomAxisAngle(random);
         T aaOther2 = createRandomAxisAngle(random);
         T aaActual = createRandomAxisAngle(random);
         T aaExpected = createEmptyAxisAngle();

         Quaternion qOther1 = new Quaternion(aaOther1);
         Quaternion qOther2 = new Quaternion(aaOther2);
         Quaternion qExpected = new Quaternion();

         { // Test multiplyInvertThis(AxisAngleReadOnly other)
            aaActual.set(aaOther1);
            qExpected.set(aaOther1);
            aaActual.multiplyInvertThis(aaOther2);

            AxisAngleTools.multiplyInvertLeft(aaOther1, aaOther2, aaExpected);
            EuclidCoreTestTools.assertAxisAngleEquals(aaActual, aaExpected, getEpsilon());

            QuaternionTools.multiplyConjugateLeft(qOther1, qOther2, qExpected);
            aaExpected.set(qExpected);
            EuclidCoreTestTools.assertAxisAngleEquals(aaActual, aaExpected, getEpsilon());
         }

         { // Test multiplyInvertOther(AxisAngleReadOnly other)
            aaActual.set(aaOther1);
            aaActual.multiplyInvertOther(aaOther2);
            QuaternionTools.multiplyConjugateRight(qOther1, qOther2, qExpected);

            AxisAngleTools.multiplyInvertRight(aaOther1, aaOther2, aaExpected);
            EuclidCoreTestTools.assertAxisAngleEquals(aaActual, aaExpected, getEpsilon());

            aaExpected.set(qExpected);
            EuclidCoreTestTools.assertAxisAngleEquals(aaActual, aaExpected, getEpsilon());
         }
      }
   }

   @Test
   public void testPreMultiply()
   {
      Random random = new Random(65445L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         T aaOther1 = createRandomAxisAngle(random);
         T aaOther2 = createRandomAxisAngle(random);
         T aaActual = createRandomAxisAngle(random);
         T aaExpected = createEmptyAxisAngle();

         Quaternion qOther1 = new Quaternion(aaOther1);
         Quaternion qOther2 = new Quaternion(aaOther2);
         Quaternion qExpected = new Quaternion();

         { // Test multiplyConjugateThis(AxisAngleReadOnly other)
            aaActual.set(aaOther1);
            qExpected.set(aaOther1);
            aaActual.preMultiply(aaOther2);

            AxisAngleTools.multiply(aaOther2, aaOther1, aaExpected);
            EuclidCoreTestTools.assertAxisAngleEquals(aaActual, aaExpected, getEpsilon());

            QuaternionTools.multiply(qOther2, qOther1, qExpected);
            aaExpected.set(qExpected);
            EuclidCoreTestTools.assertAxisAngleEquals(aaActual, aaExpected, getEpsilon());
         }
      }
   }

   @Test
   public void testPreMultiplyInvert()
   {
      Random random = new Random(65445L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         T aaOther1 = createRandomAxisAngle(random);
         T aaOther2 = createRandomAxisAngle(random);
         T aaActual = createRandomAxisAngle(random);
         T aaExpected = createEmptyAxisAngle();

         Quaternion qOther1 = new Quaternion(aaOther1);
         Quaternion qOther2 = new Quaternion(aaOther2);
         Quaternion qExpected = new Quaternion();

         { // Test multiplyInvertThis(AxisAngleReadOnly other)
            aaActual.set(aaOther1);
            qExpected.set(aaOther1);
            aaActual.preMultiplyInvertThis(aaOther2);

            AxisAngleTools.multiplyInvertRight(aaOther2, aaOther1, aaExpected);
            EuclidCoreTestTools.assertAxisAngleEquals(aaActual, aaExpected, getEpsilon());

            QuaternionTools.multiplyConjugateRight(qOther2, qOther1, qExpected);
            aaExpected.set(qExpected);
            EuclidCoreTestTools.assertAxisAngleEquals(aaActual, aaExpected, getEpsilon());
         }

         { // Test multiplyInvertOther(AxisAngleReadOnly other)
            aaActual.set(aaOther1);
            aaActual.preMultiplyInvertOther(aaOther2);
            QuaternionTools.multiplyConjugateLeft(qOther2, qOther1, qExpected);

            AxisAngleTools.multiplyInvertLeft(aaOther2, aaOther1, aaExpected);
            EuclidCoreTestTools.assertAxisAngleEquals(aaActual, aaExpected, getEpsilon());

            aaExpected.set(qExpected);
            EuclidCoreTestTools.assertAxisAngleEquals(aaActual, aaExpected, getEpsilon());
         }
      }
   }

   @Test
   public void testAppendYawPitchRoll() throws Exception
   {
      Random random = new Random(35454L);
      
      T expected = createEmptyAxisAngle();
      T actual = createEmptyAxisAngle();
      double scale = 0.5 + random.nextDouble();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // appendYawRotation(double yaw)
         T original = createRandomAxisAngle(random);
         double yaw = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);
         T yawRotation = createAxisAngle(0.0, 0.0, 1.0, yaw);

         AxisAngleTools.multiply(original, yawRotation, expected);

         actual.set(original);
         actual.setX(scale * actual.getX());
         actual.setY(scale * actual.getY());
         actual.setZ(scale * actual.getZ());
         actual.appendYawRotation(yaw);

         EuclidCoreTestTools.assertAxisAngleEquals(expected, actual, getEpsilon());

         expected.set(0.0, 0.0, 0.0, random.nextDouble());
         actual.set(expected);
         actual.appendYawRotation(yaw);
         EuclidCoreTestTools.assertAxisAngleEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // appendPitchRotation(double pitch)
         T original = createRandomAxisAngle(random);
         double pitch = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);
         T pitchRotation = createAxisAngle(0.0, 1.0, 0.0, pitch);

         AxisAngleTools.multiply(original, pitchRotation, expected);

         actual.set(original);
         actual.setX(scale * actual.getX());
         actual.setY(scale * actual.getY());
         actual.setZ(scale * actual.getZ());
         actual.appendPitchRotation(pitch);

         EuclidCoreTestTools.assertAxisAngleEquals(expected, actual, getEpsilon());

         expected.set(0.0, 0.0, 0.0, random.nextDouble());
         actual.set(expected);
         actual.appendPitchRotation(pitch);
         EuclidCoreTestTools.assertAxisAngleEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // appendRollRotation(double roll)
         T original = createRandomAxisAngle(random);
         double roll = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);
         T rollRotation = createAxisAngle(1.0, 0.0, 0.0, roll);

         AxisAngleTools.multiply(original, rollRotation, expected);

         actual.set(original);
         actual.setX(scale * actual.getX());
         actual.setY(scale * actual.getY());
         actual.setZ(scale * actual.getZ());
         actual.appendRollRotation(roll);

         EuclidCoreTestTools.assertAxisAngleEquals(expected, actual, getEpsilon());

         expected.set(0.0, 0.0, 0.0, random.nextDouble());
         actual.set(expected);
         actual.appendRollRotation(roll);
         EuclidCoreTestTools.assertAxisAngleEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testPrependYawPitchRoll() throws Exception
   {
      Random random = new Random(35454L);
      
      T expected = createEmptyAxisAngle();
      T actual = createEmptyAxisAngle();
      double scale = 0.5 + random.nextDouble();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // appendYawRotation(double yaw)
         T original = createRandomAxisAngle(random);
         double yaw = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);
         T yawRotation = createAxisAngle(0.0, 0.0, 1.0, yaw);

         AxisAngleTools.multiply(yawRotation, original, expected);

         actual.set(original);
         actual.setX(scale * actual.getX());
         actual.setY(scale * actual.getY());
         actual.setZ(scale * actual.getZ());
         actual.prependYawRotation(yaw);

         EuclidCoreTestTools.assertAxisAngleEquals(expected, actual, getEpsilon());

         expected.set(0.0, 0.0, 0.0, random.nextDouble());
         actual.set(expected);
         actual.prependYawRotation(yaw);
         EuclidCoreTestTools.assertAxisAngleEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // appendPitchRotation(double pitch)
         T original = createRandomAxisAngle(random);
         double pitch = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);
         T pitchRotation = createAxisAngle(0.0, 1.0, 0.0, pitch);

         AxisAngleTools.multiply(pitchRotation, original, expected);

         actual.set(original);
         actual.setX(scale * actual.getX());
         actual.setY(scale * actual.getY());
         actual.setZ(scale * actual.getZ());
         actual.prependPitchRotation(pitch);

         EuclidCoreTestTools.assertAxisAngleEquals(expected, actual, getEpsilon());

         expected.set(0.0, 0.0, 0.0, random.nextDouble());
         actual.set(expected);
         actual.prependPitchRotation(pitch);
         EuclidCoreTestTools.assertAxisAngleEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // appendRollRotation(double roll)
         T original = createRandomAxisAngle(random);
         double roll = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);
         T rollRotation = createAxisAngle(1.0, 0.0, 0.0, roll);

         AxisAngleTools.multiply(rollRotation, original, expected);

         actual.set(original);
         actual.setX(scale * actual.getX());
         actual.setY(scale * actual.getY());
         actual.setZ(scale * actual.getZ());
         actual.prependRollRotation(roll);

         EuclidCoreTestTools.assertAxisAngleEquals(expected, actual, getEpsilon());

         expected.set(0.0, 0.0, 0.0, random.nextDouble());
         actual.set(expected);
         actual.prependRollRotation(roll);
         EuclidCoreTestTools.assertAxisAngleEquals(expected, actual, getEpsilon());
      }
   }
}