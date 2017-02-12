package us.ihmc.geometry.axisAngle;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.geometry.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.testingTools.GeometryBasicsTestTools;
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
      GeometryBasicsTestTools.assertAxisAngleContainsOnlyNaN(axisAngle);
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
      GeometryBasicsTestTools.assertAxisAngleContainsOnlyNaN(axisAngle);
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
         double x = GeometryBasicsRandomTools.generateRandomDouble(random);
         double y = GeometryBasicsRandomTools.generateRandomDouble(random);
         double z = GeometryBasicsRandomTools.generateRandomDouble(random);
         double angle = GeometryBasicsRandomTools.generateRandomDouble(random);
         T expectedAxisAngle = createAxisAngle(-x, -y, -z, -angle);
         T actualAxisAngle = createAxisAngle(x, y, z, angle);
         actualAxisAngle.negate();
         assertTrue(expectedAxisAngle.equals(actualAxisAngle));
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
            Vector3D vectorAxis = GeometryBasicsRandomTools.generateRandomVector3DWithFixedLength(random, 1.0);
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
            GeometryBasicsTestTools.assertAxisAngleEquals(actualAxisAngle, expectedAxisAngle, getEpsilon());
         }
      }

      { // Test set(AxisAngleReadOnly other)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            expectedAxisAngle = createRandomAxisAngle(random);
            actualAxisAngle.set((AxisAngleReadOnly) expectedAxisAngle);
            GeometryBasicsTestTools.assertAxisAngleEquals(actualAxisAngle, expectedAxisAngle, getEpsilon());
         }
      }

      { // Test set(double[] axisAngleArray)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            expectedAxisAngle = createRandomAxisAngle(random);
            double[] axisAngleArray = new double[] {expectedAxisAngle.getX(), expectedAxisAngle.getY(), expectedAxisAngle.getZ(), expectedAxisAngle.getAngle()};
            actualAxisAngle.set(axisAngleArray);
            GeometryBasicsTestTools.assertAxisAngleEquals(expectedAxisAngle, actualAxisAngle, getEpsilon());
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
            GeometryBasicsTestTools.assertAxisAngleEquals(expectedAxisAngle, actualAxisAngle, getEpsilon());
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
            Quaternion quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
            actualAxisAngle.set(quaternion);
            AxisAngleConversion.convertQuaternionToAxisAngle(quaternion, expectedAxisAngle);
            GeometryBasicsTestTools.assertAxisAngleEquals(expectedAxisAngle, actualAxisAngle, getEpsilon());
         }
      }

      { // Test set(RotationMatrixReadOnly rotationMatrix)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            RotationMatrix matrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
            actualAxisAngle.set(matrix);
            AxisAngleConversion.convertMatrixToAxisAngle(matrix, expectedAxisAngle);
            GeometryBasicsTestTools.assertAxisAngleEquals(expectedAxisAngle, actualAxisAngle, getEpsilon());
         }
      }

      { // Test set(Vector3DReadOnly rotationVector)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            Vector3D rotationVector = GeometryBasicsRandomTools.generateRandomRotationVector(random);
            actualAxisAngle.set(rotationVector);
            AxisAngleConversion.convertRotationVectorToAxisAngle(rotationVector, expectedAxisAngle);
            GeometryBasicsTestTools.assertAxisAngleEquals(expectedAxisAngle, actualAxisAngle, getEpsilon());
         }
      }

      { // Test setYawPitchRoll(double[] yawPitchRoll)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            double[] yawPitchRoll = GeometryBasicsRandomTools.generateRandomYawPitchRoll(random);
            actualAxisAngle.setYawPitchRoll(yawPitchRoll);
            AxisAngleConversion.convertYawPitchRollToAxisAngle(yawPitchRoll, expectedAxisAngle);
            GeometryBasicsTestTools.assertAxisAngleEquals(expectedAxisAngle, actualAxisAngle, getEpsilon());
         }
      }

      { // Test setYawPitchRoll(double yaw, double pitch, double roll)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            double yaw = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI);
            double pitch = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI / 2.0);
            double roll = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI);
            actualAxisAngle.setYawPitchRoll(yaw, pitch, roll);
            AxisAngleConversion.convertYawPitchRollToAxisAngle(yaw, pitch, roll, expectedAxisAngle);
            GeometryBasicsTestTools.assertAxisAngleEquals(expectedAxisAngle, actualAxisAngle, getEpsilon());
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
}