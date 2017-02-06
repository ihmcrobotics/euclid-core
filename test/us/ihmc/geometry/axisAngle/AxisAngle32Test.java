package us.ihmc.geometry.axisAngle;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Assert;
import org.junit.Test;

import us.ihmc.geometry.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.testingTools.GeometryBasicsTestTools;
import us.ihmc.geometry.tuple3D.Vector;
import us.ihmc.geometry.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.tuple4D.Quaternion;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;

public class AxisAngle32Test
{
   public static final int NUMBER_OF_ITERATIONS = AxisAngleTest.NUMBER_OF_ITERATIONS;
   public static final double EPS = 1e-6;

   @Test
   public void testAxisAngle32()
   {
      Random random = new Random(613615L);
      AxisAngle32 axisAngle = new AxisAngle32(), expected;
      { // Test AxisAngle32()
         Assert.assertTrue(axisAngle.getX() == 1.0);
         Assert.assertTrue(axisAngle.getY() == 0.0);
         Assert.assertTrue(axisAngle.getZ() == 0.0);
         Assert.assertTrue(axisAngle.getAngle() == 0.0);
      }

      { // Test AxisAngle32(AxisAngleBasics other)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            axisAngle = expected = GeometryBasicsRandomTools.generateRandomAxisAngle32(random);

            AxisAngle32 axisAngle2 = new AxisAngle32((AxisAngleReadOnly<AxisAngle32>) axisAngle);

            GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle, axisAngle2, EPS);
            GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle, expected, EPS);
         }
      }

      { // Test AxisAngle32(float x, float y, float z, float angle)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            expected = GeometryBasicsRandomTools.generateRandomAxisAngle32(random);
            axisAngle = new AxisAngle32(expected.getX32(), expected.getY32(), expected.getZ32(), expected.getAngle32());

            GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle, expected, EPS);
         }
      }

      { // Test AxisAngle32(float[] axisAngleArray)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            expected = GeometryBasicsRandomTools.generateRandomAxisAngle32(random);
            float[] axisAngleArray;
            float[] axisAngleArrayCopy;
            axisAngleArray = axisAngleArrayCopy = new float[] {expected.getX32(), expected.getY32(), expected.getZ32(), expected.getAngle32()};

            axisAngle = new AxisAngle32(axisAngleArray);

            GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle, expected, EPS);

            for (int j = 0; j < axisAngleArray.length; j++)
               Assert.assertTrue(axisAngleArray[j] == axisAngleArrayCopy[j]);
         }
      }

      { // Test AxisAngle32(VectorBasics axis, float angle)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            Vector vectorAxis, vectorAxisCopy;
            vectorAxis = vectorAxisCopy = GeometryBasicsRandomTools.generateRandomVector(random);

            float angle, angleCopy;
            angle = angleCopy = random.nextFloat();

            axisAngle = new AxisAngle32(vectorAxis, angle);

            Assert.assertEquals(axisAngle.getX(), vectorAxis.getX(), EPS);
            Assert.assertEquals(axisAngle.getY(), vectorAxis.getY(), EPS);
            Assert.assertEquals(axisAngle.getZ(), vectorAxis.getZ(), EPS);
            Assert.assertEquals(axisAngle.getAngle(), angle, EPS);

            GeometryBasicsTestTools.assertRotationVectorEquals(vectorAxis, vectorAxisCopy, EPS);
            Assert.assertTrue(angle == angleCopy);
         }
      }

      { // Test AxisAngle32(QuaternionBasics quaternion)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            Quaternion quaternion, quaternionCopy;
            quaternion = quaternionCopy = GeometryBasicsRandomTools.generateRandomQuaternion(random);

            axisAngle = new AxisAngle32((QuaternionReadOnly) quaternion);
            AxisAngle32 expectedAxisAngle32 = new AxisAngle32();
            AxisAngleConversion.convertQuaternionToAxisAngle(quaternion, (AxisAngleBasics<?>) expectedAxisAngle32);

            GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle, expectedAxisAngle32, EPS);
            GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
         }
      }

      { // Test AxisAngle32(RotationMatrix rotationMatrix)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            RotationMatrix matrix, matrixCopy;
            matrix = matrixCopy = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);

            float angle;
            float angleCopy;
            angle = angleCopy = random.nextFloat();

            axisAngle = new AxisAngle32(matrix);
            AxisAngle32 expectedAxisAngle32 = new AxisAngle32();
            AxisAngleConversion.convertMatrixToAxisAngle((RotationMatrixReadOnly) matrix, (AxisAngleBasics<?>) expectedAxisAngle32);

            GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle, expectedAxisAngle32, EPS);

            Assert.assertTrue(angle == angleCopy);

            GeometryBasicsTestTools.assertMatrix3DEquals(matrix, matrixCopy, EPS);
         }
      }

      { // Test AxisAngle32(VectorBasics rotationVector)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            Vector rotationVector, rotationVectorCopy;
            rotationVector = rotationVectorCopy = GeometryBasicsRandomTools.generateRandomRotationVector(random);

            axisAngle = new AxisAngle32(rotationVector);
            AxisAngle32 expectedAxisAngle32 = new AxisAngle32();
            AxisAngleConversion.convertRotationVectorToAxisAngle((Vector3DReadOnly) rotationVector, (AxisAngleBasics<?>) expectedAxisAngle32);

            GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle, expectedAxisAngle32, EPS);
            GeometryBasicsTestTools.assertRotationVectorEquals(rotationVector, rotationVectorCopy, EPS);
         }
      }
   }

   @Test
   public void testContainsNaN()
   {
      AxisAngle32 axisAngle = new AxisAngle32();

      axisAngle.set(0.0, 0.0, 0.0, 0.0);
      assertFalse(axisAngle.containsNaN());
      axisAngle.set(Double.NaN, 0.0, 0.0, 0.0);
      assertTrue(axisAngle.containsNaN());
      axisAngle.set(0.0, Double.NaN, 0.0, 0.0);
      assertTrue(axisAngle.containsNaN());
      axisAngle.set(0.0, 0.0, Double.NaN, 0.0);
      assertTrue(axisAngle.containsNaN());
      axisAngle.set(0.0, 0.0, 0.0, Double.NaN);
      assertTrue(axisAngle.containsNaN());
   }

   @Test
   public void testSet()
   {
      AxisAngle32 axisAngle = new AxisAngle32();
      AxisAngle32 expected;
      Random random = new Random(64654L);

      { // Test set(AxisAngle32 other)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            AxisAngle32 axisAngle2;
            AxisAngle32 axisAngle2Copy;

            axisAngle2 = axisAngle2Copy = GeometryBasicsRandomTools.generateRandomAxisAngle32(random);
            axisAngle.set(axisAngle2);

            GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle, axisAngle2, EPS);
            GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle2, axisAngle2Copy, EPS);
         }
      }

      { // Test set(AxisAngleBasics other)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            AxisAngle32 axisAngle2;
            AxisAngle32 axisAngle2Copy;

            axisAngle2 = axisAngle2Copy = GeometryBasicsRandomTools.generateRandomAxisAngle32(random);
            axisAngle.set((AxisAngleReadOnly<?>) axisAngle2);

            GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle, axisAngle2, EPS);
            GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle2, axisAngle2Copy, EPS);
         }
      }

      { // Test set(float x, float y, float z, float angle)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            expected = GeometryBasicsRandomTools.generateRandomAxisAngle32(random);
            axisAngle.set(expected.getX32(), expected.getY32(), expected.getZ32(), expected.getAngle32());

            GeometryBasicsTestTools.assertAxisAngleEquals(expected, axisAngle, EPS);
         }
      }

      { // Test set(float[] axisAngleArray)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            expected = GeometryBasicsRandomTools.generateRandomAxisAngle32(random);
            float[] axisAngleArray, axisAngleArrayCopy;
            axisAngleArray = axisAngleArrayCopy = new float[] {expected.getX32(), expected.getY32(), expected.getZ32(), expected.getAngle32()};

            axisAngle.set(axisAngleArray);

            GeometryBasicsTestTools.assertAxisAngleEquals(expected, axisAngle, EPS);

            for (int j = 0; j < axisAngleArray.length; j++)
               Assert.assertTrue(axisAngleArray[0] == axisAngleArrayCopy[0]);
         }
      }

      { // Test set(float[] axisAngleArray, int startIndex)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            expected = GeometryBasicsRandomTools.generateRandomAxisAngle32(random);
            float[] axisAngleArray, axisAngleArrayCopy;
            axisAngleArray = axisAngleArrayCopy = new float[] {expected.getX32(), expected.getY32(), expected.getZ32(), expected.getAngle32()};
            int startIndex;
            int startIndexCopy;
            startIndex = startIndexCopy = 0;

            axisAngle.set(axisAngleArray, startIndex);

            GeometryBasicsTestTools.assertAxisAngleEquals(expected, axisAngle, EPS);

            for (int j = 0; j < axisAngleArray.length; j++)
               Assert.assertTrue(axisAngleArray[0] == axisAngleArrayCopy[0]);

            Assert.assertTrue(startIndex >= 0);
            Assert.assertTrue(startIndex == startIndexCopy);
            Assert.assertTrue(startIndex <= startIndex + axisAngleArray.length);
         }
      }

      { // Test set(QuaternionBasics quaternion)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            Quaternion quaternion, quaternionCopy;
            quaternion = quaternionCopy = GeometryBasicsRandomTools.generateRandomQuaternion(random);

            axisAngle.set(quaternion);
            AxisAngle32 expectedAxisAngle32 = new AxisAngle32();
            AxisAngleConversion.convertQuaternionToAxisAngle(quaternion, (AxisAngleBasics<?>) expectedAxisAngle32);

            GeometryBasicsTestTools.assertAxisAngleEquals(expectedAxisAngle32, axisAngle, EPS);

            GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
         }
      }

      { // Test set(RotationMatrix rotationMatrix) about X-axis
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            RotationMatrix matrix, matrixCopy;
            matrix = matrixCopy = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);

            axisAngle.set(matrix);
            AxisAngle32 expectedAxisAngle32 = new AxisAngle32();
            AxisAngleConversion.convertMatrixToAxisAngle((RotationMatrixReadOnly) matrix, (AxisAngleBasics<?>) expectedAxisAngle32);

            GeometryBasicsTestTools.assertAxisAngleEquals(expectedAxisAngle32, axisAngle, EPS);
            GeometryBasicsTestTools.assertMatrix3DEquals(matrix, matrixCopy, EPS);
         }
      }

      { // Test set(VectorBasics rotationVector)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            Vector rotationVector, rotationVectorCopy;
            rotationVector = rotationVectorCopy = GeometryBasicsRandomTools.generateRandomRotationVector(random);

            axisAngle.set(rotationVector);
            AxisAngle32 expectedAxisAngle32 = new AxisAngle32();
            AxisAngleConversion.convertRotationVectorToAxisAngle((Vector3DReadOnly) rotationVector, (AxisAngleBasics<?>) expectedAxisAngle32);

            GeometryBasicsTestTools.assertAxisAngleEquals(expectedAxisAngle32, axisAngle, EPS);
            GeometryBasicsTestTools.assertRotationVectorEquals(rotationVector, rotationVectorCopy, EPS);
         }
      }

      { // Test set(VectorBasics axis, float angle)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            Vector vectorAxis, vectorAxisCopy;
            vectorAxis = vectorAxisCopy = GeometryBasicsRandomTools.generateRandomRotationVector(random);
            float angle;
            angle = random.nextFloat();

            axisAngle.set(vectorAxis, angle);

            Assert.assertEquals(axisAngle.getX(), vectorAxis.getX(), EPS);
            Assert.assertEquals(axisAngle.getY(), vectorAxis.getY(), EPS);
            Assert.assertEquals(axisAngle.getZ(), vectorAxis.getZ(), EPS);
            Assert.assertEquals(axisAngle.getAngle(), angle, EPS);

            GeometryBasicsTestTools.assertRotationVectorEquals(vectorAxis, vectorAxisCopy, EPS);
         }
      }

   }

   @Test
   public void testSetToNaN()
   {
      AxisAngle32 axisAngle = new AxisAngle32();
      axisAngle.setToNaN();

      boolean containsNaN = axisAngle.containsNaN();
      Assert.assertTrue(containsNaN);

      GeometryBasicsTestTools.assertAxisAngleContainsOnlyNaN(axisAngle);
   }

   @Test
   public void testSetToZero()
   {
      AxisAngle32 axisAngle = new AxisAngle32();
      axisAngle.setToNaN();

      Assert.assertFalse(axisAngle.getX() == 1.0);
      Assert.assertFalse(axisAngle.getY() == 0.0);
      Assert.assertFalse(axisAngle.getZ() == 0.0);
      Assert.assertFalse(axisAngle.getAngle() == 0.0);

      GeometryBasicsTestTools.assertAxisAngleContainsOnlyNaN(axisAngle);

      axisAngle.setToZero();

      Assert.assertTrue(axisAngle.getX() == 1.0); // Set to default (x = 1.0, not 0.0)
      Assert.assertTrue(axisAngle.getY() == 0.0);
      Assert.assertTrue(axisAngle.getZ() == 0.0);
      Assert.assertTrue(axisAngle.getAngle() == 0.0);
   }

   @Test
   public void testSetAngle()
   {
      AxisAngle32 axisAngle = new AxisAngle32();
      Random random = new Random(5646541L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         float angle;
         float angleCopy;
         angle = angleCopy = random.nextFloat();
         axisAngle.setAngle(angle);

         Assert.assertTrue(axisAngle.getX() == 1.0); // Set to default
         Assert.assertTrue(axisAngle.getY() == 0.0);
         Assert.assertTrue(axisAngle.getZ() == 0.0);
         Assert.assertTrue(axisAngle.getAngle() == angle);
         Assert.assertTrue(angle == angleCopy);
      }
   }

   @Test
   public void testSetX()
   {
      AxisAngle32 axisAngle = new AxisAngle32();
      Random random = new Random(5646541L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         float x;
         float xCopy;
         x = xCopy = random.nextFloat();
         axisAngle.setX(x);

         Assert.assertTrue(axisAngle.getX() == x); // Set to default
         Assert.assertTrue(axisAngle.getY() == 0.0);
         Assert.assertTrue(axisAngle.getZ() == 0.0);
         Assert.assertTrue(axisAngle.getAngle() == 0.0);
         Assert.assertTrue(x == xCopy);
      }
   }

   @Test
   public void testSetY()
   {
      AxisAngle32 axisAngle = new AxisAngle32();
      Random random = new Random(5646541L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         float y;
         float yCopy;
         y = yCopy = random.nextFloat();
         axisAngle.setY(y);

         Assert.assertTrue(axisAngle.getX() == 1.0); // Set to default
         Assert.assertTrue(axisAngle.getY() == y);
         Assert.assertTrue(axisAngle.getZ() == 0.0);
         Assert.assertTrue(axisAngle.getAngle() == 0.0);
         Assert.assertTrue(y == yCopy);
      }
   }

   @Test
   public void testSetZ()
   {
      AxisAngle32 axisAngle = new AxisAngle32();
      Random random = new Random(5646541L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         float z;
         float zCopy;
         z = zCopy = random.nextFloat();
         axisAngle.setZ(z);

         Assert.assertTrue(axisAngle.getX() == 1.0); // Set to default
         Assert.assertTrue(axisAngle.getY() == 0.0);
         Assert.assertTrue(axisAngle.getZ() == z);
         Assert.assertTrue(axisAngle.getAngle() == 0.0);
         Assert.assertTrue(z == zCopy);
      }
   }

   @Test
   public void testGet()
   {
      Random random = new Random(3513515L);
      AxisAngle32 axisAngle, axisAngleCopy;
      float[] axisAngleArray = new float[4];

      { // Test get(float[] axisAngleArray)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            axisAngle = axisAngleCopy = GeometryBasicsRandomTools.generateRandomAxisAngle32(random);
            axisAngle.get(axisAngleArray);

            GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle, axisAngleCopy, EPS);

            Assert.assertTrue(axisAngle.getX32() == axisAngleArray[0]);
            Assert.assertTrue(axisAngle.getY32() == axisAngleArray[1]);
            Assert.assertTrue(axisAngle.getZ32() == axisAngleArray[2]);
            Assert.assertTrue(axisAngle.getAngle32() == axisAngleArray[3]);
         }
      }

      { // Test get(float[] axisAngleArray, int startIndex)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            axisAngle = axisAngleCopy = GeometryBasicsRandomTools.generateRandomAxisAngle32(random);

            int startIndex;
            int startIndexCopy;
            startIndex = startIndexCopy = 0;

            axisAngle.get(axisAngleArray, startIndex);

            GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle, axisAngleCopy, EPS);

            Assert.assertTrue(axisAngle.getX32() == axisAngleArray[0]);
            Assert.assertTrue(axisAngle.getY32() == axisAngleArray[1]);
            Assert.assertTrue(axisAngle.getZ32() == axisAngleArray[2]);
            Assert.assertTrue(axisAngle.getAngle32() == axisAngleArray[3]);

            Assert.assertTrue(startIndex == startIndexCopy);
            Assert.assertTrue(startIndex >= 0);
            Assert.assertTrue(startIndex <= startIndex + axisAngleArray.length);
         }
      }
   }

   @Test
   public void testGetAngle()
   {
      AxisAngle32 axisAngle = new AxisAngle32();
      Random random = new Random(564648L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         float setAngle = random.nextFloat();
         axisAngle.setAngle(setAngle);
         double returnAngle = axisAngle.getAngle();

         Assert.assertTrue(setAngle == returnAngle);
      }
   }

   @Test
   public void testGetX()
   {
      AxisAngle32 axisAngle = new AxisAngle32();
      Random random = new Random(564648L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         float setX = random.nextFloat();
         axisAngle.setX(setX);
         double returnX = axisAngle.getX();

         Assert.assertTrue(setX == returnX);
      }
   }

   @Test
   public void testGetY()
   {
      AxisAngle32 axisAngle = new AxisAngle32();
      Random random = new Random(564648L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         float setY = random.nextFloat();
         axisAngle.setY(setY);
         double returnY = axisAngle.getY();

         Assert.assertTrue(setY == returnY);
      }
   }

   @Test
   public void testGetZ()
   {
      AxisAngle32 axisAngle = new AxisAngle32();
      Random random = new Random(564648L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         float setZ = random.nextFloat();
         axisAngle.setZ(setZ);
         double returnZ = axisAngle.getZ();

         Assert.assertTrue(setZ == returnZ);
      }
   }

   @Test
   public void testGetAngle32()
   {
      AxisAngle32 axisAngle = new AxisAngle32();
      Random random = new Random(564648L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         float setAngle = random.nextFloat();
         axisAngle.setAngle(setAngle);
         float returnAngle = axisAngle.getAngle32();

         Assert.assertTrue(setAngle == returnAngle);
      }
   }

   @Test
   public void testGetX32()
   {
      AxisAngle32 axisAngle = new AxisAngle32();
      Random random = new Random(564648L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         float setX = random.nextFloat();
         axisAngle.setX(setX);
         float returnX = axisAngle.getX32();

         Assert.assertTrue(setX == returnX);
      }
   }

   @Test
   public void testGetY32()
   {
      AxisAngle32 axisAngle = new AxisAngle32();
      Random random = new Random(564648L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         float setY = random.nextFloat();
         axisAngle.setY(setY);
         float returnY = axisAngle.getY32();

         Assert.assertTrue(setY == returnY);
      }
   }

   @Test
   public void testGetZ32()
   {
      AxisAngle32 axisAngle = new AxisAngle32();
      Random random = new Random(564648L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         float setZ = random.nextFloat();
         axisAngle.setZ(setZ);
         float returnZ = axisAngle.getZ32();

         Assert.assertTrue(setZ == returnZ);
      }
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(621541L);
      float epsilon = random.nextFloat();

      AxisAngle32 axisAngle1 = GeometryBasicsRandomTools.generateRandomAxisAngle32(random);
      AxisAngle32 axisAngle2 = new AxisAngle32(axisAngle1);

      assertTrue(axisAngle1.epsilonEquals(axisAngle2, epsilon));

      for (int index = 0; index < 4; index++)
      {
         axisAngle2.set(axisAngle1);
         axisAngle2.set(index, axisAngle1.get(index) + 0.999f * epsilon);
         assertTrue(axisAngle1.epsilonEquals(axisAngle2, epsilon));

         axisAngle2.set(axisAngle1);
         axisAngle2.set(index, axisAngle1.get(index) - 0.999f * epsilon);
         assertTrue(axisAngle1.epsilonEquals(axisAngle2, epsilon));

         axisAngle2.set(axisAngle1);
         axisAngle2.set(index, axisAngle1.get(index) + 1.001f * epsilon);
         assertFalse(axisAngle1.epsilonEquals(axisAngle2, epsilon));

         axisAngle2.set(axisAngle1);
         axisAngle2.set(index, axisAngle1.get(index) - 1.001f * epsilon);
         assertFalse(axisAngle1.epsilonEquals(axisAngle2, epsilon));
      }
   }

   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(621541L);
      float smallestEpsilon = 1.0e-6f;

      AxisAngle32 axisAngle1 = GeometryBasicsRandomTools.generateRandomAxisAngle32(random);
      AxisAngle32 axisAngle2 = new AxisAngle32();

      assertFalse(axisAngle1.equals(axisAngle2));
      assertFalse(axisAngle1.equals(null));
      assertFalse(axisAngle1.equals(new double[5]));

      axisAngle2.set(axisAngle1);
      assertTrue(axisAngle1.equals(axisAngle2));

      for (int index = 0; index < 4; index++)
      {
         axisAngle2.set(axisAngle1);
         assertTrue(axisAngle1.equals(axisAngle2));
         axisAngle2.set(index, axisAngle1.get(index) + smallestEpsilon);
         assertFalse(axisAngle1.equals(axisAngle2));

         axisAngle2.set(axisAngle1);
         assertTrue(axisAngle1.equals(axisAngle2));
         axisAngle2.set(index, axisAngle1.get(index) - smallestEpsilon);
         assertFalse(axisAngle1.equals(axisAngle2));
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      AxisAngle32 axisAngle = GeometryBasicsRandomTools.generateRandomAxisAngle32(random);

      int newHashCode, previousHashCode;
      newHashCode = axisAngle.hashCode();
      assertEquals(newHashCode, axisAngle.hashCode());

      previousHashCode = axisAngle.hashCode();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         axisAngle.set(random.nextInt(4), random.nextFloat());
         newHashCode = axisAngle.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }
}
