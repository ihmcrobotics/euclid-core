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
import us.ihmc.geometry.tuple.RotationVectorConversion;
import us.ihmc.geometry.tuple.Vector;
import us.ihmc.geometry.tuple.interfaces.Vector3DBasics;
import us.ihmc.geometry.tuple.interfaces.VectorReadOnly;
import us.ihmc.geometry.tuple4D.Quaternion;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;

public class AxisAngleTest
{
   public static final int NUMBER_OF_ITERATIONS = 100;
   public static final double EPS = 1e-15;

   @Test
   public void testAxisAngle()
   {
      Random random = new Random(613615L);
      AxisAngle axisAngle = new AxisAngle(), expected;
      { // Test AxisAngle()
         Assert.assertTrue(axisAngle.getX() == 1.0);
         Assert.assertTrue(axisAngle.getY() == 0.0);
         Assert.assertTrue(axisAngle.getZ() == 0.0);
         Assert.assertTrue(axisAngle.getAngle() == 0.0);
      }

      { // Test AxisAngle(AxisAngleBasics other)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            axisAngle = expected = GeometryBasicsRandomTools.generateRandomAxisAngle(random);

            AxisAngle axisAngle2 = new AxisAngle((AxisAngleReadOnly<AxisAngle>) axisAngle);

            GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle, axisAngle2, EPS);
            GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle, expected, EPS);
         }
      }

      { // Test AxisAngle(double x, double y, double z, double angle)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            expected = GeometryBasicsRandomTools.generateRandomAxisAngle(random);
            axisAngle = new AxisAngle(expected.getX(), expected.getY(), expected.getZ(), expected.getAngle());

            GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle, expected, EPS);
         }
      }

      { // Test AxisAngle(double[] axisAngleArray)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            expected = GeometryBasicsRandomTools.generateRandomAxisAngle(random);
            double[] axisAngleArray;
            double[] axisAngleArrayCopy;
            axisAngleArray = axisAngleArrayCopy = new double[] {expected.getX(), expected.getY(), expected.getZ(), expected.getAngle()};

            axisAngle = new AxisAngle(axisAngleArray);

            GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle, expected, EPS);

            for (int j = 0; j < axisAngleArray.length; j++)
               Assert.assertTrue(axisAngleArray[j] == axisAngleArrayCopy[j]);
         }
      }

      { // Test AxisAngle(VectorBasics axis, double angle)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            Vector vectorAxis, vectorAxisCopy;
            vectorAxis = vectorAxisCopy = GeometryBasicsRandomTools.generateRandomVector(random);

            double angle, angleCopy;
            angle = angleCopy = random.nextDouble();

            axisAngle = new AxisAngle(vectorAxis, angle);

            Assert.assertTrue(axisAngle.getX() == vectorAxis.getX());
            Assert.assertTrue(axisAngle.getY() == vectorAxis.getY());
            Assert.assertTrue(axisAngle.getZ() == vectorAxis.getZ());
            Assert.assertTrue(axisAngle.getAngle() == angle);

            GeometryBasicsTestTools.assertRotationVectorEquals(vectorAxis, vectorAxisCopy, EPS);
            Assert.assertTrue(angle == angleCopy);
         }
      }

      { // Test AxisAngle(QuaternionBasics quaternion)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            Quaternion quaternion, quaternionCopy;
            quaternion = quaternionCopy = GeometryBasicsRandomTools.generateRandomQuaternion(random);

            axisAngle = new AxisAngle((QuaternionReadOnly) quaternion);
            AxisAngle expectedAxisAngle = new AxisAngle();
            AxisAngleConversion.convertQuaternionToAxisAngle(quaternion, (AxisAngleBasics<?>) expectedAxisAngle);

            GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle, expectedAxisAngle, EPS);
            GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
         }
      }

      { // Test AxisAngle(RotationMatrix rotationMatrix)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            RotationMatrix matrix, matrixCopy;
            matrix = matrixCopy = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);

            double angle;
            double angleCopy;
            angle = angleCopy = random.nextDouble();

            axisAngle = new AxisAngle(matrix);
            AxisAngle expectedAxisAngle = new AxisAngle();
            AxisAngleConversion.convertMatrixToAxisAngle((RotationMatrixReadOnly) matrix, (AxisAngleBasics<?>) expectedAxisAngle);

            GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle, expectedAxisAngle, EPS);

            Assert.assertTrue(angle == angleCopy);

            GeometryBasicsTestTools.assertMatrix3DEquals(matrix, matrixCopy, EPS);
         }
      }

      { // Test AxisAngle(VectorBasics rotationVector)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            Vector rotationVector, rotationVectorCopy;
            rotationVector = rotationVectorCopy = GeometryBasicsRandomTools.generateRandomRotationVector(random);

            axisAngle = new AxisAngle(rotationVector);
            AxisAngle expectedAxisAngle = new AxisAngle();
            AxisAngleConversion.convertRotationVectorToAxisAngle((VectorReadOnly) rotationVector, (AxisAngleBasics<?>) expectedAxisAngle);

            GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle, expectedAxisAngle, EPS);
            GeometryBasicsTestTools.assertRotationVectorEquals(rotationVector, rotationVectorCopy, EPS);
         }
      }
   }

   @Test
   public void testContainsNaN()
   {
      AxisAngle axisAngle = new AxisAngle();

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
      AxisAngle axisAngle = new AxisAngle();
      AxisAngle expected;
      Random random = new Random(64654L);

      { // Test set(AxisAngle other)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            AxisAngle axisAngle2;
            AxisAngle axisAngle2Copy;

            axisAngle2 = axisAngle2Copy = GeometryBasicsRandomTools.generateRandomAxisAngle(random);
            axisAngle.set(axisAngle2);

            GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle, axisAngle2, EPS);
            GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle2, axisAngle2Copy, EPS);
         }
      }

      { // Test set(AxisAngleBasics other)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            AxisAngle axisAngle2;
            AxisAngle axisAngle2Copy;

            axisAngle2 = axisAngle2Copy = GeometryBasicsRandomTools.generateRandomAxisAngle(random);
            axisAngle.set((AxisAngleReadOnly<AxisAngle>) axisAngle2);

            GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle, axisAngle2, EPS);
            GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle2, axisAngle2Copy, EPS);
         }
      }

      { // Test set(double x, double y, double z, double angle)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            expected = GeometryBasicsRandomTools.generateRandomAxisAngle(random);
            axisAngle.set(expected.getX(), expected.getY(), expected.getZ(), expected.getAngle());

            GeometryBasicsTestTools.assertAxisAngleEquals(expected, axisAngle, EPS);
         }
      }

      { // Test set(double[] axisAngleArray)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            expected = GeometryBasicsRandomTools.generateRandomAxisAngle(random);
            double[] axisAngleArray, axisAngleArrayCopy;
            axisAngleArray = axisAngleArrayCopy = new double[] {expected.getX(), expected.getY(), expected.getZ(), expected.getAngle()};

            axisAngle.set(axisAngleArray);

            GeometryBasicsTestTools.assertAxisAngleEquals(expected, axisAngle, EPS);

            for (int j = 0; j < axisAngleArray.length; j++)
               Assert.assertTrue(axisAngleArray[0] == axisAngleArrayCopy[0]);
         }
      }

      { // Test set(double[] axisAngleArray, int startIndex)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            expected = GeometryBasicsRandomTools.generateRandomAxisAngle(random);
            double[] axisAngleArray, axisAngleArrayCopy;
            axisAngleArray = axisAngleArrayCopy = new double[] {expected.getX(), expected.getY(), expected.getZ(), expected.getAngle()};
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
            AxisAngle expectedAxisAngle = new AxisAngle();
            AxisAngleConversion.convertQuaternionToAxisAngle(quaternion, (AxisAngleBasics<?>) expectedAxisAngle);

            GeometryBasicsTestTools.assertAxisAngleEquals(expectedAxisAngle, axisAngle, EPS);

            GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
         }
      }

      { // Test set(RotationMatrix rotationMatrix) about X-axis
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            RotationMatrix matrix, matrixCopy;
            matrix = matrixCopy = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);

            axisAngle.set(matrix);
            AxisAngle expectedAxisAngle = new AxisAngle();
            AxisAngleConversion.convertMatrixToAxisAngle((RotationMatrixReadOnly) matrix, (AxisAngleBasics<?>) expectedAxisAngle);

            GeometryBasicsTestTools.assertAxisAngleEquals(expectedAxisAngle, axisAngle, EPS);
            GeometryBasicsTestTools.assertMatrix3DEquals(matrix, matrixCopy, EPS);
         }
      }

      { // Test set(VectorBasics rotationVector)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            Vector rotationVector, rotationVectorCopy;
            rotationVector = rotationVectorCopy = GeometryBasicsRandomTools.generateRandomRotationVector(random);

            axisAngle.set(rotationVector);
            AxisAngle expectedAxisAngle = new AxisAngle();
            AxisAngleConversion.convertRotationVectorToAxisAngle((VectorReadOnly) rotationVector, (AxisAngleBasics<?>) expectedAxisAngle);

            GeometryBasicsTestTools.assertAxisAngleEquals(expectedAxisAngle, axisAngle, EPS);
            GeometryBasicsTestTools.assertRotationVectorEquals(rotationVector, rotationVectorCopy, EPS);
         }
      }

      { // Test set(VectorBasics axis, double angle)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            Vector vectorAxis, vectorAxisCopy;
            vectorAxis = vectorAxisCopy = GeometryBasicsRandomTools.generateRandomRotationVector(random);
            double angle;
            angle = random.nextDouble();

            axisAngle.set(vectorAxis, angle);

            Assert.assertTrue(axisAngle.getX() == vectorAxis.getX());
            Assert.assertTrue(axisAngle.getY() == vectorAxis.getY());
            Assert.assertTrue(axisAngle.getZ() == vectorAxis.getZ());
            Assert.assertTrue(axisAngle.getAngle() == angle);

            GeometryBasicsTestTools.assertRotationVectorEquals(vectorAxis, vectorAxisCopy, EPS);
         }
      }

   }

   @Test
   public void testSetToNaN()
   {
      AxisAngle axisAngle = new AxisAngle();
      axisAngle.setToNaN();

      boolean containsNaN = axisAngle.containsNaN();
      Assert.assertTrue(containsNaN);

      GeometryBasicsTestTools.assertAxisAngleContainsOnlyNaN(axisAngle);
   }

   @Test
   public void testSetToZero()
   {
      AxisAngle axisAngle = new AxisAngle();
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
      AxisAngle axisAngle = new AxisAngle();
      Random random = new Random(5646541L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double angle;
         double angleCopy;
         angle = angleCopy = random.nextDouble();
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
      AxisAngle axisAngle = new AxisAngle();
      Random random = new Random(5646541L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double x;
         double xCopy;
         x = xCopy = random.nextDouble();
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
      AxisAngle axisAngle = new AxisAngle();
      Random random = new Random(5646541L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double y;
         double yCopy;
         y = yCopy = random.nextDouble();
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
      AxisAngle axisAngle = new AxisAngle();
      Random random = new Random(5646541L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double z;
         double zCopy;
         z = zCopy = random.nextDouble();
         axisAngle.setZ(z);

         Assert.assertTrue(axisAngle.getX() == 1.0); // Set to default
         Assert.assertTrue(axisAngle.getY() == 0.0);
         Assert.assertTrue(axisAngle.getZ() == z);
         Assert.assertTrue(axisAngle.getAngle() == 0.0);
         Assert.assertTrue(z == zCopy);
      }
   }

   @Test
   public void testGetRotationVector()
   {
      Random random = new Random(2343456L);
      AxisAngle axisAngle, axisAngleCopy;
      Vector vector = new Vector();
      Vector expectedVector = new Vector();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         axisAngle = axisAngleCopy = GeometryBasicsRandomTools.generateRandomAxisAngle(random);

         vector.setToNaN();
         axisAngle.getRotationVector((Vector3DBasics) vector);
         RotationVectorConversion.convertAxisAngleToRotationVector(axisAngle, expectedVector);

         GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle, axisAngleCopy, EPS);
         GeometryBasicsTestTools.assertRotationVectorEquals(vector, expectedVector, EPS);
      }
   }

   @Test
   public void testGet()
   {
      Random random = new Random(3513515L);
      AxisAngle axisAngle, axisAngleCopy;
      double[] axisAngleArray = new double[4];

      { // Test get(double[] axisAngleArray)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            axisAngle = axisAngleCopy = GeometryBasicsRandomTools.generateRandomAxisAngle(random);
            axisAngle.get(axisAngleArray);

            GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle, axisAngleCopy, EPS);

            Assert.assertTrue(axisAngle.getX() == axisAngleArray[0]);
            Assert.assertTrue(axisAngle.getY() == axisAngleArray[1]);
            Assert.assertTrue(axisAngle.getZ() == axisAngleArray[2]);
            Assert.assertTrue(axisAngle.getAngle() == axisAngleArray[3]);
         }
      }

      { // Test get(double[] axisAngleArray, int startIndex)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            axisAngle = axisAngleCopy = GeometryBasicsRandomTools.generateRandomAxisAngle(random);

            int startIndex;
            int startIndexCopy;
            startIndex = startIndexCopy = 0;

            axisAngle.get(axisAngleArray, startIndex);

            GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle, axisAngleCopy, EPS);

            Assert.assertTrue(axisAngle.getX() == axisAngleArray[0]);
            Assert.assertTrue(axisAngle.getY() == axisAngleArray[1]);
            Assert.assertTrue(axisAngle.getZ() == axisAngleArray[2]);
            Assert.assertTrue(axisAngle.getAngle() == axisAngleArray[3]);

            Assert.assertTrue(startIndex == startIndexCopy);
            Assert.assertTrue(startIndex >= 0);
            Assert.assertTrue(startIndex <= startIndex + axisAngleArray.length);
         }
      }

      {
         double x    = random.nextDouble();
         double y    = random.nextDouble();
         double z    = random.nextDouble();
         double angle= random.nextDouble();
         axisAngle = new AxisAngle(x, y, z, angle);
         assertTrue(axisAngle.getX() == x);
         assertTrue(axisAngle.getY() == y);
         assertTrue(axisAngle.getZ() == z);
         assertTrue(axisAngle.getAngle() == angle);
      }
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(621541L);
      double epsilon = random.nextDouble();

      AxisAngle axisAngle1 = GeometryBasicsRandomTools.generateRandomAxisAngle(random);
      AxisAngle axisAngle2 = new AxisAngle(axisAngle1);

      assertTrue(axisAngle1.epsilonEquals(axisAngle2, epsilon));

      for (int index = 0; index < 4; index++)
      {
         axisAngle2.set(axisAngle1);
         axisAngle2.set(index, axisAngle1.get(index) + 0.999 * epsilon);
         assertTrue(axisAngle1.epsilonEquals(axisAngle2, epsilon));

         axisAngle2.set(axisAngle1);
         axisAngle2.set(index, axisAngle1.get(index) - 0.999 * epsilon);
         assertTrue(axisAngle1.epsilonEquals(axisAngle2, epsilon));

         axisAngle2.set(axisAngle1);
         axisAngle2.set(index, axisAngle1.get(index) + 1.001 * epsilon);
         assertFalse(axisAngle1.epsilonEquals(axisAngle2, epsilon));

         axisAngle2.set(axisAngle1);
         axisAngle2.set(index, axisAngle1.get(index) - 1.001 * epsilon);
         assertFalse(axisAngle1.epsilonEquals(axisAngle2, epsilon));
      }
   }

   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(621541L);
      double smallestEpsilon = 1.0e-15;

      AxisAngle axisAngle1 = GeometryBasicsRandomTools.generateRandomAxisAngle(random);
      AxisAngle axisAngle2 = new AxisAngle();

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
      AxisAngle axisAngle = GeometryBasicsRandomTools.generateRandomAxisAngle(random);

      int newHashCode, previousHashCode;
      newHashCode = axisAngle.hashCode();
      assertEquals(newHashCode, axisAngle.hashCode());

      previousHashCode = axisAngle.hashCode();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         axisAngle.set(random.nextInt(4), random.nextDouble());
         newHashCode = axisAngle.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }
}
