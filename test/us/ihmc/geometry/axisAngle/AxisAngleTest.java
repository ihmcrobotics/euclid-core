package us.ihmc.geometry.axisAngle;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

import java.util.Random;

import org.junit.Assert;
import org.junit.Test;

import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.testingTools.GeometryBasicsTestTools;
import us.ihmc.geometry.tuple3D.Vector3D;
import us.ihmc.geometry.tuple4D.Quaternion;

public class AxisAngleTest extends AxisAngleBasicsTest<AxisAngle>
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

            AxisAngle axisAngle2 = new AxisAngle(axisAngle);

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
            Vector3D vectorAxis, vectorAxisCopy;
            vectorAxis = vectorAxisCopy = GeometryBasicsRandomTools.generateRandomVector3D(random);

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

            axisAngle = new AxisAngle(quaternion);
            AxisAngle expectedAxisAngle = new AxisAngle();
            AxisAngleConversion.convertQuaternionToAxisAngle(quaternion, expectedAxisAngle);

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
            AxisAngleConversion.convertMatrixToAxisAngle(matrix, expectedAxisAngle);

            GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle, expectedAxisAngle, EPS);

            Assert.assertTrue(angle == angleCopy);

            GeometryBasicsTestTools.assertMatrix3DEquals(matrix, matrixCopy, EPS);
         }
      }

      { // Test AxisAngle(VectorBasics rotationVector)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            Vector3D rotationVector, rotationVectorCopy;
            rotationVector = rotationVectorCopy = GeometryBasicsRandomTools.generateRandomRotationVector(random);

            axisAngle = new AxisAngle(rotationVector);
            AxisAngle expectedAxisAngle = new AxisAngle();
            AxisAngleConversion.convertRotationVectorToAxisAngle(rotationVector, expectedAxisAngle);

            GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle, expectedAxisAngle, EPS);
            GeometryBasicsTestTools.assertRotationVectorEquals(rotationVector, rotationVectorCopy, EPS);
         }
      }

      { // Test AxisAngle(double yaw, double pitch, double roll)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            double[] yawPitchRoll = GeometryBasicsRandomTools.generateRandomYawPitchRoll(random);

            axisAngle = new AxisAngle(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
            AxisAngle expectedAxisAngle = new AxisAngle();
            AxisAngleConversion.convertYawPitchRollToAxisAngle(yawPitchRoll, expectedAxisAngle);

            GeometryBasicsTestTools.assertAxisAngleEquals(axisAngle, expectedAxisAngle, EPS);
         }
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

   @Override
   public AxisAngle createEmptyAxisAngle()
   {
      return new AxisAngle();
   }

   @Override
   public AxisAngle createAxisAngle(double ux, double uy, double uz, double angle)
   {
      return new AxisAngle(ux, uy, uz, angle);
   }

   @Override
   public AxisAngle createRandomAxisAngle(Random random)
   {
      return GeometryBasicsRandomTools.generateRandomAxisAngle(random);
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-14;
   }

   @Override
   public double getSmallestEpsilon()
   {
      return 1.0e-15;
   }
}
