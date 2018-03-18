package us.ihmc.euclid.axisAngle;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Assert;
import org.junit.Test;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.rotationConversion.AxisAngleConversion;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

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
            axisAngle = expected = EuclidCoreRandomTools.nextAxisAngle(random);

            AxisAngle axisAngle2 = new AxisAngle(axisAngle);

            EuclidCoreTestTools.assertAxisAngleEquals(axisAngle, axisAngle2, EPS);
            EuclidCoreTestTools.assertAxisAngleEquals(axisAngle, expected, EPS);
         }
      }

      { // Test AxisAngle(double x, double y, double z, double angle)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            expected = EuclidCoreRandomTools.nextAxisAngle(random);
            axisAngle = new AxisAngle(expected.getX(), expected.getY(), expected.getZ(), expected.getAngle());

            EuclidCoreTestTools.assertAxisAngleEquals(axisAngle, expected, EPS);
         }
      }

      { // Test AxisAngle(double[] axisAngleArray)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            expected = EuclidCoreRandomTools.nextAxisAngle(random);
            double[] axisAngleArray;
            double[] axisAngleArrayCopy;
            axisAngleArray = axisAngleArrayCopy = new double[] {expected.getX(), expected.getY(), expected.getZ(), expected.getAngle()};

            axisAngle = new AxisAngle(axisAngleArray);

            EuclidCoreTestTools.assertAxisAngleEquals(axisAngle, expected, EPS);

            for (int j = 0; j < axisAngleArray.length; j++)
               Assert.assertTrue(axisAngleArray[j] == axisAngleArrayCopy[j]);
         }
      }

      { // Test AxisAngle(VectorBasics axis, double angle)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            Vector3D vectorAxis, vectorAxisCopy;
            vectorAxis = vectorAxisCopy = EuclidCoreRandomTools.nextVector3D(random);

            double angle, angleCopy;
            angle = angleCopy = random.nextDouble();

            axisAngle = new AxisAngle(vectorAxis, angle);

            Assert.assertTrue(axisAngle.getX() == vectorAxis.getX());
            Assert.assertTrue(axisAngle.getY() == vectorAxis.getY());
            Assert.assertTrue(axisAngle.getZ() == vectorAxis.getZ());
            Assert.assertTrue(axisAngle.getAngle() == angle);

            EuclidCoreTestTools.assertRotationVectorGeometricallyEquals(vectorAxis, vectorAxisCopy, EPS);
            Assert.assertTrue(angle == angleCopy);
         }
      }

      { // Test AxisAngle(QuaternionBasics quaternion)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            Quaternion quaternion, quaternionCopy;
            quaternion = quaternionCopy = EuclidCoreRandomTools.nextQuaternion(random);

            axisAngle = new AxisAngle(quaternion);
            AxisAngle expectedAxisAngle = new AxisAngle();
            AxisAngleConversion.convertQuaternionToAxisAngle(quaternion, expectedAxisAngle);

            EuclidCoreTestTools.assertAxisAngleEquals(axisAngle, expectedAxisAngle, EPS);
            EuclidCoreTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
         }
      }

      { // Test AxisAngle(RotationMatrix rotationMatrix)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            RotationMatrix matrix, matrixCopy;
            matrix = matrixCopy = EuclidCoreRandomTools.nextRotationMatrix(random);

            double angle;
            double angleCopy;
            angle = angleCopy = random.nextDouble();

            axisAngle = new AxisAngle(matrix);
            AxisAngle expectedAxisAngle = new AxisAngle();
            AxisAngleConversion.convertMatrixToAxisAngle(matrix, expectedAxisAngle);

            EuclidCoreTestTools.assertAxisAngleEquals(axisAngle, expectedAxisAngle, EPS);

            Assert.assertTrue(angle == angleCopy);

            EuclidCoreTestTools.assertMatrix3DEquals(matrix, matrixCopy, EPS);
         }
      }

      { // Test AxisAngle(VectorBasics rotationVector)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            Vector3D rotationVector, rotationVectorCopy;
            rotationVector = rotationVectorCopy = EuclidCoreRandomTools.nextRotationVector(random);

            axisAngle = new AxisAngle(rotationVector);
            AxisAngle expectedAxisAngle = new AxisAngle();
            AxisAngleConversion.convertRotationVectorToAxisAngle(rotationVector, expectedAxisAngle);

            EuclidCoreTestTools.assertAxisAngleEquals(axisAngle, expectedAxisAngle, EPS);
            EuclidCoreTestTools.assertRotationVectorGeometricallyEquals(rotationVector, rotationVectorCopy, EPS);
         }
      }

      { // Test AxisAngle(double yaw, double pitch, double roll)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            double[] yawPitchRoll = EuclidCoreRandomTools.nextYawPitchRoll(random);

            axisAngle = new AxisAngle(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
            AxisAngle expectedAxisAngle = new AxisAngle();
            AxisAngleConversion.convertYawPitchRollToAxisAngle(yawPitchRoll, expectedAxisAngle);

            EuclidCoreTestTools.assertAxisAngleEquals(axisAngle, expectedAxisAngle, EPS);
         }
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      AxisAngle axisAngle = EuclidCoreRandomTools.nextAxisAngle(random);

      int newHashCode, previousHashCode;
      newHashCode = axisAngle.hashCode();
      assertEquals(newHashCode, axisAngle.hashCode());

      previousHashCode = axisAngle.hashCode();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         axisAngle.setElement(random.nextInt(4), random.nextDouble());
         newHashCode = axisAngle.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }

   @Override
   @Test
   public void testGeometricallyEquals() throws Exception
   {
      super.testGeometricallyEquals();

      Random random = new Random(35454L);

      AxisAngle aabA;
      AxisAngle aabB;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; ++i)
      {
         double epsilon = random.nextDouble();
         aabA = EuclidCoreRandomTools.nextAxisAngle(random);
         AxisAngle aa = new AxisAngle(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0), epsilon + random.nextDouble() - random.nextDouble());

         aabB = new AxisAngle(aa);
         aabB.preMultiply(aabA);

         if (((AxisAngleReadOnly) aabA).geometricallyEquals(aabB, epsilon))
         {
            assertTrue(aabA.geometricallyEquals(aabB, epsilon));
         }
         else
         {
            assertFalse(aabA.geometricallyEquals(aabB, epsilon));
         }
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
      return EuclidCoreRandomTools.nextAxisAngle(random);
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
