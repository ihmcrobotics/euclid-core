package us.ihmc.geometry.tuple4D;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.testingTools.GeometryBasicsTestTools;
import us.ihmc.geometry.tuple3D.Vector3D;

public class QuaternionTest extends QuaternionBasicsTest<Quaternion>
{
   public static final int NUMBER_OF_ITERATIONS = 100;
   public static final double EPS = 1e-14;

   @Test
   public void testQuaternion()
   {
      Random random = new Random(613615L);
      Quaternion quaternion = new Quaternion();
      Quaternion quaternionCopy;
      Quaternion expected = new Quaternion();

      { // Test Quaternion()
         expected.setToZero();
         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, expected, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Quaternion(QuaternionBasics other)
         quaternion = quaternionCopy = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         Quaternion quaternion2 = new Quaternion(quaternion);

         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternion2, EPS);
         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, quaternionCopy, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Quaternion(double x, double y, double z, double s)
         expected = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         expected.normalizeAndLimitToPiMinusPi();
         quaternion = new Quaternion(expected.getX(), expected.getY(), expected.getZ(), expected.getS());

         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, expected, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Quaternion(double[] quaternionArray)
         expected = GeometryBasicsRandomTools.generateRandomQuaternion(random);

         double[] quaternionArray;
         quaternionArray = new double[] {expected.getX(), expected.getY(), expected.getZ(), expected.getS()};

         quaternion = new Quaternion(quaternionArray);

         GeometryBasicsTestTools.assertQuaternionEquals(expected, quaternion, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Quaternion(RotationMatrix rotationMatrix)
         RotationMatrix rotationMatrix, rotationMatrixCopy;
         rotationMatrix = rotationMatrixCopy = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);

         quaternion = new Quaternion(rotationMatrix);
         QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, expected);

         GeometryBasicsTestTools.assertQuaternionEquals(expected, quaternion, EPS);
         GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Quaternion(VectorBasics rotationVector)
         Vector3D rotationVector, rotationVectorCopy;
         rotationVector = rotationVectorCopy = GeometryBasicsRandomTools.generateRandomRotationVector(random);

         quaternion = new Quaternion(rotationVector);
         QuaternionConversion.convertRotationVectorToQuaternion(rotationVector, expected);

         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, expected, EPS);
         GeometryBasicsTestTools.assertRotationVectorEquals(rotationVector, rotationVectorCopy, EPS);
      }
   }

   @Override
   @Test
   @Ignore
   public void testApplyTransform()
   {
      fail("Not yet implemented");
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      Quaternion q = GeometryBasicsRandomTools.generateRandomQuaternion(random);

      int newHashCode, previousHashCode;
      newHashCode = q.hashCode();
      assertEquals(newHashCode, q.hashCode());

      previousHashCode = q.hashCode();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double qx = q.getX();
         double qy = q.getY();
         double qz = q.getZ();
         double qs = q.getS();
         switch (random.nextInt(4))
         {
         case 0:
            qx = random.nextDouble();
            break;
         case 1:
            qy = random.nextDouble();
            break;
         case 2:
            qz = random.nextDouble();
            break;
         case 3:
            qs = random.nextDouble();
            break;
         }
         q.setUnsafe(qx, qy, qz, qs);
         newHashCode = q.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }

   @Override
   public Quaternion createEmptyTuple()
   {
      return new Quaternion();
   }

   @Override
   public Quaternion createRandomTuple(Random random)
   {
      return GeometryBasicsRandomTools.generateRandomQuaternion(random);
   }

   @Override
   public Quaternion createTuple(double x, double y, double z, double s)
   {
      Quaternion quaternion = new Quaternion();
      quaternion.setUnsafe(x, y, z, s);
      return quaternion;
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-15;
   }
}
