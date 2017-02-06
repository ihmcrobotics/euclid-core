package us.ihmc.geometry.tuple3D;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Assert;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.geometry.tuple3D.Tuple3D;
import us.ihmc.geometry.tuple3D.Vector3D;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DReadOnly;

public class VectorTest extends TupleTest
{
   @Test
   public void testVector()
   {
      Random random = new Random(621541L);
      Vector3D vector = new Vector3D();

      { // Test Vector()
         Assert.assertTrue(0 == vector.getX());
         Assert.assertTrue(0 == vector.getY());
         Assert.assertTrue(0 == vector.getZ());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Vector(double x, double y, double z)
         double newX = random.nextDouble();
         double newY = random.nextDouble();
         double newZ = random.nextDouble();

         vector = new Vector3D(newX, newY, newZ);

         Assert.assertTrue(newX == vector.getX());
         Assert.assertTrue(newY == vector.getY());
         Assert.assertTrue(newZ == vector.getZ());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Vector(double[] vectorArray)
         double[] randomVectorArray = {random.nextDouble(), random.nextDouble(), random.nextDouble()};
         Vector3D vectorArray = new Vector3D(randomVectorArray);

         Assert.assertTrue(randomVectorArray[0] == vectorArray.getX());
         Assert.assertTrue(randomVectorArray[1] == vectorArray.getY());
         Assert.assertTrue(randomVectorArray[2] == vectorArray.getZ());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Vector(TupleBasics tuple)
         Vector3D vector2 = new Vector3D();
         vector2.setX(random.nextDouble());
         vector2.setY(random.nextDouble());
         vector2.setZ(random.nextDouble());

         vector = new Vector3D((Tuple3DReadOnly) vector2);

         Assert.assertTrue(vector.getX() == vector2.getX());
         Assert.assertTrue(vector.getY() == vector2.getY());
         Assert.assertTrue(vector.getZ() == vector2.getZ());
      }
   }

   @Test
   public void testSet()
   {
      Random random = new Random(65465131L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double newX = random.nextDouble();
         double newY = random.nextDouble();
         double newZ = random.nextDouble();

         Vector3D vector = new Vector3D(newX, newY, newZ);

         Vector3D vector2 = new Vector3D();
         vector2.set(vector);

         Assert.assertTrue(vector.getX() == vector2.getX());
         Assert.assertTrue(vector.getY() == vector2.getY());
         Assert.assertTrue(vector.getZ() == vector2.getZ());
      }
   }

   @Test
   public void testAngle()
   {
      Random random = new Random(56461L);
      Vector3D vector1 = new Vector3D();
      Vector3D vector2 = new Vector3D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         vector1.setX(random.nextDouble());
         vector1.setY(random.nextDouble());
         vector1.setZ(random.nextDouble());

         vector2.setX(random.nextDouble());
         vector2.setY(random.nextDouble());
         vector2.setZ(random.nextDouble());

         double actualAngle = vector1.angle(vector2);
         double dotProduct = vector1.dot(vector2);
         double magnitudeVector1 = vector1.length();
         double magnitudeVector2 = vector2.length();
         double expectedAngle = Math.acos(dotProduct / (magnitudeVector1 * magnitudeVector2));

         Assert.assertTrue(actualAngle == expectedAngle);
      }
   }

   @Test
   public void testCross()
   {
      Random random = new Random(5751684L);
      Vector3D vector1 = new Vector3D();
      Vector3D vector2 = new Vector3D();
      Vector3D vector3 = new Vector3D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         vector1.setX(random.nextDouble());
         vector1.setY(random.nextDouble());
         vector1.setZ(random.nextDouble());

         vector2.setX(random.nextDouble());
         vector2.setY(random.nextDouble());
         vector2.setZ(random.nextDouble());

         vector3.cross(vector1, vector2);

         double x = vector1.getY() * vector2.getZ() - vector1.getZ() * vector2.getY();
         double y = vector2.getX() * vector1.getZ() - vector2.getZ() * vector1.getX();
         double z = vector1.getX() * vector2.getY() - vector1.getY() * vector2.getX();

         double magnitudeVector1 = vector1.length();
         double magnitudeVector2 = vector2.length();
         double actualMagnitudeVector3 = vector3.length();

         double theta = Math.acos(vector1.dot(vector2) / (magnitudeVector1 * magnitudeVector2));
         double expectedMagnitudeVector3 = magnitudeVector1 * magnitudeVector2 * Math.sin(theta);
         double vector3DotVector1 = vector3.dot(vector1);
         double vector3DotVector2 = vector3.dot(vector2);

         Assert.assertEquals(expectedMagnitudeVector3, actualMagnitudeVector3, 1e-13);
         Assert.assertEquals(vector3DotVector1, 0, 1e-15);
         Assert.assertEquals(vector3DotVector2, 0, 1e-15);

         Assert.assertTrue(vector3.getX() == x);
         Assert.assertTrue(vector3.getY() == y);
         Assert.assertTrue(vector3.getZ() == z);
      }
   }

   @Test
   public void testDot()
   {
      Random random = new Random(615651L);
      Vector3D vector1 = new Vector3D();
      Vector3D vector2 = new Vector3D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         vector1.setX(random.nextDouble());
         vector1.setY(random.nextDouble());
         vector1.setZ(random.nextDouble());

         vector2.setX(random.nextDouble());
         vector2.setY(random.nextDouble());
         vector2.setZ(random.nextDouble());

         double actualDot = vector1.dot(vector2);
         double expectedDot = vector1.getX() * vector2.getX() + vector1.getY() * vector2.getY() + vector1.getZ() * vector2.getZ();

         Assert.assertTrue(actualDot == expectedDot);
      }
   }

   @Test
   public void testLength()
   {
      Random random = new Random(312310L);
      Vector3D vector = new Vector3D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         vector.setX(random.nextDouble());
         vector.setY(random.nextDouble());
         vector.setZ(random.nextDouble());

         double length = vector.length();
         double expectedLengthSquared = vector.getX() * vector.getX() + vector.getY() * vector.getY() + vector.getZ() * vector.getZ();
         double expectedLength = Math.sqrt(expectedLengthSquared);

         Assert.assertTrue(length == expectedLength);
      }
   }

   @Test
   public void testLengthSquared()
   {
      Random random = new Random(312310L);
      Vector3D vector = new Vector3D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         vector.setX(random.nextDouble());
         vector.setY(random.nextDouble());
         vector.setZ(random.nextDouble());

         double lengthSquared = vector.lengthSquared();
         double expectedLengthSquared = vector.getX() * vector.getX() + vector.getY() * vector.getY() + vector.getZ() * vector.getZ();

         Assert.assertTrue(lengthSquared == expectedLengthSquared);
      }
   }

   @Test
   public void testNormalize()
   {
      Random random = new Random(312310L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test normalize()
         Vector3D vector = new Vector3D();

         vector.setX(random.nextDouble());
         vector.setY(random.nextDouble());
         vector.setZ(random.nextDouble());

         vector.normalize();

         double expectedLength = 1;
         double normalLength = vector.length();
         double difference = Math.abs(expectedLength - normalLength);

         Assert.assertTrue(difference < 1e-15);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test normalize(Vector vector)
         Vector3D vector = new Vector3D();

         vector.setX(random.nextDouble());
         vector.setY(random.nextDouble());
         vector.setZ(random.nextDouble());

         Vector3D normal = new Vector3D();

         normal.setAndNormalize(vector);

         double expectedLength = 1;
         double actualLength = normal.length();
         double difference = Math.abs(expectedLength - actualLength);

         Assert.assertTrue(difference < 1e-15);
      }
   }

   @Ignore
   @Test
   public void testApplyTransform()
   {
      fail("Not yet implemented");
   }

   @Test
   public void testEpsilonEquals()
   {
      Random random = new Random(621541L);
      Vector3D vector1 = new Vector3D();
      Vector3D vector2 = new Vector3D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double epsilon = random.nextDouble();

         vector1.setX(random.nextDouble());
         vector1.setY(random.nextDouble());
         vector1.setZ(random.nextDouble());

         vector2.setX(vector1.getX() + 0.1 * epsilon);
         vector2.setY(vector1.getY() + 0.1 * epsilon);
         vector2.setZ(vector1.getZ() + 0.1 * epsilon);
         assertTrue(vector1.epsilonEquals(vector2, epsilon));

         vector2.setX(vector1.getX() - 0.1 * epsilon);
         vector2.setY(vector1.getY() - 0.1 * epsilon);
         vector2.setZ(vector1.getZ() - 0.1 * epsilon);
         assertTrue(vector1.epsilonEquals(vector2, epsilon));

         vector2.setX(vector1.getX() - 1.1 * epsilon);
         vector2.setY(vector1.getY() - 0.1 * epsilon);
         vector2.setZ(vector1.getZ() - 0.1 * epsilon);
         assertFalse(vector1.epsilonEquals(vector2, epsilon));

         vector2.setX(vector1.getX() - 0.1 * epsilon);
         vector2.setY(vector1.getY() - 1.1 * epsilon);
         vector2.setZ(vector1.getZ() - 0.1 * epsilon);
         assertFalse(vector1.epsilonEquals(vector2, epsilon));

         vector2.setX(vector1.getX() - 0.1 * epsilon);
         vector2.setY(vector1.getY() - 0.1 * epsilon);
         vector2.setZ(vector1.getZ() - 1.1 * epsilon);
         assertFalse(vector1.epsilonEquals(vector2, epsilon));

         vector2.setX(vector1.getX() + 1.1 * epsilon);
         vector2.setY(vector1.getY() + 0.1 * epsilon);
         vector2.setZ(vector1.getZ() + 0.1 * epsilon);
         assertFalse(vector1.epsilonEquals(vector2, epsilon));

         vector2.setX(vector1.getX()+ 0.1 * epsilon);
         vector2.setY(vector1.getY()+ 1.1 * epsilon);
         vector2.setZ(vector1.getZ()+ 0.1 * epsilon);
         assertFalse(vector1.epsilonEquals(vector2, epsilon));

         vector2.setX(vector1.getX() + 0.1 * epsilon);
         vector2.setY(vector1.getY() + 0.1 * epsilon);
         vector2.setZ(vector1.getZ() + 1.1 * epsilon);
         assertFalse(vector1.epsilonEquals(vector2, epsilon));

         vector2.setX(vector1.getX() + epsilon - (float) 1.0e-7);
         vector2.setY(vector1.getY() + epsilon - (float) 1.0e-7);
         vector2.setZ(vector1.getZ() + epsilon - (float) 1.0e-7);
         assertTrue(vector1.epsilonEquals(vector2, epsilon));

         vector2.setX(vector1.getX() - epsilon + (float) 1.0e-7);
         vector2.setY(vector1.getY() - epsilon + (float) 1.0e-7);
         vector2.setZ(vector1.getZ() - epsilon + (float) 1.0e-7);
         assertTrue(vector1.epsilonEquals(vector2, epsilon));
      }
   }

   @Override
   public Tuple3D createEmptyTuple()
   {
      return new Vector3D();
   }
}
