package us.ihmc.geometry.tuple2D;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Assert;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple2D.interfaces.Vector2DReadOnly;

public class Vector2DTest extends Tuple2DTest<Vector2D>
{
   @Test
   public void testVector2D()
   {
      Random random = new Random(621541L);
      Vector2D vector = new Vector2D();

      { // Test Vector2D()
         Assert.assertTrue(0 == vector.getX());
         Assert.assertTrue(0 == vector.getY());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Vector2D(double x, double y, double z)
         double newX = random.nextDouble();
         double newY = random.nextDouble();

         vector = new Vector2D(newX, newY);

         Assert.assertTrue(newX == vector.getX());
         Assert.assertTrue(newY == vector.getY());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Vector2D(double[] vectorArray)
         double[] randomVector2DArray = {random.nextDouble(), random.nextDouble()};
         double[] copyRandomVector2DArray = new double[2];
         copyRandomVector2DArray[0] = randomVector2DArray[0];
         copyRandomVector2DArray[1] = randomVector2DArray[1];

         Vector2D vectorArray = new Vector2D(randomVector2DArray);

         Assert.assertTrue(randomVector2DArray[0] == vectorArray.getX());
         Assert.assertTrue(randomVector2DArray[1] == vectorArray.getY());

         Assert.assertTrue(copyRandomVector2DArray[0] == randomVector2DArray[0]);
         Assert.assertTrue(copyRandomVector2DArray[1] == randomVector2DArray[1]);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Vector2D(TupleBasics tuple)
         Vector2D vector2 = GeometryBasicsRandomTools.generateRandomVector2D(random);
         vector = new Vector2D((Tuple2DReadOnly<?>) vector2);

         Assert.assertTrue(vector.getX() == vector2.getX());
         Assert.assertTrue(vector.getY() == vector2.getY());
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

         Vector2D vector = new Vector2D(newX, newY);
         Vector2D vector2 = new Vector2D();
         vector2.set(vector);

         Assert.assertTrue(vector.getX() == vector2.getX());
         Assert.assertTrue(vector.getY() == vector2.getY());
      }
   }

   @Test
   public void testAngle()
   {
      Random random = new Random(56461L);
      Vector2D vector1 = new Vector2D();
      Vector2D vector2 = new Vector2D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         vector1 = GeometryBasicsRandomTools.generateRandomVector2D(random);
         vector2 = GeometryBasicsRandomTools.generateRandomVector2D(random);

         double actualAngle = vector1.angle(vector2);
         double dotProduct = vector1.dot(vector2);
         double magnitudeVector2D1 = vector1.length();
         double magnitudeVector2D2 = vector2.length();
         double expectedAngle = Math.acos(dotProduct / (magnitudeVector2D1 * magnitudeVector2D2));

         Assert.assertTrue(actualAngle == expectedAngle);
      }
   }

   @Test
   public void testCross()
   {
      Random random = new Random(5751684L);
      Vector2D vector1 = new Vector2D();
      Vector2D vector2 = new Vector2D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         vector1 = GeometryBasicsRandomTools.generateRandomVector2D(random);
         vector2 = GeometryBasicsRandomTools.generateRandomVector2D(random);

         { // Test cross(Vector2DBasics other)
            double actualMagnitudeVector3 = vector1.cross(vector2);
            double expectedMagnitudeVector3 = vector1.getX() * vector2.getY() - vector1.getY() * vector2.getX();

            Assert.assertEquals(expectedMagnitudeVector3, actualMagnitudeVector3, 1e-15);
         }

         { // Test cross(Vector2DBasics v1, Vector2DBasics v2)
            double actualMagnitudeVector3 = Vector2DReadOnly.cross(vector1, vector2);
            double expectedMagnitudeVector3 = vector1.getX() * vector2.getY() - vector1.getY() * vector2.getX();

            Assert.assertEquals(expectedMagnitudeVector3, actualMagnitudeVector3, 1e-15);
         }
      }
   }

   @Test
   public void testDot()
   {
      Random random = new Random(615651L);
      Vector2D vector1 = new Vector2D();
      Vector2D vector2 = new Vector2D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         vector1 = GeometryBasicsRandomTools.generateRandomVector2D(random);
         vector2 = GeometryBasicsRandomTools.generateRandomVector2D(random);

         double actualDot = vector1.dot(vector2);
         double expectedDot = vector1.getX() * vector2.getX() + vector1.getY() * vector2.getY();

         Assert.assertTrue(actualDot == expectedDot);
      }
   }

   @Test
   public void testLength()
   {
      Random random = new Random(312310L);
      Vector2D vector = new Vector2D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         vector = GeometryBasicsRandomTools.generateRandomVector2D(random);

         double length = vector.length();
         double expectedLengthSquared = vector.getX() * vector.getX() + vector.getY() * vector.getY();
         double expectedLength = Math.sqrt(expectedLengthSquared);

         Assert.assertTrue(length == expectedLength);
      }
   }

   @Test
   public void testLengthSquared()
   {
      Random random = new Random(312310L);
      Vector2D vector = new Vector2D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         vector = GeometryBasicsRandomTools.generateRandomVector2D(random);

         double lengthSquared = vector.lengthSquared();
         double expectedLengthSquared = vector.getX() * vector.getX() + vector.getY() * vector.getY();

         Assert.assertTrue(lengthSquared == expectedLengthSquared);
      }
   }

   @Test
   public void testNormalize()
   {
      Random random = new Random(312310L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test normalize()
         Vector2D vector = GeometryBasicsRandomTools.generateRandomVector2D(random);

         vector.normalize();

         double expectedLength = 1;
         double normalLength = vector.length();
         double difference = Math.abs(expectedLength - normalLength);

         Assert.assertTrue(difference < 1e-15);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test normalize(Vector2D vector)
         Vector2D vector = GeometryBasicsRandomTools.generateRandomVector2D(random);

         Vector2D normal = new Vector2D();

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
      double epsilon = random.nextDouble();
      Vector2D vector1 = GeometryBasicsRandomTools.generateRandomVector2D(random);
      Vector2D vector2 = new Vector2D();

      assertTrue(vector1.epsilonEquals(vector2, epsilon));

      for (int index = 0; index < 2; index++)
      {
         vector2.set(vector1);
         vector2.set(index, vector1.get(index) + 0.999 * epsilon);
         assertTrue(vector1.epsilonEquals(vector2, epsilon));

         vector2.set(vector1);
         vector2.set(index, vector1.get(index) - 0.999 * epsilon);
         assertTrue(vector1.epsilonEquals(vector2, epsilon));

         vector2.set(vector1);
         vector2.set(index, vector1.get(index) + 1.001 * epsilon);
         assertFalse(vector1.epsilonEquals(vector2, epsilon));

         vector2.set(vector1);
         vector2.set(index, vector1.get(index) - 1.001 * epsilon);
         assertFalse(vector1.epsilonEquals(vector2, epsilon));
      }
   }

   @Override
   public Vector2D createEmptyTuple()
   {
      return new Vector2D();
   }
}
