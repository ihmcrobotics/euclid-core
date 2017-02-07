package us.ihmc.geometry.tuple2D;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Assert;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.tuple2D.interfaces.Point2DBasics;

public class Point2D32Test extends Tuple2D32Test<Point2D32>
{
   @Test
   public void testPoint2D32()
   {
      Random random = new Random(621541L);
      Point2D32 point = new Point2D32();

      { // Test Point2D32()
         Assert.assertTrue(0f == point.getX32());
         Assert.assertTrue(0f == point.getY32());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Point2D32(float x, float y)
         float newX = random.nextFloat();
         float newY = random.nextFloat();

         point = new Point2D32(newX, newY);

         Assert.assertTrue(newX == point.getX32());
         Assert.assertTrue(newY == point.getY32());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Point2D32(float[] pointArray)
         float[] randomPoint2D32Array = {random.nextFloat(), random.nextFloat()};
         float[] copyRandomPoint2D32Array = new float[2];
         copyRandomPoint2D32Array[0] = randomPoint2D32Array[0];
         copyRandomPoint2D32Array[1] = randomPoint2D32Array[1];

         Point2D32 pointArray = new Point2D32(randomPoint2D32Array);

         Assert.assertTrue(randomPoint2D32Array[0] == pointArray.getX32());
         Assert.assertTrue(randomPoint2D32Array[1] == pointArray.getY32());

         Assert.assertTrue(copyRandomPoint2D32Array[0] == randomPoint2D32Array[0]);
         Assert.assertTrue(copyRandomPoint2D32Array[1] == randomPoint2D32Array[1]);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Point2D32(TupleBasics tuple)
         Point2D32 point2 = GeometryBasicsRandomTools.generateRandomPoint2D32(random);

         point = new Point2D32(point2);

         Assert.assertTrue(point.getX32() == point2.getX32());
         Assert.assertTrue(point.getY32() == point2.getY32());
      }
   }

   @Test
   public void testSet()
   {
      Random random = new Random(65465131L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         float newX = random.nextFloat();
         float newY = random.nextFloat();

         Point2D32 point = new Point2D32(newX, newY);

         Point2D32 point2 = new Point2D32();
         point2.set(point);

         Assert.assertTrue(point.getX32() == point2.getX32());
         Assert.assertTrue(point.getY32() == point2.getY32());
      }
   }

   @Test
   public void testDistance()
   {
      Random random = new Random(654135L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         float newX1 = random.nextFloat();
         float newY1 = random.nextFloat();

         float newX2 = random.nextFloat();
         float newY2 = random.nextFloat();

         Point2D32 point = new Point2D32(newX1, newY1);

         Point2D32 point2 = new Point2D32(newX2, newY2);

         double distance = Math.sqrt(point.distanceSquared((Point2DBasics<?>) point2));
         double distance2 = point.distance(point2);

         Assert.assertTrue(distance == distance2);
      }
   }

   @Test
   @Ignore
   public void testDistanceL1()
   {
      fail("Not yet implemented");
   }

   @Test
   @Ignore
   public void testDistanceLinf()
   {
      fail("Not yet implemented");
   }

   @Test
   public void testDistanceSquared()
   {
      Random random = new Random(654135L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         float newX1 = random.nextFloat();
         float newY1 = random.nextFloat();

         float newX2 = random.nextFloat();
         float newY2 = random.nextFloat();

         Point2D32 point = new Point2D32(newX1, newY1);
         Point2D32 point2 = new Point2D32(newX2, newY2);

         double distance = point.distanceSquared(point2);
         double dx = point.getX32() - point2.getX32();
         double dy = point.getY32() - point2.getY32();
         double expectedDistance = dx * dx + dy * dy;

         Assert.assertTrue(distance == expectedDistance);
      }
   }

   @Test
   @Ignore
   public void testApplyTransform()
   {
      fail("Not yet implemented");
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(621541L);
      float epsilon = random.nextFloat();

      Point2D32 p1 = GeometryBasicsRandomTools.generateRandomPoint2D32(random);
      Point2D32 p2 = new Point2D32(p1);

      assertTrue(p1.epsilonEquals(p2, epsilon));

      for (int index = 0; index < 2; index++)
      {
         p2.set(p1);
         p2.set(index, p1.get(index) + 0.999f * epsilon);
         assertTrue(p1.epsilonEquals(p2, epsilon));

         p2.set(p1);
         p2.set(index, p1.get(index) - 0.999f * epsilon);
         assertTrue(p1.epsilonEquals(p2, epsilon));

         p2.set(p1);
         p2.set(index, p1.get(index) + 1.001f * epsilon);
         assertFalse(p1.epsilonEquals(p2, epsilon));

         p2.set(p1);
         p2.set(index, p1.get(index) - 1.001f * epsilon);
         assertFalse(p1.epsilonEquals(p2, epsilon));
      }
   }

   @Override
   public Point2D32 createEmptyTuple()
   {
      return new Point2D32();
   }

   @Override
   public Point2D32 createTuple(double x, double y)
   {
      return new Point2D32((float) x, (float) y);
   }

   @Override
   public Point2D32 createRandomTuple(Random random)
   {
      return GeometryBasicsRandomTools.generateRandomPoint2D32(random);
   }
}
