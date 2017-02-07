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

public class Point2DTest extends Tuple2DTest<Point2D>
{
   @Test
   public void testPoint2D()
   {
      Random random = new Random(621541L);
      Point2D point = new Point2D();

      { // Test Point2D()
         Assert.assertTrue(0 == point.getX());
         Assert.assertTrue(0 == point.getY());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Point2D(double x, double y)
         double newX = random.nextDouble();
         double newY = random.nextDouble();

         point = new Point2D(newX, newY);

         Assert.assertTrue(newX == point.getX());
         Assert.assertTrue(newY == point.getY());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Point2D(double[] pointArray)
         double[] randomPoint2DArray = {random.nextDouble(), random.nextDouble()};
         double[] copyRandomPoint2DArray = new double[2];
         copyRandomPoint2DArray[0] = randomPoint2DArray[0];
         copyRandomPoint2DArray[1] = randomPoint2DArray[1];

         Point2D pointArray = new Point2D(randomPoint2DArray);

         Assert.assertTrue(randomPoint2DArray[0] == pointArray.getX());
         Assert.assertTrue(randomPoint2DArray[1] == pointArray.getY());

         Assert.assertTrue(copyRandomPoint2DArray[0] == randomPoint2DArray[0]);
         Assert.assertTrue(copyRandomPoint2DArray[1] == randomPoint2DArray[1]);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Point2D(TupleBasics tuple)
         Point2D point2 = new Point2D();
         point2.setX(random.nextDouble());
         point2.setY(random.nextDouble());

         point = new Point2D(point2);

         Assert.assertTrue(point.getX() == point2.getX());
         Assert.assertTrue(point.getY() == point2.getY());
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

         Point2D point = new Point2D(newX, newY);

         Point2D point2 = new Point2D();
         point2.set(point);

         Assert.assertTrue(point.getX() == point2.getX());
         Assert.assertTrue(point.getY() == point2.getY());
      }
   }

   @Test
   public void testDistance()
   {
      Random random = new Random(654135L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double newX1 = random.nextDouble();
         double newY1 = random.nextDouble();

         double newX2 = random.nextDouble();
         double newY2 = random.nextDouble();

         Point2D point = new Point2D(newX1, newY1);
         Point2D point2 = new Point2D(newX2, newY2);

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
         double newX1 = random.nextDouble();
         double newY1 = random.nextDouble();

         double newX2 = random.nextDouble();
         double newY2 = random.nextDouble();

         Point2D point = new Point2D(newX1, newY1);
         Point2D point2 = new Point2D(newX2, newY2);

         double distance = point.distanceSquared((Point2DBasics<?>) point2);
         double dx = point.getX() - point2.getX();
         double dy = point.getY() - point2.getY();
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
      double epsilon = random.nextDouble();

      Point2D p1 = GeometryBasicsRandomTools.generateRandomPoint2D(random);
      Point2D p2 = new Point2D(p1);

      assertTrue(p1.epsilonEquals(p2, epsilon));

      for (int index = 0; index < 2; index++)
      {
         p2.set(p1);
         p2.set(index, p1.get(index) + 0.999 * epsilon);
         assertTrue(p1.epsilonEquals(p2, epsilon));

         p2.set(p1);
         p2.set(index, p1.get(index) - 0.999 * epsilon);
         assertTrue(p1.epsilonEquals(p2, epsilon));

         p2.set(p1);
         p2.set(index, p1.get(index) + 1.001 * epsilon);
         assertFalse(p1.epsilonEquals(p2, epsilon));

         p2.set(p1);
         p2.set(index, p1.get(index) - 1.001 * epsilon);
         assertFalse(p1.epsilonEquals(p2, epsilon));
      }
   }

   @Override
   public Point2D createEmptyTuple()
   {
      return new Point2D();
   }

   @Override
   public Point2D createTuple(double x, double y)
   {
      return new Point2D(x, y);
   }

   @Override
   public Point2D createRandomTuple(Random random)
   {
      return GeometryBasicsRandomTools.generateRandomPoint2D(random);
   }
}
