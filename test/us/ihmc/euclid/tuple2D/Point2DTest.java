package us.ihmc.euclid.tuple2D;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Assert;
import org.junit.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;

public class Point2DTest extends Point2DBasicsTest<Point2D>
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
         Point2D pointArray = new Point2D(randomPoint2DArray);

         Assert.assertTrue(randomPoint2DArray[0] == pointArray.getX());
         Assert.assertTrue(randomPoint2DArray[1] == pointArray.getY());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Point2D(Tuple2DReadOnly tuple)
         Point2D point2 = createRandomTuple(random);
         point = new Point2D(point2);

         Assert.assertTrue(point.getX() == point2.getX());
         Assert.assertTrue(point.getY() == point2.getY());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Point2D(Tuple3DReadOnly tuple)
         Point3D point2 = EuclidCoreRandomTools.generateRandomPoint3D(random);
         point = new Point2D(point2);

         Assert.assertTrue(point.getX() == point2.getX());
         Assert.assertTrue(point.getY() == point2.getY());
      }
   }

   @Test
   public void testGeometricallyEquals() throws Exception {
      Point2D pointA;
      Point2D pointB;
      Random random = new Random(621541L);

      for (int i = 0; i < 100; ++i) {
         pointA = EuclidCoreRandomTools.generateRandomPoint2D(random);
         pointB = EuclidCoreRandomTools.generateRandomPoint2D(random);

         if (pointA.epsilonEquals(pointB, getEpsilon())) {
            assertTrue(pointA.geometricallyEquals(pointB, Math.sqrt(3)*getEpsilon()));
         } else {
            if (Math.sqrt(EuclidCoreTools.normSquared(pointA.getX() - pointB.getX(), pointA.getY() - pointB.getY())) <= getEpsilon()) {
               assertTrue(pointA.geometricallyEquals(pointB, getEpsilon()));
            } else {
               assertFalse(pointA.geometricallyEquals(pointB, getEpsilon()));
            }
         }
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      Point2D tuple1 = createRandomTuple(random);

      int newHashCode, previousHashCode;
      newHashCode = tuple1.hashCode();
      assertEquals(newHashCode, tuple1.hashCode());

      previousHashCode = tuple1.hashCode();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         tuple1.setElement(i % 2, random.nextDouble());
         newHashCode = tuple1.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-15;
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
      return EuclidCoreRandomTools.generateRandomPoint2D(random);
   }
}
