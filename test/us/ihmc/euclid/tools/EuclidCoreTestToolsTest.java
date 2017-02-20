package us.ihmc.euclid.tools;

import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple4D.Quaternion;

public class EuclidCoreTestToolsTest
{
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 0.0001;

   @Test
   public void testAssertQuaternionEqualsUsingDifference() throws Exception
   {
      Random random = new Random(4270L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Quaternion q1 = EuclidCoreRandomTools.generateRandomQuaternion(random);
         Quaternion q2 = new Quaternion();

         Quaternion qDiff = new Quaternion();
         AxisAngle axisAngleDiff = new AxisAngle();

         double angle = EuclidCoreRandomTools.generateRandomDouble(random, EPSILON);
         axisAngleDiff.set(EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, 1.0), angle);
         qDiff.set(axisAngleDiff);
         q2.multiply(q1, qDiff);
         EuclidCoreTestTools.assertQuaternionEqualsUsingDifference(q1, q2, EPSILON);

         angle = 2.0 * Math.PI + EuclidCoreRandomTools.generateRandomDouble(random, EPSILON);
         axisAngleDiff.set(EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, 1.0), angle);
         qDiff.set(axisAngleDiff);
         q2.multiply(q1, qDiff);
         EuclidCoreTestTools.assertQuaternionEqualsUsingDifference(q1, q2, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Quaternion q1 = EuclidCoreRandomTools.generateRandomQuaternion(random);
         Quaternion q2 = new Quaternion();

         Quaternion qDiff = new Quaternion();
         AxisAngle axisAngleDiff = new AxisAngle();

         double delta = EuclidCoreRandomTools.generateRandomDouble(random, 5.0 * EPSILON);
         double angle = delta;
         axisAngleDiff.set(EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, 1.0), angle);
         qDiff.set(axisAngleDiff);
         q2.multiply(q1, qDiff);

         if (Math.abs(delta) < EPSILON)
         {
            EuclidCoreTestTools.assertQuaternionEqualsUsingDifference(q1, q2, EPSILON);
         }
         else
         {
            boolean hasThrownError = false;
            try
            {
               EuclidCoreTestTools.assertQuaternionEqualsUsingDifference(q1, q2, EPSILON);
            }
            catch (AssertionError e)
            {
               hasThrownError = true;
            }
            if (!hasThrownError)
               fail("Should have thrown an AssertionError");
         }

         angle = 2.0 * Math.PI + delta;
         axisAngleDiff.set(EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, 1.0), angle);
         qDiff.set(axisAngleDiff);
         q2.multiply(q1, qDiff);
         if (Math.abs(delta) < EPSILON)
         {
            EuclidCoreTestTools.assertQuaternionEqualsUsingDifference(q1, q2, EPSILON);
         }
         else
         {
            boolean hasThrownError = false;
            try
            {
               EuclidCoreTestTools.assertQuaternionEqualsUsingDifference(q1, q2, EPSILON);
            }
            catch (AssertionError e)
            {
               hasThrownError = true;
            }
            if (!hasThrownError)
               fail("Should have thrown an AssertionError");
         }

      }
   }

}
