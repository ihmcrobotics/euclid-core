package us.ihmc.geometry.compiler;

import java.util.Random;

import org.junit.Test;

import us.ihmc.geometry.transform.RigidBodyTransform;
import us.ihmc.geometry.tuple.Vector;

/**
 * This test is designed to measure the speed of transform code
 * after it has been compiled by the JVM.
 * 
 * @author Duncan Calvert <a href="mailto:dcalvert@ihmc.us">(dcalvert@ihmc.us)</a>
 */
public class CompiledTransformPerformanceTest
{
   @Test
   public void testTransformingAVectorManyTimes()
   {
      Random random = new Random(1230930210L);

      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      rigidBodyTransform.setRotationYawPitchRollAndZeroTranslation(2.0 * Math.PI * random.nextDouble(), 2.0 * Math.PI * random.nextDouble(),
                                                                   2.0 * Math.PI * random.nextDouble());
      rigidBodyTransform.setTranslation(10000.0 * (random.nextDouble() - 0.5), 10000.0 * (random.nextDouble() - 0.5), 10000.0 * (random.nextDouble() - 0.5));
      
      Vector vector = new Vector();
      vector.setX(10000.0 * (random.nextDouble() - 0.5));
      vector.setY(10000.0 * (random.nextDouble() - 0.5));
      vector.setZ(10000.0 * (random.nextDouble() - 0.5));
      
      int n = 80;
      
      String[] vectorOutput = new String[n];
      
      for (int i = 0; i < n; i++)
      {
         long nanoTime = tranformVectorNTimes(rigidBodyTransform, vector, 200000);
         vectorOutput[i] = vector.toString();
         System.out.println(nanoTime);
      }
      
      for (int i = 0; i < vectorOutput.length; i++)
      {
         System.out.println("Vector: " + vectorOutput[i]);
      }
   }
   
   private long tranformVectorNTimes(RigidBodyTransform rigidBodyTransform, Vector vector, int n)
   {
      long start = System.nanoTime();
      
      for (int i = 0; i < n / 10; i++)
      {
         rigidBodyTransform.transform(vector);
         rigidBodyTransform.transform(vector);
         rigidBodyTransform.transform(vector);
         rigidBodyTransform.transform(vector);
         rigidBodyTransform.transform(vector);
         rigidBodyTransform.transform(vector);
         rigidBodyTransform.transform(vector);
         rigidBodyTransform.transform(vector);
         rigidBodyTransform.transform(vector);
         rigidBodyTransform.transform(vector);
      }
      
      long end = System.nanoTime();
      
      return end - start;
   }
}
