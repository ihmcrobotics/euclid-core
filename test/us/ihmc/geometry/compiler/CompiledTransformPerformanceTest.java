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
   public void testTransformingAVector200000Times()
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
      
      for (int i = 0; i < 20; i++)
      {
         System.out.println("Run " + i + ":");
         tranformVector10000Times(rigidBodyTransform, vector);
      }
   }
   
   private void tranformVector10000Times(RigidBodyTransform rigidBodyTransform, Vector vector)
   {
      long start = System.nanoTime();
      
      for (int i = 0; i < 10000; i++)
      {
         rigidBodyTransform.transform(vector);
      }
      
      long end = System.nanoTime();
      
      System.out.println("Vector: " + vector);
      System.out.println("Time: " + (end - start) / (double) 1e-9 + "(s)");
   }
}
