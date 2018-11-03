package us.ihmc.euclid.yawPitchRoll;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;

public class YawPitchRollTest extends YawPitchRollBasicsTest<YawPitchRoll>
{
   @Override
   public YawPitchRoll createEmptyYawPitchRoll()
   {
      return new YawPitchRoll();
   }

   @Override
   public YawPitchRoll createRandomYawPitchRoll(Random random)
   {
      return EuclidCoreRandomTools.nextYawPitchRoll(random);
   }

   @Override
   public YawPitchRoll createYawPitchRoll(double yaw, double pitch, double roll)
   {
      return new YawPitchRoll(yaw, pitch, roll);
   }

   @Override
   public YawPitchRoll createYawPitchRoll(Orientation3DReadOnly orientation3D)
   {
      return new YawPitchRoll(orientation3D);
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-13;
   }

   @Override
   public double getSmallestEpsilon()
   {
      return 1.0e-15;
   }
}
