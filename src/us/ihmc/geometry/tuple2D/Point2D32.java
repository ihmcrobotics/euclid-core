package us.ihmc.geometry.tuple2D;

import java.io.Serializable;

import us.ihmc.geometry.tuple2D.interfaces.Point2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;

public class Point2D32 extends Tuple2D32<Point2D32> implements Serializable, Point2DBasics<Point2D32>
{
   private static final long serialVersionUID = -4681227587308478166L;

   public Point2D32()
   {
      super();
   }

   public Point2D32(float x, float y)
   {
      super(x, y);
   }

   public Point2D32(float[] pointArray)
   {
      super(pointArray);
   }

   public Point2D32(Tuple2DReadOnly<?> tuple)
   {
      super(tuple);
   }
}
