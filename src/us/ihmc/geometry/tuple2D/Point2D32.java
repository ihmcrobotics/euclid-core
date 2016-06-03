package us.ihmc.geometry.tuple2D;

import java.io.Serializable;

import us.ihmc.geometry.interfaces.GeometryObject;
import us.ihmc.geometry.transform.interfaces.Transform;
import us.ihmc.geometry.tuple2D.interfaces.Point2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;

public class Point2D32 extends Tuple2D32 implements Serializable, Point2DBasics, GeometryObject<Point2D32>
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

   public Point2D32(Tuple2DReadOnly tuple)
   {
      super(tuple);
   }

   @Override
   public void set(Point2D32 other)
   {
      super.set(other);
   }

   public final float distance(Point2DBasics other)
   {
      return (float) Math.sqrt(distanceSquared(other));
   }

   public final float distanceSquared(Point2DBasics other)
   {
      float dx = (float) (getX() - other.getX());
      float dy = (float) (getY() - other.getY());
      return dx * dx + dy * dy;
   }

   @Override
   public void applyTransform(Transform transform)
   {
      transform.transform(this);
   }

   @Override
   public boolean epsilonEquals(Point2D32 other, double epsilon)
   {
      return super.epsilonEquals(other, epsilon);
   }
}
