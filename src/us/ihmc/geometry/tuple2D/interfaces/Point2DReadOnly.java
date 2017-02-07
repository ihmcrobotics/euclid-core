package us.ihmc.geometry.tuple2D.interfaces;

public interface Point2DReadOnly<T extends Point2DReadOnly<T>> extends Tuple2DReadOnly<T>
{
   default double distance(Point2DReadOnly<?> other)
   {
      return Math.sqrt(distanceSquared(other));
   }

   default double distanceSquared(Point2DReadOnly<?> other)
   {
      double dx = getX() - other.getX();
      double dy = getY() - other.getY();
      return dx * dx + dy * dy;
   }
}
