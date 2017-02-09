package us.ihmc.geometry.tuple4D.interfaces;

/**
 * Read-only interface for a 4 dimensional vector representing a generic quaternion.
 * <p>
 * When describing a 4D tuple, its 4 components are often gathered in two groups: the scalar part {@code s}
 * and the vector part ({@code x}, {@code y}, {@code z}).
 * </p>
 * <p>
 * Note on the difference between applying a 3D transform on a quaternion and a 4D vector:
 * <ul>
 *    <li> When transformed by a homogeneous transformation matrix, a quaternion is only
 *     pre-multiplied by the rotation part of the transform, resulting in concatenating
 *     the orientations of the transform and the quaternion.
 *    <li> When transformed by a homogeneous transformation matrix, a 4D vector scalar
 *     part {@code s} remains unchanged. The vector part ({@code x}, {@code y}, {@code z})
 *     is scaled and rotated, and translated by {@code s} times the translation part of the transform.
 *     Note that for {@code s = 0}, a 4D vector behaves as a 3D vector, and for {@code s = 1}
 *     it behaves as a 3D point.
 * </ul>
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 * @param <T> The final type of the vector used.
 */
public interface Vector4DReadOnly<T extends Vector4DReadOnly<T>> extends Tuple4DReadOnly<T>
{
}
