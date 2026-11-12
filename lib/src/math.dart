import 'dart:math' as math;

/// Immutable 3D vector for modular building math.
final class BuildVec3 {
  final double x;
  final double y;
  final double z;

  const BuildVec3(this.x, this.y, this.z);

  static const zero = BuildVec3(0.0, 0.0, 0.0);
  static const unitX = BuildVec3(1.0, 0.0, 0.0);
  static const unitY = BuildVec3(0.0, 1.0, 0.0);
  static const unitZ = BuildVec3(0.0, 0.0, 1.0);

  BuildVec3 operator +(BuildVec3 o) => BuildVec3(x + o.x, y + o.y, z + o.z);
  BuildVec3 operator -(BuildVec3 o) => BuildVec3(x - o.x, y - o.y, z - o.z);
  BuildVec3 operator -() => BuildVec3(-x, -y, -z);
  BuildVec3 operator *(double s) => BuildVec3(x * s, y * s, z * s);
  BuildVec3 operator /(double s) => BuildVec3(x / s, y / s, z / s);

  double dot(BuildVec3 o) => x * o.x + y * o.y + z * o.z;

  BuildVec3 cross(BuildVec3 o) => BuildVec3(
        y * o.z - z * o.y,
        z * o.x - x * o.z,
        x * o.y - y * o.x,
      );

  double get lengthSquared => x * x + y * y + z * z;
  double get length => math.sqrt(lengthSquared);

  double distanceTo(BuildVec3 o) => (this - o).length;

  BuildVec3 get normalized {
    final len = length;
    if (len < 1e-12) return zero;
    return this / len;
  }

  Map<String, double> toJson() => {'x': x, 'y': y, 'z': z};

  factory BuildVec3.fromJson(Map<String, Object?> raw) => BuildVec3(
        (raw['x'] as num).toDouble(),
        (raw['y'] as num).toDouble(),
        (raw['z'] as num).toDouble(),
      );

  @override
  String toString() =>
      'BuildVec3(${x.toStringAsFixed(3)}, ${y.toStringAsFixed(3)}, ${z.toStringAsFixed(3)})';
}

/// Unit quaternion for piece orientation & socket transforms.
final class BuildQuat {
  final double x;
  final double y;
  final double z;
  final double w;

  const BuildQuat(this.x, this.y, this.z, this.w);

  static const identity = BuildQuat(0.0, 0.0, 0.0, 1.0);

  factory BuildQuat.fromAxisAngle(BuildVec3 axis, double angleRadians) {
    final n = axis.normalized;
    final half = angleRadians * 0.5;
    final s = math.sin(half);
    return BuildQuat(n.x * s, n.y * s, n.z * s, math.cos(half));
  }

  factory BuildQuat.fromYawRadians(double yawRadians) =>
      BuildQuat.fromAxisAngle(BuildVec3.unitY, yawRadians);

  factory BuildQuat.fromYawDegrees(double yawDegrees) =>
      BuildQuat.fromYawRadians(yawDegrees * math.pi / 180.0);

  BuildQuat get normalized {
    final len = math.sqrt(x * x + y * y + z * z + w * w);
    if (len < 1e-12) return identity;
    final inv = 1.0 / len;
    return BuildQuat(x * inv, y * inv, z * inv, w * inv);
  }

  BuildQuat get conjugate => BuildQuat(-x, -y, -z, w);

  BuildQuat operator *(BuildQuat o) => BuildQuat(
        w * o.x + x * o.w + y * o.z - z * o.y,
        w * o.y - x * o.z + y * o.w + z * o.x,
        w * o.z + x * o.y - y * o.x + z * o.w,
        w * o.w - x * o.x - y * o.y - z * o.z,
      );

  BuildVec3 rotate(BuildVec3 v) {
    final qv = BuildVec3(x, y, z);
    final t = qv.cross(v) * 2.0;
    return v + t * w + qv.cross(t);
  }

  double get yawRadians =>
      math.atan2(2.0 * (w * y + x * z), 1.0 - 2.0 * (y * y + x * x));
}

/// Oriented Bounding Box supporting full 15-axis Separating Axis Theorem (SAT) overlap checks.
final class BuildObb {
  final BuildVec3 center;
  final BuildQuat rotation;
  final BuildVec3 halfExtents;

  const BuildObb({
    required this.center,
    required this.rotation,
    required this.halfExtents,
  });

  BuildObb shrunk(double margin) => BuildObb(
        center: center,
        rotation: rotation,
        halfExtents: BuildVec3(
          math.max(0.001, halfExtents.x - margin),
          math.max(0.001, halfExtents.y - margin),
          math.max(0.001, halfExtents.z - margin),
        ),
      );

  /// Conservative axis-aligned minimum Y coordinate in world space.
  double get worldMinY {
    final ax = rotation.rotate(BuildVec3.unitX);
    final ay = rotation.rotate(BuildVec3.unitY);
    final az = rotation.rotate(BuildVec3.unitZ);
    final ry = ax.y.abs() * halfExtents.x +
        ay.y.abs() * halfExtents.y +
        az.y.abs() * halfExtents.z;
    return center.y - ry;
  }

  /// Exact 15-axis SAT overlap test against [other].
  bool overlaps(BuildObb other, {double tolerance = 1e-5}) {
    final aAxes = [
      rotation.rotate(BuildVec3.unitX).normalized,
      rotation.rotate(BuildVec3.unitY).normalized,
      rotation.rotate(BuildVec3.unitZ).normalized,
    ];
    final bAxes = [
      other.rotation.rotate(BuildVec3.unitX).normalized,
      other.rotation.rotate(BuildVec3.unitY).normalized,
      other.rotation.rotate(BuildVec3.unitZ).normalized,
    ];
    final aExt = [halfExtents.x, halfExtents.y, halfExtents.z];
    final bExt = [other.halfExtents.x, other.halfExtents.y, other.halfExtents.z];
    final diff = other.center - center;

    bool separatedOn(BuildVec3 axis) {
      final lenSq = axis.lengthSquared;
      if (lenSq < 1e-10) return false;
      final n = axis / math.sqrt(lenSq);
      final dist = diff.dot(n).abs();
      final ra = aExt[0] * aAxes[0].dot(n).abs() +
          aExt[1] * aAxes[1].dot(n).abs() +
          aExt[2] * aAxes[2].dot(n).abs();
      final rb = bExt[0] * bAxes[0].dot(n).abs() +
          bExt[1] * bAxes[1].dot(n).abs() +
          bExt[2] * bAxes[2].dot(n).abs();
      return dist >= ra + rb - tolerance;
    }

    for (var i = 0; i < 3; i++) {
      if (separatedOn(aAxes[i])) return false;
      if (separatedOn(bAxes[i])) return false;
    }
    for (var i = 0; i < 3; i++) {
      for (var j = 0; j < 3; j++) {
        if (separatedOn(aAxes[i].cross(bAxes[j]))) return false;
      }
    }
    return true;
  }
}
