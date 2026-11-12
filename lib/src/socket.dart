import 'math.dart';

/// Resource unit required to construct a modular piece.
final class BuildCostItem {
  final String resourceId;
  final double amount;

  const BuildCostItem(this.resourceId, this.amount);
}

/// Typed architectural attachment port on a [BuildPieceDef].
final class BuildSocket {
  final String id;
  final String socketType;
  final Set<String> compatibleTypes;
  final BuildVec3 localPosition;
  final BuildVec3 localForward;
  final BuildVec3 localUp;
  final bool requireOpposingForward;

  const BuildSocket({
    required this.id,
    required this.socketType,
    this.compatibleTypes = const {},
    required this.localPosition,
    this.localForward = BuildVec3.unitZ,
    this.localUp = BuildVec3.unitY,
    this.requireOpposingForward = false,
  });

  bool canConnectTo(BuildSocket other) {
    final aAcceptsB =
        compatibleTypes.isEmpty ? socketType == other.socketType : compatibleTypes.contains(other.socketType);
    final bAcceptsA =
        other.compatibleTypes.isEmpty ? other.socketType == socketType : other.compatibleTypes.contains(socketType);
    return aAcceptsB || bAcceptsA;
  }
}

/// World-space transformed [BuildSocket] belonging to a placed piece.
final class WorldBuildSocket {
  final int pieceInstanceId;
  final BuildSocket socket;
  final BuildVec3 worldPosition;
  final BuildVec3 worldForward;
  final BuildVec3 worldUp;

  const WorldBuildSocket({
    required this.pieceInstanceId,
    required this.socket,
    required this.worldPosition,
    required this.worldForward,
    required this.worldUp,
  });
}

/// Blueprint definition of a snap-compatible architectural building module.
final class BuildPieceDef {
  final String id;
  final String name;
  final String category;
  final BuildVec3 halfExtents;
  final List<BuildSocket> sockets;
  final double rotationStepDegrees;
  final bool requiresGrounded;
  final bool providesFoundationSupport;
  final double structuralLoadCapacity;
  final List<BuildCostItem> costs;

  const BuildPieceDef({
    required this.id,
    required this.name,
    this.category = 'structure',
    required this.halfExtents,
    this.sockets = const [],
    this.rotationStepDegrees = 90.0,
    this.requiresGrounded = false,
    this.providesFoundationSupport = false,
    this.structuralLoadCapacity = 100.0,
    this.costs = const [],
  });

  List<WorldBuildSocket> worldSockets(
    int instanceId,
    BuildVec3 worldPos,
    BuildQuat worldRot,
  ) {
    return [
      for (final s in sockets)
        WorldBuildSocket(
          pieceInstanceId: instanceId,
          socket: s,
          worldPosition: worldPos + worldRot.rotate(s.localPosition),
          worldForward: worldRot.rotate(s.localForward).normalized,
          worldUp: worldRot.rotate(s.localUp).normalized,
        ),
    ];
  }
}

/// Committed structural piece inside a [SnapGridSolver] world.
final class PlacedBuildPiece {
  final int instanceId;
  final String defId;
  final BuildVec3 position;
  final BuildQuat rotation;
  final int yawSteps;
  final bool directlyGrounded;
  bool supported;
  final Set<int> connectedPieceIds = {};

  PlacedBuildPiece({
    required this.instanceId,
    required this.defId,
    required this.position,
    required this.rotation,
    required this.yawSteps,
    required this.directlyGrounded,
    this.supported = true,
  });

  BuildObb toObb(BuildPieceDef def) => BuildObb(
        center: position,
        rotation: rotation,
        halfExtents: def.halfExtents,
      );
}
