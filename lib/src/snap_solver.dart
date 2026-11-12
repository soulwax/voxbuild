import 'dart:collection';
import 'dart:math' as math;

import 'math.dart';
import 'socket.dart';

/// Placement legality result produced by [SnapGridSolver.evaluatePlacement].
enum BuildPlacementValidity {
  valid,
  unknownPiece,
  blockedOverlap,
  ungrounded,
  incompatibleSocket,
}

/// Candidate snapped world transform & diagnostic state prior to committing a piece.
final class BuildSnapCandidate {
  final String defId;
  final BuildVec3 position;
  final BuildQuat rotation;
  final int yawSteps;
  final int? snappedToPieceId;
  final String? snappedToSocketId;
  final String? sourceSocketId;
  final double snapDistance;
  final BuildPlacementValidity validity;
  final int? overlapPieceId;

  const BuildSnapCandidate({
    required this.defId,
    required this.position,
    required this.rotation,
    required this.yawSteps,
    this.snappedToPieceId,
    this.snappedToSocketId,
    this.sourceSocketId,
    required this.snapDistance,
    required this.validity,
    this.overlapPieceId,
  });

  bool get isValid => validity == BuildPlacementValidity.valid;
}

/// Evaluates magnetic socket snapping, quantized grids, SAT OBB clearance & structural grounding.
final class SnapGridSolver {
  double gridSize;
  double heightStep;
  double socketSnapRadius;
  double clearanceMargin;

  final Map<String, BuildPieceDef> _defs = {};
  final Map<int, PlacedBuildPiece> _placed = {};
  int _nextId = 1;

  SnapGridSolver({
    this.gridSize = 2.0,
    this.heightStep = 1.5,
    this.socketSnapRadius = 1.25,
    this.clearanceMargin = 0.02,
  });

  Map<String, BuildPieceDef> get definitions => Map.unmodifiable(_defs);
  Map<int, PlacedBuildPiece> get placedPieces => Map.unmodifiable(_placed);

  void registerPiece(BuildPieceDef def) {
    _defs[def.id] = def;
  }

  void clear() {
    _placed.clear();
  }

  double _snapAxis(double v, double step) =>
      step <= 1e-6 ? v : (v / step).roundToDouble() * step;

  /// Evaluates socket alignment + fallback quantized grid + clearance + grounding at [cursorWorld].
  BuildSnapCandidate evaluatePlacement({
    required String defId,
    required BuildVec3 cursorWorld,
    int yawSteps = 0,
    int? preferredTargetPieceId,
    double groundHeight = 0.0,
    double groundTolerance = 0.08,
    bool ignoreGroundCheck = false,
  }) {
    final def = _defs[defId];
    if (def == null) {
      return BuildSnapCandidate(
        defId: defId,
        position: cursorWorld,
        rotation: BuildQuat.identity,
        yawSteps: yawSteps,
        snapDistance: 0.0,
        validity: BuildPlacementValidity.unknownPiece,
      );
    }

    final rotDeg = yawSteps * def.rotationStepDegrees;
    var chosenRot = BuildQuat.fromYawDegrees(rotDeg);
    var chosenPos = BuildVec3(
      _snapAxis(cursorWorld.x, gridSize),
      groundHeight + _snapAxis(cursorWorld.y - groundHeight, heightStep),
      _snapAxis(cursorWorld.z, gridSize),
    );
    int? snappedPieceId;
    String? snappedSocketId;
    String? sourceSocketId;
    var bestDist = double.infinity;

    // Socket-to-socket search against nearby placed pieces.
    if (def.sockets.isNotEmpty && _placed.isNotEmpty) {
      for (final existing in _placed.values) {
        if (preferredTargetPieceId != null && existing.instanceId != preferredTargetPieceId) {
          continue;
        }
        final existingDef = _defs[existing.defId];
        if (existingDef == null || existingDef.sockets.isEmpty) continue;
        final worldSockets = existingDef.worldSockets(
          existing.instanceId,
          existing.position,
          existing.rotation,
        );

        for (final targetWs in worldSockets) {
          final distCursorToSocket = cursorWorld.distanceTo(targetWs.worldPosition);
          if (distCursorToSocket > socketSnapRadius + def.halfExtents.length) continue;

          for (final srcSock in def.sockets) {
            if (!srcSock.canConnectTo(targetWs.socket)) continue;

            // Compute aligned candidate rotation if socket requires opposing or matching forward,
            // otherwise use player-specified yawSteps.
            var candRot = BuildQuat.fromYawDegrees(rotDeg);
            if (srcSock.requireOpposingForward) {
              final targetYaw = math.atan2(-targetWs.worldForward.x, -targetWs.worldForward.z);
              final srcYaw = math.atan2(srcSock.localForward.x, srcSock.localForward.z);
              candRot = BuildQuat.fromYawRadians(targetYaw - srcYaw);
            }
            final candCenter = targetWs.worldPosition - candRot.rotate(srcSock.localPosition);
            final scoreDist = cursorWorld.distanceTo(targetWs.worldPosition);

            // Prefer candidates that don't overlap when multiple sockets sit close together.
            final candObb = BuildObb(
              center: candCenter,
              rotation: candRot,
              halfExtents: def.halfExtents,
            ).shrunk(clearanceMargin);
            final hasOverlap = _findOverlap(candObb) != null;

            final effectiveScore = hasOverlap ? scoreDist + 1000.0 : scoreDist;
            if (effectiveScore < bestDist) {
              bestDist = effectiveScore;
              chosenPos = candCenter;
              chosenRot = candRot;
              snappedPieceId = existing.instanceId;
              snappedSocketId = targetWs.socket.id;
              sourceSocketId = srcSock.id;
            }
          }
        }
      }
    }

    final finalObb = BuildObb(
      center: chosenPos,
      rotation: chosenRot,
      halfExtents: def.halfExtents,
    );
    final overlapId = _findOverlap(finalObb.shrunk(clearanceMargin));
    if (overlapId != null) {
      return BuildSnapCandidate(
        defId: defId,
        position: chosenPos,
        rotation: chosenRot,
        yawSteps: yawSteps,
        snappedToPieceId: snappedPieceId,
        snappedToSocketId: snappedSocketId,
        sourceSocketId: sourceSocketId,
        snapDistance: bestDist.isFinite ? bestDist : 0.0,
        validity: BuildPlacementValidity.blockedOverlap,
        overlapPieceId: overlapId,
      );
    }

    final touchesGround = finalObb.worldMinY <= groundHeight + groundTolerance;
    final attachedToSupported =
        snappedPieceId != null && (_placed[snappedPieceId]?.supported ?? false);
    if (!ignoreGroundCheck && def.requiresGrounded && !touchesGround && !attachedToSupported) {
      return BuildSnapCandidate(
        defId: defId,
        position: chosenPos,
        rotation: chosenRot,
        yawSteps: yawSteps,
        snappedToPieceId: snappedPieceId,
        snappedToSocketId: snappedSocketId,
        sourceSocketId: sourceSocketId,
        snapDistance: bestDist.isFinite ? bestDist : 0.0,
        validity: BuildPlacementValidity.ungrounded,
      );
    }

    return BuildSnapCandidate(
      defId: defId,
      position: chosenPos,
      rotation: chosenRot,
      yawSteps: yawSteps,
      snappedToPieceId: snappedPieceId,
      snappedToSocketId: snappedSocketId,
      sourceSocketId: sourceSocketId,
      snapDistance: bestDist.isFinite ? bestDist : 0.0,
      validity: BuildPlacementValidity.valid,
    );
  }

  int? _findOverlap(BuildObb testObb) {
    for (final existing in _placed.values) {
      final def = _defs[existing.defId];
      if (def == null) continue;
      if (testObb.overlaps(existing.toObb(def).shrunk(clearanceMargin))) {
        return existing.instanceId;
      }
    }
    return null;
  }

  /// Validates and places a building piece onto the grid, wiring socket connectivity.
  PlacedBuildPiece? place({
    required String defId,
    required BuildVec3 cursorWorld,
    int yawSteps = 0,
    int? preferredTargetPieceId,
    double groundHeight = 0.0,
    double groundTolerance = 0.08,
    bool ignoreGroundCheck = false,
  }) {
    final cand = evaluatePlacement(
      defId: defId,
      cursorWorld: cursorWorld,
      yawSteps: yawSteps,
      preferredTargetPieceId: preferredTargetPieceId,
      groundHeight: groundHeight,
      groundTolerance: groundTolerance,
      ignoreGroundCheck: ignoreGroundCheck,
    );
    if (!cand.isValid) return null;
    return placeExact(
      defId: cand.defId,
      position: cand.position,
      rotation: cand.rotation,
      yawSteps: cand.yawSteps,
      groundHeight: groundHeight,
      groundTolerance: groundTolerance,
    );
  }

  /// Inserts a pre-validated piece directly (used by blueprints or server replication).
  PlacedBuildPiece? placeExact({
    required String defId,
    required BuildVec3 position,
    required BuildQuat rotation,
    int yawSteps = 0,
    double groundHeight = 0.0,
    double groundTolerance = 0.08,
    bool skipOverlapCheck = false,
  }) {
    final def = _defs[defId];
    if (def == null) return null;
    final obb = BuildObb(
      center: position,
      rotation: rotation,
      halfExtents: def.halfExtents,
    );
    if (!skipOverlapCheck && _findOverlap(obb.shrunk(clearanceMargin)) != null) {
      return null;
    }

    final touchesGround = obb.worldMinY <= groundHeight + groundTolerance;
    final piece = PlacedBuildPiece(
      instanceId: _nextId++,
      defId: defId,
      position: position,
      rotation: rotation,
      yawSteps: yawSteps,
      directlyGrounded: def.providesFoundationSupport || touchesGround,
    );
    _placed[piece.instanceId] = piece;
    _linkNeighbors(piece, def);
    recomputeSupport();
    return piece;
  }

  void _linkNeighbors(PlacedBuildPiece piece, BuildPieceDef def) {
    final aSockets = def.worldSockets(piece.instanceId, piece.position, piece.rotation);
    for (final other in _placed.values) {
      if (other.instanceId == piece.instanceId) continue;
      final otherDef = _defs[other.defId];
      if (otherDef == null) continue;
      final bSockets = otherDef.worldSockets(other.instanceId, other.position, other.rotation);

      var linked = false;
      for (final wsA in aSockets) {
        for (final wsB in bSockets) {
          if (wsA.worldPosition.distanceTo(wsB.worldPosition) <= 0.15 &&
              wsA.socket.canConnectTo(wsB.socket)) {
            linked = true;
            break;
          }
        }
        if (linked) break;
      }

      // Also check flush face contact within 0.08m if no explicit socket touched
      if (!linked &&
          !piece.toObb(def).shrunk(0.02).overlaps(other.toObb(otherDef).shrunk(0.02)) &&
          piece.toObb(def).shrunk(-0.08).overlaps(other.toObb(otherDef).shrunk(-0.08))) {
        linked = true;
      }

      if (linked) {
        piece.connectedPieceIds.add(other.instanceId);
        other.connectedPieceIds.add(piece.instanceId);
      }
    }
  }

  bool removePiece(int instanceId) {
    final removed = _placed.remove(instanceId);
    if (removed == null) return false;
    for (final neighborId in removed.connectedPieceIds) {
      _placed[neighborId]?.connectedPieceIds.remove(instanceId);
    }
    recomputeSupport();
    return true;
  }

  /// Flood-fills support from directly grounded foundations; returns IDs of unsupported floating pieces.
  List<int> recomputeSupport() {
    for (final p in _placed.values) {
      p.supported = false;
    }
    final queue = Queue<PlacedBuildPiece>();
    for (final p in _placed.values) {
      if (p.directlyGrounded) {
        p.supported = true;
        queue.add(p);
      }
    }
    while (queue.isNotEmpty) {
      final cur = queue.removeFirst();
      for (final nid in cur.connectedPieceIds) {
        final neighbor = _placed[nid];
        if (neighbor != null && !neighbor.supported) {
          neighbor.supported = true;
          queue.add(neighbor);
        }
      }
    }
    return [
      for (final p in _placed.values)
        if (!p.supported) p.instanceId,
    ];
  }
}
