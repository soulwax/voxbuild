import 'math.dart';
import 'snap_solver.dart';
import 'socket.dart';

/// Single piece entry inside a reusable [ConstructionBlueprint].
final class BlueprintPieceEntry {
  final String defId;
  final BuildVec3 localPosition;
  final int yawSteps;
  final double rotationStepDegrees;

  const BlueprintPieceEntry({
    required this.defId,
    required this.localPosition,
    this.yawSteps = 0,
    this.rotationStepDegrees = 90.0,
  });

  Map<String, Object?> toJson() => {
        'defId': defId,
        'localPosition': localPosition.toJson(),
        'yawSteps': yawSteps,
        'rotationStepDegrees': rotationStepDegrees,
      };

  factory BlueprintPieceEntry.fromJson(Map<String, Object?> raw) => BlueprintPieceEntry(
        defId: raw['defId'] as String,
        localPosition: BuildVec3.fromJson(raw['localPosition'] as Map<String, Object?>),
        yawSteps: (raw['yawSteps'] as num?)?.toInt() ?? 0,
        rotationStepDegrees: (raw['rotationStepDegrees'] as num?)?.toDouble() ?? 90.0,
      );
}

/// Serializable compound structure template supporting rotated multi-piece stamping and cost tallying.
final class ConstructionBlueprint {
  final String name;
  final List<BlueprintPieceEntry> entries;

  const ConstructionBlueprint({
    required this.name,
    required this.entries,
  });

  /// Captures placed pieces relative to [origin] (or the first piece's XZ center / minY if omitted).
  factory ConstructionBlueprint.capture({
    required String name,
    required Iterable<PlacedBuildPiece> pieces,
    required Map<String, BuildPieceDef> definitions,
    BuildVec3? origin,
  }) {
    final list = pieces.toList();
    if (list.isEmpty) {
      return ConstructionBlueprint(name: name, entries: const []);
    }
    final anchor = origin ?? BuildVec3(list.first.position.x, 0.0, list.first.position.z);
    return ConstructionBlueprint(
      name: name,
      entries: [
        for (final p in list)
          BlueprintPieceEntry(
            defId: p.defId,
            localPosition: p.position - anchor,
            yawSteps: p.yawSteps,
            rotationStepDegrees: definitions[p.defId]?.rotationStepDegrees ?? 90.0,
          ),
      ],
    );
  }

  /// Sums all construction resource requirements across every entry in this blueprint.
  Map<String, double> totalCosts(Map<String, BuildPieceDef> definitions) {
    final totals = <String, double>{};
    for (final e in entries) {
      final def = definitions[e.defId];
      if (def == null) continue;
      for (final c in def.costs) {
        totals[c.resourceId] = (totals[c.resourceId] ?? 0.0) + c.amount;
      }
    }
    return totals;
  }

  /// Stamps the entire rotated compound blueprint into [solver] at [worldOrigin], returning placed pieces.
  List<PlacedBuildPiece> stamp(
    SnapGridSolver solver, {
    required BuildVec3 worldOrigin,
    int yawSteps = 0,
    double blueprintStepDegrees = 90.0,
    double groundHeight = 0.0,
  }) {
    final baseRot = BuildQuat.fromYawDegrees(yawSteps * blueprintStepDegrees);
    final placed = <PlacedBuildPiece>[];
    for (final entry in entries) {
      final def = solver.definitions[entry.defId];
      if (def == null) continue;
      final worldPos = worldOrigin + baseRot.rotate(entry.localPosition);
      final pieceYawDeg =
          yawSteps * blueprintStepDegrees + entry.yawSteps * entry.rotationStepDegrees;
      final pieceRot = BuildQuat.fromYawDegrees(pieceYawDeg);
      final res = solver.placeExact(
        defId: entry.defId,
        position: worldPos,
        rotation: pieceRot,
        yawSteps: entry.yawSteps + yawSteps,
        groundHeight: groundHeight,
      );
      if (res != null) {
        placed.add(res);
      }
    }
    return placed;
  }

  Map<String, Object?> toJson() => {
        'name': name,
        'entries': [for (final e in entries) e.toJson()],
      };

  factory ConstructionBlueprint.fromJson(Map<String, Object?> raw) => ConstructionBlueprint(
        name: raw['name'] as String,
        entries: [
          for (final item in (raw['entries'] as List<Object?>))
            BlueprintPieceEntry.fromJson(item as Map<String, Object?>),
        ],
      );
}
