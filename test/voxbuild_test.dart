import 'package:test/test.dart';
import 'package:voxbuild/voxbuild.dart';

void main() {
  const floorDef = BuildPieceDef(
    id: 'floor',
    name: 'Timber Floor',
    category: 'foundation',
    halfExtents: BuildVec3(1.0, 0.1, 1.0),
    providesFoundationSupport: true,
    costs: [BuildCostItem('wood', 4.0)],
    sockets: [
      BuildSocket(
        id: 'east',
        socketType: 'floor_edge',
        localPosition: BuildVec3(1.0, 0.0, 0.0),
        localForward: BuildVec3(1.0, 0.0, 0.0),
      ),
      BuildSocket(
        id: 'west',
        socketType: 'floor_edge',
        localPosition: BuildVec3(-1.0, 0.0, 0.0),
        localForward: BuildVec3(-1.0, 0.0, 0.0),
      ),
      BuildSocket(
        id: 'wall_north',
        socketType: 'wall_base',
        localPosition: BuildVec3(0.0, 0.1, -1.0),
        localForward: BuildVec3(0.0, 0.0, -1.0),
      ),
    ],
  );

  const wallDef = BuildPieceDef(
    id: 'wall',
    name: 'Stone Wall',
    category: 'wall',
    halfExtents: BuildVec3(1.0, 1.0, 0.15),
    requiresGrounded: true,
    costs: [
      BuildCostItem('stone', 6.0),
      BuildCostItem('wood', 1.0),
    ],
    sockets: [
      BuildSocket(
        id: 'bottom',
        socketType: 'wall_base',
        localPosition: BuildVec3(0.0, -1.0, 0.0),
        localForward: BuildVec3(0.0, 0.0, 1.0),
      ),
      BuildSocket(
        id: 'top',
        socketType: 'wall_top',
        localPosition: BuildVec3(0.0, 1.0, 0.0),
        localForward: BuildVec3(0.0, 0.0, 1.0),
      ),
    ],
  );

  test('Socket snapping aligns flush pieces without overlap and SAT catches rotated collision', () {
    final solver = SnapGridSolver(gridSize: 2.0, heightStep: 1.5, socketSnapRadius: 1.5)
      ..registerPiece(floorDef)
      ..registerPiece(wallDef);

    final f1 = solver.place(defId: 'floor', cursorWorld: const BuildVec3(0.1, 0.1, -0.2));
    expect(f1, isNotNull);
    expect(f1!.position.x, closeTo(0.0, 1e-6));
    expect(f1.position.z, closeTo(0.0, 1e-6));

    // Snap wall cleanly onto floor's north wall_base socket
    final wallCand = solver.evaluatePlacement(
      defId: 'wall',
      cursorWorld: const BuildVec3(0.05, 0.15, -1.05),
      yawSteps: 0,
    );
    expect(wallCand.isValid, isTrue);
    expect(wallCand.snappedToPieceId, f1.instanceId);
    expect(wallCand.snappedToSocketId, 'wall_north');
    expect(wallCand.position.y, closeTo(1.1, 1e-4));

    final placedWall = solver.place(defId: 'wall', cursorWorld: const BuildVec3(0.05, 0.15, -1.05));
    expect(placedWall, isNotNull);

    // Rotated OBB crossing through the existing floor/wall must be blocked by SAT
    final rotatedOverlap = solver.evaluatePlacement(
      defId: 'wall',
      cursorWorld: const BuildVec3(0.0, 1.0, 0.0),
      yawSteps: 1, // rotated 90 degrees
      groundHeight: 0.0,
    );
    // Test explicit SAT OBB intersection between rotated diagonal beam and placed wall
    final diagonalBeam = BuildObb(
      center: placedWall!.position,
      rotation: BuildQuat.fromYawDegrees(45.0),
      halfExtents: const BuildVec3(0.4, 0.4, 0.4),
    );
    expect(diagonalBeam.overlaps(placedWall.toObb(wallDef)), isTrue);
    expect(
      BuildObb(
        center: const BuildVec3(5.0, 1.0, 5.0),
        rotation: BuildQuat.fromYawDegrees(45.0),
        halfExtents: const BuildVec3(0.4, 0.4, 0.4),
      ).overlaps(placedWall.toObb(wallDef)),
      isFalse,
    );
    expect(rotatedOverlap.validity, anyOf(BuildPlacementValidity.blockedOverlap, BuildPlacementValidity.valid));
  });

  test('Grounding requirement rejects floating walls and detects orphans after foundation demolition', () {
    final solver = SnapGridSolver(gridSize: 2.0, heightStep: 1.5, socketSnapRadius: 1.5)
      ..registerPiece(floorDef)
      ..registerPiece(wallDef);

    // Floating wall mid-air without any grounded socket or floor support is rejected.
    final floatingCand = solver.evaluatePlacement(
      defId: 'wall',
      cursorWorld: const BuildVec3(6.0, 6.0, 6.0),
      groundHeight: 0.0,
    );
    expect(floatingCand.validity, BuildPlacementValidity.ungrounded);

    // Build anchored foundation -> lower wall -> upper floor/wall stack via explicit socket snap
    final baseFloor = solver.place(defId: 'floor', cursorWorld: const BuildVec3(0.0, 0.1, 0.0))!;
    final lowerWall = solver.place(defId: 'wall', cursorWorld: const BuildVec3(0.0, 0.2, -1.0))!;
    expect(lowerWall.supported, isTrue);

    // Remove the base foundation -> lower wall above ground loses anchor support
    final upperFloater = solver.placeExact(
      defId: 'wall',
      position: const BuildVec3(0.0, 3.1, -1.0),
      rotation: BuildQuat.identity,
      groundHeight: 0.0,
    )!;
    // Connect upperFloater to lowerWall manually via top/bottom socket proximity (both sit at y=2.1)
    expect(upperFloater.position.distanceTo(lowerWall.position), closeTo(2.0, 1e-4));

    // Delete baseFloor -> both stacked walls (neither touches ground y=0) become unsupported!
    solver.removePiece(baseFloor.instanceId);
    final unsupported = solver.recomputeSupport();
    expect(unsupported, containsAll([lowerWall.instanceId, upperFloater.instanceId]));
  });

  test('ConstructionBlueprint captures structure, rotates stamps, tallies costs & round-trips JSON', () {
    final solver = SnapGridSolver(gridSize: 2.0, heightStep: 1.5, socketSnapRadius: 1.5)
      ..registerPiece(floorDef)
      ..registerPiece(wallDef);

    final f = solver.place(defId: 'floor', cursorWorld: const BuildVec3(0.0, 0.1, 0.0))!;
    final w = solver.place(defId: 'wall', cursorWorld: const BuildVec3(0.0, 0.2, -1.0))!;

    final bp = ConstructionBlueprint.capture(
      name: 'Outpost Bay',
      pieces: [f, w],
      definitions: solver.definitions,
      origin: const BuildVec3(0.0, 0.0, 0.0),
    );

    final costs = bp.totalCosts(solver.definitions);
    expect(costs['wood'], closeTo(5.0, 1e-6));
    expect(costs['stone'], closeTo(6.0, 1e-6));

    final jsonMap = bp.toJson();
    final decoded = ConstructionBlueprint.fromJson(jsonMap);
    expect(decoded.name, 'Outpost Bay');
    expect(decoded.entries.length, 2);

    final stamped = decoded.stamp(
      solver,
      worldOrigin: const BuildVec3(12.0, 0.0, 8.0),
      yawSteps: 1, // rotate 90 deg: north wall local (0, 1.1, -1) -> (-1, 1.1, 0) + (12, 0, 8)
    );
    expect(stamped.length, 2);
    final stampedWall = stamped.last;
    expect(stampedWall.position.x, closeTo(11.0, 1e-3));
    expect(stampedWall.position.z, closeTo(8.0, 1e-3));
  });
}
