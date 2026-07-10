// three-mesh-bvh's published typings omit the `indirect` build option (runtime-supported since
// v0.6, see node_modules/three-mesh-bvh/src/core/MeshBVH.js DEFAULT_OPTIONS). We rely on it: an
// indirect build leaves the shared geometry index untouched instead of reordering it in place,
// which would scramble the triangle -> solidOfTri mapping of the visible mesh.
import "three-mesh-bvh";

declare module "three-mesh-bvh" {
  interface MeshBVHOptions {
    indirect?: boolean;
  }
}
