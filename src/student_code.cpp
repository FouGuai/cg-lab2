#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL {

template <typename vector_type>
vector_type lerp(const vector_type &lhs, const vector_type &rhs, float t) {
  return t * lhs + (1 - t) * rhs;
}

/**
 * Evaluates one step of the de Casteljau's algorithm using the given points and
 * the scalar parameter t (class member).
 *
 * @param points A vector of points in 2D
 * @return A vector containing intermediate points or the final interpolated
 * vector
 */
std::vector<Vector2D>
BezierCurve::evaluateStep(std::vector<Vector2D> const &points) {
  // TODO Part 1.
  std::vector<Vector2D> currentPoints = points;

  // Repeat the de Casteljau algorithm until we have a single point.
  while (currentPoints.size() > 1) {
    std::vector<Vector2D> nextPoints;

    // Interpolate between adjacent points
    for (size_t i = 0; i < currentPoints.size() - 1; ++i) {
      nextPoints.push_back(lerp(currentPoints[i], currentPoints[i + 1], t));
    }

    // Update currentPoints for the next iteration
    currentPoints = std::move(nextPoints);
  }

  // Return the final point (or intermediate points, depending on how many steps
  // you want)
  return currentPoints;
}

/**
 * Evaluates one step of the de Casteljau's algorithm using the given
 * points and the scalar parameter t (function parameter).
 *
 * @param points    A vector of points in 3D
 * @param t         Scalar interpolation parameter
 * @return A vector containing intermediate points or the final
 * interpolated vector
 */
std::vector<Vector3D>
BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const {
  // TODO Part 2.
  std::vector<Vector3D> currentPoints = points;

  // Repeat the de Casteljau algorithm until we have a single point
  while (currentPoints.size() > 1) {
    std::vector<Vector3D> nextPoints;

    // Interpolate between adjacent points along the current direction
    for (size_t i = 0; i < currentPoints.size() - 1; ++i) {
      nextPoints.push_back(lerp(currentPoints[i], currentPoints[i + 1], t));
    }

    // Update currentPoints for the next iteration
    currentPoints = nextPoints;
  }

  return currentPoints; // Final point after interpolation
}

/**
 * Fully evaluates de Casteljau's algorithm for a vector of points at scalar
 * parameter t
 *
 * @param points    A vector of points in 3D
 * @param t         Scalar interpolation parameter
 * @return Final interpolated vector
 */
Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points,
                                 double t) const {
  // TODO Part 2.
  std::vector<Vector3D> currentPoints = points;

  // Repeatedly apply evaluateStep to reduce to a single point
  while (currentPoints.size() > 1) {
    currentPoints = evaluateStep(currentPoints, t);
  }

  // Return the single point on the Bézier curve
  return currentPoints.front();
}

/**
 * Evaluates the Bezier patch at parameter (u, v)
 *
 * @param u         Scalar interpolation parameter
 * @param v         Scalar interpolation parameter (along the other axis)
 * @return Final interpolated vector
 */
Vector3D BezierPatch::evaluate(double u, double v) const {
  // TODO Part 2.
  // First, evaluate the Bézier curves along the u direction for each fixed v
  std::vector<Vector3D> intermediatePoints;

  for (const auto &row : controlPoints) { // Each row corresponds to a fixed v
    intermediatePoints.push_back(evaluate1D(row, u)); // Interpolate along u
  }

  // Now, interpolate between the intermediate points along the v direction
  return evaluate1D(intermediatePoints, v); // Interpolate along v
}

Vector3D Vertex::normal(void) const {
  // TODO Part 3.
  // Returns an approximate unit normal at this vertex, computed by
  // taking the area-weighted average of the normals of neighboring
  // triangles, then normalizing.

  Vector3D normal(0, 0, 0); // Initialize the normal to zero
  float totalArea =
      0.0f; // Variable to accumulate the total area for normalization

  // Iterate over all halfedges incident on this vertex
  HalfedgeIter h = _halfedge;
  do {
    // Get the face this halfedge is part of
    FaceIter face = h->face();

    // Get the three vertices of the face
    Vector3D v1 = face->halfedge()->vertex()->position;
    Vector3D v2 = face->halfedge()->next()->vertex()->position;
    Vector3D v3 = face->halfedge()->next()->next()->vertex()->position;

    Vector3D edge1 = v2 - v1;
    Vector3D edge2 = v3 - v1;

    Vector3D crossProduct = cross(edge1, edge2);
    float area = crossProduct.norm() * 0.5f; // Area of the triangle

    crossProduct.normalize();

    normal += crossProduct * area;
    totalArea += area;

    h = h->twin()->next();
  } while (
      h !=
      _halfedge); // Done when all halfedges around the vertex are processed

  // Normalize the final normal vector
  if (totalArea > 0.0f) {
    normal /= totalArea; // Normalize by dividing by the total area
    normal.normalize();  // Ensure the normal is a unit vector
  }

  return normal;
}

EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0) {
  // TODO Part 4.
  // This method should flip the given edge and return an iterator to the
  // flipped edge.
  HalfedgeIter he0 = e0->halfedge();
  HalfedgeIter he1 = e0->halfedge()->twin();

  FaceIter f0 = he0->face();
  FaceIter f1 = he1->face();

  if (e0->isBoundary()) {
    return EdgeIter();
  }

  HalfedgeIter nhe0 = he0->next();
  HalfedgeIter nhe1 = he1->next();

  HalfedgeIter nnhe0 = nhe0->next();
  HalfedgeIter nnhe1 = nhe1->next();

  VertexIter v0 = nnhe0->vertex();
  VertexIter v1 = nnhe1->vertex();

  nhe0->next() = he0;
  nhe1->next() = he1;

  he0->setNeighbors(nnhe1, he1, v0, e0, f0);
  he1->setNeighbors(nnhe0, he0, v1, e0, f1);

  nnhe1->next() = nhe0;
  nnhe1->face() = f0;

  nnhe0->next() = nhe1;
  nnhe0->face() = f1;

  f0->halfedge() = he0;
  f1->halfedge() = he1;
  v0->halfedge() = he0;
  v1->halfedge() = he1;

  e0->halfedge() = he0;
  return e0;
}

VertexIter HalfedgeMesh::splitEdge(EdgeIter e0) {
  // TODO Part 5.
  // This method should split the given edge and return an iterator to the
  // newly inserted vertex. The halfedge of this vertex should point along the
  // edge that was split, rather than the new edges.

  HalfedgeIter he0 = e0->halfedge();
  HalfedgeIter he1 = he0->twin();
  FaceIter f0 = he0->face();
  FaceIter f1 = he1->face();

  if (f0->isBoundary() || f1->isBoundary()) {
    return VertexIter();
  }

  HalfedgeIter nhe0 = he0->next();
  HalfedgeIter nhe1 = he1->next();
  HalfedgeIter nnhe0 = nhe0->next();
  HalfedgeIter nnhe1 = nhe1->next();

  VertexIter v0 = he0->vertex();
  VertexIter v1 = he1->vertex();
  VertexIter nv0 = nnhe0->vertex();
  VertexIter nv1 = nnhe1->vertex();

  VertexIter v = newVertex();
  v->position = 0.5 * (v0->position + v1->position);
  v->isNew = true;
  // middle line

  FaceIter nf0 = newFace();
  FaceIter nf1 = newFace();
  EdgeIter e1 = newEdge();
  HalfedgeIter mhe0 = newHalfedge();
  HalfedgeIter mhe1 = newHalfedge();

  he1->edge() = e1;
  e1->halfedge() = he1;
  // e1->isNew = true;

  mhe0->setNeighbors(nhe0, he1, v, e1, nf0);
  mhe1->setNeighbors(nhe1, he0, v, e0, nf1);
  nf0->halfedge() = mhe0;
  nf1->halfedge() = mhe1;
  e0->halfedge() = he0;

  // row line
  HalfedgeIter lfhe = newHalfedge();
  HalfedgeIter lhhe = newHalfedge();
  HalfedgeIter rfhe = newHalfedge();
  HalfedgeIter rhhe = newHalfedge();
  EdgeIter el = newEdge();
  EdgeIter er = newEdge();

  lfhe->setNeighbors(mhe0, lhhe, nv0, el, nf0);
  lhhe->setNeighbors(nnhe0, lfhe, v, el, f0);
  nhe0->next() = lfhe;
  he0->setNeighbors(lhhe, mhe1, v0, e0, f0);
  el->halfedge() = lfhe;
  el->isNew = true;
  nhe0->face() = nf0;

  rfhe->setNeighbors(mhe1, rhhe, nv1, er, nf1);
  rhhe->setNeighbors(nnhe1, rfhe, v, er, f1);
  nhe1->next() = rfhe;
  he1->setNeighbors(rhhe, mhe0, v1, e1, f1);
  er->halfedge() = rfhe;
  er->isNew = true;
  nhe1->face() = nf1;

  v->halfedge() = mhe0;

  f0->halfedge() = he0;
  f1->halfedge() = he1;

  return v;
}

void MeshResampler::upsample(HalfedgeMesh &mesh) {
  // TODO Part 6.
  // This routine should increase the number of triangles in the mesh using
  // Loop subdivision. One possible solution is to break up the method as
  // listed below.

  // 1. Compute new positions for all the vertices in the input mesh, using
  // the Loop subdivision rule, and store them in Vertex::newPosition. At this
  // point, we also want to mark each vertex as being a vertex of the original
  // mesh.

  // 2. Compute the updated vertex positions associated with edges, and store
  // it in Edge::newPosition.

  // 3. Split every edge in the mesh, in any order. For future reference,
  // we're also going to store some information about which subdivide edges
  // come from splitting an edge in the original mesh, and which edges are
  // new, by setting the flat Edge::isNew. Note that in this loop, we only
  // want to iterate over edges of the original mesh---otherwise, we'll end up
  // splitting edges that we just split (and the loop will never end!)

  // 4. Flip any new edge that connects an old and new vertex.

  // 5. Copy the new vertex positions into final Vertex::position.
  // Step 1: Split edges to perform 4-1 subdivision.
  // Iterate over all edges in the mesh to split them.

  for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
    e->isNew = 0;

    HalfedgeIter h0 = e->halfedge();
    HalfedgeIter h1 = h0->next();
    HalfedgeIter h2 = h1->next();
    HalfedgeIter h3 = h0->twin();
    HalfedgeIter h4 = h3->next();
    HalfedgeIter h5 = h4->next();

    VertexIter B = h0->vertex();
    VertexIter C = h3->vertex();
    VertexIter A = h2->vertex();
    VertexIter D = h5->vertex();

    e->newPosition = 3.0 / 8.0 * (B->position + C->position) +
                     1.0 / 8.0 * (A->position + D->position);
  }

  for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
    v->isNew = 0;

    HalfedgeIter h = v->halfedge();
    Vector3D adjacentSum(0, 0, 0);
    do {
      adjacentSum += h->twin()->vertex()->position;
      h = h->twin()->next();
    } while (h != v->halfedge());
    float n = (float)v->degree();
    float u = (n == 3.0) ? (3.0 / 16.0) : (3.0 / (8.0 * n));
    v->newPosition = (1.0 - n * u) * v->position + u * adjacentSum;
  }

  for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
    VertexIter B = e->halfedge()->vertex();
    VertexIter C = e->halfedge()->twin()->vertex();

    if (!B->isNew && !C->isNew) {
      VertexIter E = mesh.splitEdge(e);
      E->newPosition = e->newPosition;
    }
  }
  for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
    VertexIter v0 = e->halfedge()->vertex();
    VertexIter v1 = e->halfedge()->twin()->vertex();
    if (e->isNew && v0->isNew != v1->isNew) {
      mesh.flipEdge(e);
    }
  }

  for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
    v->position = v->newPosition;
    v->isNew = 0;
  }
}
} // namespace CGL
