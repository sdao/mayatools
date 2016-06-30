import maya.api.OpenMaya as om

selectionList = om.MGlobal.getActiveSelectionList()
itSelection = om.MItSelectionList(selectionList, om.MFn.kDagNode)

dagPaths = []
if not itSelection.isDone():
    dp = itSelection.getDagPath()
    dagPaths.append(dp)

mesh = dagPaths[0].node()
meshFn = om.MFnMesh(dagPaths[0])

# Compute new face vertices
faceIter = om.MItMeshPolygon(mesh)
newFacePoints = [None] * faceIter.count()

while not faceIter.isDone():
    facePoints = faceIter.getPoints()
    sf = 1.0 / len(facePoints)
    x = 0
    y = 0
    z = 0
    w = 0
    for pt in facePoints:
        x += pt.x
        y += pt.y
        z += pt.z
        w += pt.w
    newFacePoints[faceIter.index()] = om.MPoint(sf * x, sf * y, sf * z, sf * w)
    faceIter.next(None)

# Compute new edge vertices
edgeIter = om.MItMeshEdge(mesh)
newEdgePoints = [None] * edgeIter.count()

while not edgeIter.isDone():
    edgeFaces = edgeIter.getConnectedFaces()
    if len(edgeFaces) == 1:
        fp1 = newFacePoints[edgeFaces[0]]
        ep1 = edgeIter.point(0)
        ep2 = edgeIter.point(1)
        x = fp1.x + ep1.x + ep2.x
        y = fp1.y + ep1.y + ep2.y
        z = fp1.z + ep1.z + ep2.z
        w = fp1.w + ep1.w + ep2.w
        newEdgePoints[edgeIter.index()] = om.MPoint(x / 3.0, y / 3.0, z / 3.0, w / 3.0)
    else:
        fp1 = newFacePoints[edgeFaces[0]]
        fp2 = newFacePoints[edgeFaces[1]]
        ep1 = edgeIter.point(0)
        ep2 = edgeIter.point(1)
        x = fp1.x + fp2.x + ep1.x + ep2.x
        y = fp1.y + fp2.y + ep1.y + ep2.y
        z = fp1.z + fp2.z + ep1.z + ep2.z
        w = fp1.w + fp2.w + ep1.w + ep2.w
        newEdgePoints[edgeIter.index()] = om.MPoint(0.25 * x, 0.25 * y, 0.25 * z, 0.25 * w)
    edgeIter.next()

# Compute updated original points
vtxIter = om.MItMeshVertex(mesh)
oldPoints = meshFn.getPoints()
updatedPoints = [None] * vtxIter.count()

while not vtxIter.isDone():
    vtx = vtxIter.position()
    vtxFaces = vtxIter.getConnectedFaces()
    vtxEdgeVtxes = vtxIter.getConnectedVertices()

    n = len(vtxFaces)
    sf = 1.0 / n
    fpx = 0
    fpy = 0
    fpz = 0
    fpw = 0
    for face in vtxFaces:
        nfp = newFacePoints[face]
        fpx += nfp.x
        fpy += nfp.y
        fpz += nfp.z
        fpw += nfp.w
    F = om.MPoint(sf * fpx, sf * fpy, sf * fpz, sf * fpw)

    se = 1.0 / len(vtxEdgeVtxes)
    empx = 0
    empy = 0
    empz = 0
    empw = 0
    for edgeVtx in vtxEdgeVtxes:
        ep = oldPoints[edgeVtx]
        empx += 0.5 * (vtx.x + ep.x)
        empy += 0.5 * (vtx.y + ep.y)
        empz += 0.5 * (vtx.z + ep.z)
        empw += 0.5 * (vtx.w + ep.w)
    R = om.MPoint(se * empx, se * empy, se * empz, se * empw)

    def barycenter(ff, rr, pp, nn):
        return (ff + 2.0 * rr + (nn - 3) * pp) / float(nn)

    ux = barycenter(F.x, R.x, vtx.x, n)
    uy = barycenter(F.y, R.y, vtx.y, n)
    uz = barycenter(F.z, R.z, vtx.z, n)
    uw = barycenter(F.w, R.w, vtx.w, n)
    updatedPoints[vtxIter.index()] = om.MPoint(ux, uy, uz, uw)
    vtxIter.next()

newPoints = updatedPoints + newFacePoints + newEdgePoints
newPointsOfsFace = len(updatedPoints)
newPointsOfsEdge = newPointsOfsFace + len(newFacePoints)

polyCounts = []
polyConnects = []

faceIter = om.MItMeshPolygon(mesh)

while not faceIter.isDone():
    # Every new face corresponds to a face-vertex
    # The new face is defined as (updated vertex)--(new edge pt a)--(new face point)--(new edge pt b)
    faceEdges = faceIter.getEdges() # e0 = (v0--v1), e1 = (v1--v2), ...
    faceVtexes = faceIter.getVertices() # en -- v0 -- e0, e0 -- v1 -- e1, ...
    numVtexes = len(faceVtexes)
    for i in xrange(numVtexes):
        vtx = faceVtexes[i]
        epa = newPointsOfsEdge + faceEdges[i]
        nfp = newPointsOfsFace + int(faceIter.index())
        epb = newPointsOfsEdge + faceEdges[(numVtexes + i - 1) % numVtexes]

        polyCounts.append(4)
        polyConnects += [vtx, epa, nfp, epb]
    faceIter.next(None)

newMesh = om.MFnMesh()
newMesh.create(newPoints, polyCounts, polyConnects)
