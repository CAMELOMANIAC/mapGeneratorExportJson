import * as log from 'loglevel';
import * as isect from 'isect';
import * as d3 from 'd3-quadtree';
import Vector from '../vector';

declare module 'isect' {
    export function bush(lines: Segment[]): DetectIntersections;
}

interface DetectIntersections {
    run: () => Intersection[];
}

interface Segment {
    from: Vector;
    to: Vector;
}

interface Intersection {
    point: Vector;
    segments: Segment[];
}

interface ExportBuilding {
    height: number;
    lotWorld: Vector[];
}

/**
 * Node located along any intersection or point along the simplified road polylines 
 */
export class Node {
    private static nextId = 0;
    public id = Node.nextId++;
    public segments = new Set<Segment>();
    public adj: Node[];

    constructor(public value: Vector, public neighbors=new Set<Node>()) {}

    addSegment(segment: Segment): void {
        this.segments.add(segment);
    }

    addNeighbor(node: Node): void {
        if (node !== this) {
            this.neighbors.add(node);
            node.neighbors.add(this);
        }
    }
}

export default class Graph {
    public nodes: Node[];
    public intersections: Vector[];
    public edgeTypes = new Map<string, string>();

    public getEdgeKey(n1: Node, n2: Node): string {
        return n1.id < n2.id ? `${n1.id}_${n2.id}` : `${n2.id}_${n1.id}`;
    }

    /**
     * Create a graph from a set of streamlines
     * Finds all intersections, and creates a list of Nodes
     */
    constructor(streamlines: Vector[][], dstep: number, deleteDangling=false, streamlineTypes?: string[]) {
        const intersections = isect.bush(this.streamlinesToSegment(streamlines)).run();
        const quadtree = (d3.quadtree() as d3.Quadtree<Node>).x(n => n.value.x).y(n => n.value.y);
        const nodeAddRadius = 0.001;

        // Add all segment start and endpoints
        for (const streamline of streamlines) {
            for (let i = 0; i < streamline.length; i++) {
                const node = new Node(streamline[i]);
                if (i > 0) {
                    node.addSegment(this.vectorsToSegment(streamline[i - 1], streamline[i]));
                }

                if (i < streamline.length - 1) {
                    node.addSegment(this.vectorsToSegment(streamline[i], streamline[i + 1]));
                }

                this.fuzzyAddToQuadtree(quadtree, node, nodeAddRadius);
            }
        }

        // Add all intersections
        for (const intersection of intersections) {
            const node = new Node(new Vector(intersection.point.x, intersection.point.y));
            for (const s of intersection.segments) node.addSegment(s);
            this.fuzzyAddToQuadtree(quadtree, node, nodeAddRadius);
        }

        // For each simplified streamline, build list of nodes in order along streamline
        for (let sIdx = 0; sIdx < streamlines.length; sIdx++) {
            const streamline = streamlines[sIdx];
            const type = streamlineTypes ? streamlineTypes[sIdx] : 'minor';
            for (let i = 0; i < streamline.length - 1; i++) {
                const nodesAlongSegment =
                    this.getNodesAlongSegment(this.vectorsToSegment(streamline[i], streamline[i + 1]), quadtree, nodeAddRadius, dstep);
                
                if (nodesAlongSegment.length > 1) {
                    for (let j = 0; j < nodesAlongSegment.length - 1; j++) {
                        const n1 = nodesAlongSegment[j];
                        const n2 = nodesAlongSegment[j+1];
                        n1.addNeighbor(n2);
                        this.edgeTypes.set(this.getEdgeKey(n1, n2), type);
                    }
                } else {
                    log.error("Error Graph.js: segment with less than 2 nodes");
                }
            }
        }

        
        for (const n of quadtree.data()) {
            if (deleteDangling) {
                this.deleteDanglingNodes(n, quadtree);
            }
                n.adj = Array.from(n.neighbors);
        }

        this.nodes = quadtree.data();
        this.intersections = [];
        for (const i of intersections) this.intersections.push(new Vector(i.point.x, i.point.y));
    }

    /**
     * Serializes the graph, including road types and optional building POIs.
     */
    public toJSON(tolerance = 2.0, buildings?: ExportBuilding[]): any {
        if (tolerance > 0) {
            this.simplify(tolerance);
        }

        // 1. 노드별로 고유 ID를 매핑합니다.
        const nodeToIndex = new Map<Node, number>();
        this.nodes.forEach((node, index) => nodeToIndex.set(node, index));

        // 2. 건물 데이터 매핑
        const buildingsData = [];
        if (buildings) {
            for (let i = 0; i < buildings.length; i++) {
                const b = buildings[i];
                let cx = 0;
                let cy = 0;
                for (const v of b.lotWorld) {
                    cx += v.x;
                    cy += v.y;
                }
                cx /= b.lotWorld.length;
                cy /= b.lotWorld.length;

                let closestNodeIdx = 0;
                let minSqDist = Infinity;
                for (let j = 0; j < this.nodes.length; j++) {
                    const nodeVal = this.nodes[j].value;
                    const dx = nodeVal.x - cx;
                    const dy = nodeVal.y - cy;
                    const sqDist = dx * dx + dy * dy;
                    if (sqDist < minSqDist) {
                        minSqDist = sqDist;
                        closestNodeIdx = j;
                    }
                }

                buildingsData.push({
                    id: i,
                    height: b.height,
                    coordinates: b.lotWorld.map(v => ({ x: v.x, y: v.y })),
                    roadNodeId: closestNodeIdx
                });
            }
        }

        // 3. 그래프 데이터를 구조화합니다.
        const graphData = {
            nodes: this.nodes.map(node => ({
                x: node.value.x,
                y: node.value.y
            })),
            edges: [] as { source: number; target: number; type: string }[],
            buildings: buildingsData
        };

        // 4. 인접 노드 정보를 바탕으로 엣지 리스트를 만듭니다.
        for (let i = 0; i < this.nodes.length; i++) {
            const node = this.nodes[i];
            for (const neighbor of node.adj) {
                const neighborIndex = nodeToIndex.get(neighbor);
                if (neighborIndex !== undefined && i < neighborIndex) { // 중복 방지
                    const key = this.getEdgeKey(node, neighbor);
                    const type = this.edgeTypes.get(key) || 'minor';
                    graphData.edges.push({
                        source: i,
                        target: neighborIndex,
                        type: type
                    });
                }
            }
        }

        return graphData;
    }

    /**
     * Simplifies the graph topology by applying the Douglas-Peucker algorithm on curves.
     */
    public simplify(tolerance: number): void {
        if (tolerance <= 0) return;

        const visitedEdges = new Set<string>();
        const paths: Node[][] = [];

        // 1. First pass: extract paths starting from key nodes (degree !== 2)
        const keyNodes = this.nodes.filter(n => n.neighbors.size !== 2);
        for (const u of keyNodes) {
            for (const v of u.neighbors) {
                const key = this.getEdgeKey(u, v);
                if (visitedEdges.has(key)) continue;

                const path: Node[] = [u, v];
                visitedEdges.add(key);

                let current = v;
                let prev = u;
                while (current.neighbors.size === 2) {
                    let next: Node | null = null;
                    for (const nbr of current.neighbors) {
                        if (nbr !== prev) {
                            next = nbr;
                            break;
                        }
                    }
                    if (!next) break;

                    const nextKey = this.getEdgeKey(current, next);
                    if (visitedEdges.has(nextKey)) break;

                    path.push(next);
                    visitedEdges.add(nextKey);

                    prev = current;
                    current = next;

                    if (current.neighbors.size !== 2 || current === u) {
                        break;
                    }
                }
                paths.push(path);
            }
        }

        // 2. Second pass: extract isolated loops of degree 2 nodes
        for (const u of this.nodes) {
            if (u.neighbors.size === 2) {
                for (const v of u.neighbors) {
                    const key = this.getEdgeKey(u, v);
                    if (visitedEdges.has(key)) continue;

                    const path: Node[] = [u, v];
                    visitedEdges.add(key);

                    let current = v;
                    let prev = u;
                    while (true) {
                        let next: Node | null = null;
                        for (const nbr of current.neighbors) {
                            if (nbr !== prev) {
                                next = nbr;
                                break;
                            }
                        }
                        if (!next) break;

                        const nextKey = this.getEdgeKey(current, next);
                        if (visitedEdges.has(nextKey)) break;

                        path.push(next);
                        visitedEdges.add(nextKey);

                        prev = current;
                        current = next;

                        if (current === u) {
                            break;
                        }
                    }
                    paths.push(path);
                }
            }
        }

        // 3. For each extracted path, remove all internal edges,
        // simplify the path with Douglas-Peucker,
        // and add the simplified edges back.
        const nodesToRemove = new Set<Node>();

        for (const path of paths) {
            // Get original path type
            const pathType = this.edgeTypes.get(this.getEdgeKey(path[0], path[1])) || 'minor';

            // Remove old edges along the path
            for (let i = 0; i < path.length - 1; i++) {
                const n1 = path[i];
                const n2 = path[i + 1];
                n1.neighbors.delete(n2);
                n2.neighbors.delete(n1);
                this.edgeTypes.delete(this.getEdgeKey(n1, n2));
            }

            // Simplify the path using Douglas-Peucker
            const simplifiedPath = this.simplifyPath(path, tolerance);

            // Add new edges for the simplified path
            for (let i = 0; i < simplifiedPath.length - 1; i++) {
                const n1 = simplifiedPath[i];
                const n2 = simplifiedPath[i + 1];
                if (n1 !== n2) {
                    n1.neighbors.add(n2);
                    n2.neighbors.add(n1);
                    this.edgeTypes.set(this.getEdgeKey(n1, n2), pathType);
                }
            }

            // Collect nodes to remove (nodes that are in path but not in simplifiedPath)
            const keptNodesSet = new Set(simplifiedPath);
            for (const node of path) {
                if (!keptNodesSet.has(node)) {
                    nodesToRemove.add(node);
                }
            }
        }

        // 4. Update this.nodes and n.adj for all remaining nodes
        this.nodes = this.nodes.filter(node => !nodesToRemove.has(node));
        for (const node of this.nodes) {
            node.adj = Array.from(node.neighbors);
        }
    }

    private getSqSegDist(p: Vector, p1: Vector, p2: Vector): number {
        const dx = p2.x - p1.x;
        const dy = p2.y - p1.y;

        if (dx !== 0 || dy !== 0) {
            const t = ((p.x - p1.x) * dx + (p.y - p1.y) * dy) / (dx * dx + dy * dy);

            if (t > 1) {
                return p.distanceToSquared(p2);
            } else if (t > 0) {
                const proj = new Vector(p1.x + t * dx, p1.y + t * dy);
                return p.distanceToSquared(proj);
            }
        }

        return p.distanceToSquared(p1);
    }

    private simplifyDPStep(nodes: Node[], first: number, last: number, sqTolerance: number, kept: boolean[]): void {
        let maxSqDist = 0;
        let index = -1;

        const pFirst = nodes[first].value;
        const pLast = nodes[last].value;

        for (let i = first + 1; i < last; i++) {
            const sqDist = this.getSqSegDist(nodes[i].value, pFirst, pLast);
            if (sqDist > maxSqDist) {
                index = i;
                maxSqDist = sqDist;
            }
        }

        if (maxSqDist > sqTolerance) {
            kept[index] = true;
            this.simplifyDPStep(nodes, first, index, sqTolerance, kept);
            this.simplifyDPStep(nodes, index, last, sqTolerance, kept);
        }
    }

    private simplifyPath(nodes: Node[], tolerance: number): Node[] {
        if (nodes.length <= 2) return nodes;

        const sqTolerance = tolerance * tolerance;
        const kept = new Array(nodes.length).fill(false);
        kept[0] = true;
        kept[nodes.length - 1] = true;

        this.simplifyDPStep(nodes, 0, nodes.length - 1, sqTolerance, kept);

        const result: Node[] = [];
        for (let i = 0; i < nodes.length; i++) {
            if (kept[i]) {
                result.push(nodes[i]);
            }
        }
        return result;
    }

    /**
     * Remove dangling edges from graph to facilitate polygon finding
     */
    private deleteDanglingNodes(n: Node, quadtree: d3.Quadtree<Node>) {
        if (n.neighbors.size === 1) {
            quadtree.remove(n);
            for (let neighbor of n.neighbors) {
                neighbor.neighbors.delete(n);
                this.deleteDanglingNodes(neighbor, quadtree);
            }
        }
    }

    /**
     * Given a segment, step along segment and find all nodes along it
     */
    private getNodesAlongSegment(segment: Segment, quadtree: d3.Quadtree<Node>, radius: number, step: number): Node[] {
        // Walk dstep along each streamline, adding nodes within dstep/2
        // and connected to this streamline (fuzzy - nodeAddRadius) to list, removing from
        // quadtree and adding them all back at the end

        const foundNodes = [];
        const nodesAlongSegment: Node[] = [];

        const start = new Vector(segment.from.x, segment.from.y);
        const end = new Vector(segment.to.x, segment.to.y);
        
        const differenceVector = end.clone().sub(start);
        step = Math.min(step, differenceVector.length() / 2);  // Min of 2 step along vector
        const steps = Math.ceil(differenceVector.length() / step);
        const differenceVectorLength = differenceVector.length();

        for (let i = 0; i <= steps; i++) {
            let currentPoint = start.clone().add(differenceVector.clone().multiplyScalar(i / steps));

            // Order nodes, not by 'closeness', but by dot product
            let nodesToAdd = [];
            let closestNode = quadtree.find(currentPoint.x, currentPoint.y, radius + step/2);

            while (closestNode !== undefined) {
                quadtree.remove(closestNode);
                foundNodes.push(closestNode);
                
                let nodeOnSegment = false;
                for (let s of closestNode.segments) {
                    if (this.fuzzySegmentsEqual(s, segment)) {
                        nodeOnSegment = true;
                        break;
                    }
                }

                if (nodeOnSegment) {
                    nodesToAdd.push(closestNode);
                }

                closestNode = quadtree.find(currentPoint.x, currentPoint.y, radius + step/2);
            }

            nodesToAdd.sort((first: Node, second: Node) =>
                this.dotProductToSegment(first, start, differenceVector) - this.dotProductToSegment(second, start, differenceVector));
            nodesAlongSegment.push(...nodesToAdd);
        }

        quadtree.addAll(foundNodes);
        return nodesAlongSegment;
    }

    private fuzzySegmentsEqual(s1: Segment, s2: Segment, tolerance=0.0001): boolean {
        // From
        if (s1.from.x - s2.from.x > tolerance) {
            return false;
        }

        if (s1.from.y - s2.from.y > tolerance) {
            return false;
        }

        // To

        if (s1.to.x - s2.to.x > tolerance) {
            return false;
        }

        if (s1.to.y - s2.to.y > tolerance) {
            return false;
        }

        return true;
    }

    private dotProductToSegment(node: Node, start: Vector, differenceVector: Vector): number {
        const dotVector = node.value.clone().sub(start);
        return differenceVector.dot(dotVector);
    }

    private fuzzyAddToQuadtree(quadtree: d3.Quadtree<Node>, node: Node, radius: number): void {
        // Only add if there isn't a node within radius
        // Remember to check for double radius when querying tree, or point might be missed
        const existingNode = quadtree.find(node.value.x, node.value.y, radius);
        if (existingNode === undefined) {
            quadtree.add(node);
        } else {
            for (const neighbor of node.neighbors) existingNode.addNeighbor(neighbor);
            for (const segment of node.segments) existingNode.addSegment(segment);
        }
    }

    private streamlinesToSegment(streamlines: Vector[][]): Segment[] {
        const out: Segment[] = [];
        for (const s of streamlines) {
            for (let i = 0; i < s.length - 1; i++) {
                out.push(this.vectorsToSegment(s[i], s[i + 1]));
            }
        }

        return out;
    }

    private vectorsToSegment(v1: Vector, v2: Vector): Segment {
        return {
            from: v1,
            to:   v2
        };
    }
}
