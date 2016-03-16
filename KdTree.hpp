// SUGGESTED TAB SIZE: 4
#ifndef _KDTREE_H_
#define _KDTREE_H_

#ifndef CPPONLY
#endif

#include <vector>    // point datatype
#include <math.h>    // fabs operation
#include "MyHeaps.h" // priority queues
#include "float.h"   // max floating point number

using namespace std;

typedef vector<double> Point;

/// The root node is stored in position 0 of nodesPtrs
#define ROOT 0

namespace KdTreeBench {
    int distanceCalls = 0;
    int nodesVisited  = 0;
}

class Node{
public:
    double		key;		// the key (value along k-th dimension) of the split
    int			LIdx;		// the index to the left cell  (-1 if none)
    int			RIdx;		// the index to the right cell (-1 if none)

    /**
     * A poiter back to the structure data of input points,
     * but ONLY if the node is a LEAF, otherwise value is (-1)
     */
    int			pIdx;

    inline bool isLeaf() const{
        return pIdx>=0;
    }
    /// Default constructor
    Node(){
        LIdx = -1;
        RIdx = -1;
        key  = -1;
        pIdx  = -1;
    }
};

class KDTree {

    // Core data contained in the tree
private:
    vector<Point> points;    // Points data
private:
    vector<Node *> nodesPtrs; // Memory to keep nodes
private:
    int ndim;                // Data dimensionality
private:
    int npoints;             // Number of points
private:
    vector<int> workarray; // Used in tree construction

    /// Default destructor (delete the nodes)
public:
    ~KDTree() {
        for (unsigned int i = 0; i < nodesPtrs.size(); i++)
            delete nodesPtrs[i];
    }

    /// Heapsort algorithm used by fast KDtree construction
    // Note: this is copied almost verbatim from the heapsort
    // wiki page: http://en.wikipedia.org/wiki/Heapsort
    // 11/9/05
    // there was a bug for len==2 though!
    int heapsort(int dim, vector<int> &idx, int len) {
        unsigned int n = len;
        unsigned int i = len / 2;
        unsigned int parent, child;
        int t;

        for (; ;) {
            if (i > 0) {
                i--;
                t = idx[i];
            } else {
                n--;
                if (n == 0)
                    return 0;
                t = idx.at(n);
                idx[n] = idx[0];
            }

            parent = i;
            child = i * 2 + 1;

            while (child < n) {
                if ((child + 1 < n) && (points[idx[child + 1]][dim] > points[idx[child]][dim])) {
                    child++;
                }
                if (points[idx[child]][dim] > points[t][dim]) {
                    idx[parent] = idx[child];
                    parent = child;
                    child = parent * 2 + 1;
                }
                else {
                    break;
                }
            }
            idx[parent] = t;
        } // end of for loop
    } // end of heapsort


    /**
     * Creates a KDtree filled with the provided data.
     *
     * @param points   a vector< vector<double> > containing the point data
     * 				   the number of points and the dimensionality is inferred
     *                 by the data
     */
public:
    KDTree(const vector<Point> &points) {
        // initialize data
        this->npoints = points.size();
        this->ndim = points[0].size();
        this->points = points;
        nodesPtrs.reserve(npoints);
        workarray.resize(npoints, -1); //used in sorting based construction

        // create the heap structure to support the tree creation
        vector<MinHeap<double> > heaps(ndim, npoints);
        for (int dIdx = 0; dIdx < ndim; dIdx++)
            for (int pIdx = 0; pIdx < npoints; pIdx++)
                heaps[dIdx].push(points[pIdx][dIdx], pIdx);

        // Invoke heap sort generating indexing vectors
        // indexes[dim][i]: in dimension dim, which is the index of the i-th smallest element?
        vector<vector<int> > indexes(ndim, vector<int>(npoints, 0));
        for (int dIdx = 0; dIdx < ndim; dIdx++)
            heaps[dIdx].heapsort(indexes[dIdx]);

        // Invert the indexing structure!!
        // srtidx[dim][j]: in dimension dim, which is ordering number of point j-th?
        // the j-th smallest entry in dim dimension
        vector<vector<int> > srtidx(ndim, vector<int>(npoints, 0));
        for (unsigned int dim = 0; dim < indexes.size(); dim++)
            for (unsigned int i = 0; i < indexes[dim].size(); i++)
                srtidx[dim][indexes[dim][i]] = i;

        // First partition is on every single point ([1:npoints])
        vector<int> pidx(npoints, 0);
        for (int i = 0; i < npoints; i++) pidx[i] = i;
        build_recursively(srtidx, pidx, 0);
    }

public:
    KDTree() { }; // Default constructor - Added by Pablo Sala, Nov 25, 2008

    /// @return the number of points in the kd-tree
public:
    inline int size() { return points.size(); }

    /// @return the number of points in the kd-tree
public:
    inline int ndims() { return ndim; }

    /**
     * Algorithm that recursively performs median splits along dimension "dim"
     * using the pre-prepared information given by the sorting.
     *
     * @param sortidx: the back indexes produced by sorting along every dimension used
     *                 for linear time median computation
     * @param pidx:    a vector of indexes to active elements
     *
     * @param dim:     the current split dimension
     *
     * @note this is the memory-friendly version
     */
private:
    int build_recursively(vector<vector<int> > &sortidx, vector<int> &pidx, int dim) {

        // Stop condition
        if (pidx.size() == 1) {
            Node *node = new Node();        // create a new node
            int nodeIdx = nodesPtrs.size(); // its address is
            nodesPtrs.push_back(node);    // important to push back here
            node->LIdx = -1;                // no child
            node->RIdx = -1;                // no child
            node->pIdx = pidx[0];            // the only index available
            node->key = 0;                    // key is useless here
            return nodeIdx;
        }

        // allocate the vectors
        vector<int> Larray;
        vector<int> Rarray;
        Larray.reserve(pidx.size() / 2 + (pidx.size() % 2 == 0 ? 0 : 1));
        Rarray.reserve(pidx.size() / 2);

        // initialize the "partition" array
        // Setting parray to -1 indicates we are not using the point
        for (int i = 0; i < npoints; i++)
            workarray[i] = -1;
        for (unsigned int i = 0; i < pidx.size(); i++)
            workarray[sortidx[dim][pidx[i]]] = pidx[i];

        int pivot = -1; //index of the median element
        if ((double) pidx.size() * log((double) pidx.size()) < npoints) {
            Larray.resize(pidx.size() / 2 + (pidx.size() % 2 == 0 ? 0 : 1), -1);
            Rarray.resize(pidx.size() / 2, -1);
            heapsort(dim, pidx, pidx.size());
            std::copy(pidx.begin(), pidx.begin() + Larray.size(), Larray.begin());
            std::copy(pidx.begin() + Larray.size(), pidx.end(), Rarray.begin());
            pivot = pidx[(pidx.size() - 1) / 2];
        }
        else {
            // The middle valid value of parray is the pivot,
            // the left go to a node on the left, the right
            // and the pivot go to a node on the right.
            unsigned int TH = (pidx.size() - 1) / 2; //defines median offset
            unsigned int cnt = 0; //number of points found
            for (int i = 0; i < npoints; i++) {
                // Is the current point not in the current selection? skip
                if (workarray[i] == -1)
                    continue;

                // len/2 is on the "right" of pivot.
                // Pivot is still put on the left side
                if (cnt == TH) {
                    pivot = workarray[i];
                    Larray.push_back(workarray[i]);
                } else if (cnt > TH)
                    Rarray.push_back(workarray[i]);
                else
                    Larray.push_back(workarray[i]);

                // Don't overwork, if we already read all the necessary just stop.
                cnt++;
                if (cnt > pidx.size())
                    break;
            }
        }

        // CREATE THE NODE
        Node *node = new Node();
        int nodeIdx = nodesPtrs.size(); //why it's not +1? should not happend after push back? -> no since size() is the index of last element+1!!
        nodesPtrs.push_back(node); //important to push back here
        node->pIdx = -1; //not a leaf
        node->key = points[pivot][dim];
        node->LIdx = build_recursively(sortidx, Larray, (dim + 1) % ndim);
        node->RIdx = build_recursively(sortidx, Rarray, (dim + 1) % ndim);
        return nodeIdx;
    }

    /**
     * @param a a point in ndim-dimension
     * @param b a point in ndim-dimension
     * @returns L2 distance (in dimension ndim) between two points
     */
    inline double distance_squared(const vector<double> &a, const vector<double> &b) {
        KdTreeBench::distanceCalls++;
        double d = 0;
        double N = a.size();
        for (int i = 0; i < N; i++)
            d += (a[i] - b[i]) * (a[i] - b[i]);
        return d;
    }

public:
    int getCalls() const {
        return KdTreeBench::distanceCalls;
    }

    int getNodesVisited() const {
        return KdTreeBench::nodesVisited;
    }

public:
    std::vector<Point> search(const Point target, double radius) {
        std::vector<int> idxsInRange;
        std::vector<double> distances;

        ball_query(target, radius, idxsInRange, distances);

        std::vector<Point> results;
        results.reserve(idxsInRange.size());

        for (auto index : idxsInRange) {
            results.push_back(this->points[index]);
        }

        return results;
    }
    /**
     * Query all points at distance less or than radius from point
     *
     * @param point the center of the ndim dimensional query ball
     * @param radius the radius of the ndim dimensional query ball
     * @param idxsInRange (return) a collection of indexes of points that fall within
     *        the given ball.
     * @param distances the distances from the query point to the points within the ball
     *
     * @note This is a fairly unefficient implementation for two reasons:
     *       1) the range query is not implemented in its most efficient way
     *       2) all the points in between the bbox and the ball are visited as well, then rejected
     */
public:
    void ball_query(const Point &point, const double radius, vector<int> &idxsInRange, vector<double> &distances) {
        // create pmin pmax that bound the sphere
        Point pmin(ndim, 0);
        Point pmax(ndim, 0);
        for (int dim = 0; dim < ndim; dim++) {
            pmin[dim] = point[dim] - radius;
            pmax[dim] = point[dim] + radius;
        }
        KdTreeBench::distanceCalls = 0;
        KdTreeBench::nodesVisited = 0;

        // start from root at zero-th dimension
        ball_bbox_query(ROOT, pmin, pmax, idxsInRange, distances, point, radius * radius, 0);
    }
    /** @see ball_query, range_query
     *
     * Returns all the points withing the ball bounding box and their distances
     *
     * @note this is similar to "range_query" i just replaced "lies_in_range" with "euclidean_distance"
     */
public:
    void ball_bbox_query(int nodeIdx, Point &pmin, Point &pmax, vector<int> &inrange_idxs, vector<double> &distances,
                         const Point &point, const double &radiusSquared, int dim = 0) {
        KdTreeBench::nodesVisited++;
        Node *node = nodesPtrs[nodeIdx];

        // if it's a leaf and it lies in R
        if (node->isLeaf()) {
            double distance = distance_squared(points[node->pIdx], point);
            if (distance <= radiusSquared) {
                inrange_idxs.push_back(node->pIdx);
                distances.push_back(distance);
                return;
            }
        }
        else {
            if (node->key >= pmin[dim] && node->LIdx != -1)
                ball_bbox_query(node->LIdx, pmin, pmax, inrange_idxs, distances, point, radiusSquared,
                                (dim + 1) % ndim);
            if (node->key <= pmax[dim] && node->RIdx != -1)
                ball_bbox_query(node->RIdx, pmin, pmax, inrange_idxs, distances, point, radiusSquared,
                                (dim + 1) % ndim);
        }
    }
};

#endif




