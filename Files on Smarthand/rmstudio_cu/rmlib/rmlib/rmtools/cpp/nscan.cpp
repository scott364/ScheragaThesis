// python setup.py build_ext --inplace

#include <Python.h>
#include <math.h>
#include <vector>
#include <string>
#include <iostream>
#include <stdexcept>
#include <algorithm>

std::vector<float> _x;
std::vector<float> _y;
std::vector<float> _z;
std::vector<float> _nx;
std::vector<float> _ny;
std::vector<float> _nz;
std::vector<float> _c;

struct Point;

struct Edge {
    Point* point;
};

struct Point {

    std::vector<float> coords;

    float nx;
    float ny;
    float nz;

    float c;

    std::vector<Edge> edges;

    bool used;
};

std::vector<Point> points;


// ========================================================================== //
//                                 HELPERS                                    //
// ========================================================================== //


std::vector<float> listTupleToVector(PyObject* args) {
	std::vector<float> data;
	if (PyTuple_Check(args)) {
		for(Py_ssize_t i = 0; i < PyTuple_Size(args); i++) {
			PyObject *value = PyTuple_GetItem(args, i);
			data.push_back( PyFloat_AsDouble(value) );
		}
	} else {
		if (PyList_Check(args)) {
			for(Py_ssize_t i = 0; i < PyList_Size(args); i++) {
				PyObject *value = PyList_GetItem(args, i);
				data.push_back( PyFloat_AsDouble(value) );
			}
		} else {
			throw std::logic_error("Passed PyObject pointer was not a list or tuple!");
		}
	}
	return data;
}


PyObject* vectorToTuple(const std::vector<float> &data) {
	PyObject* tuple = PyTuple_New( data.size() );
	if (!tuple) throw std::logic_error("Unable to allocate memory for Python tuple");
	for (unsigned int i = 0; i < data.size(); i++) {
		PyObject *num = PyFloat_FromDouble( (double) data[i]);
		if (!num) {
			Py_DECREF(tuple);
			throw std::logic_error("Unable to allocate memory for Python tuple");
		}
		PyTuple_SET_ITEM(tuple, i, num);
	}

	return tuple;
}

PyObject* vectorVectorToTuple(const std::vector< std::vector< float > > &data) {
	PyObject* tuple = PyTuple_New( data.size() );
	if (!tuple) throw std::logic_error("Unable to allocate memory for Python tuple");
	for (unsigned int i = 0; i < data.size(); i++) {
		PyObject* subTuple = NULL;
		try {
			subTuple = vectorToTuple(data[i]);
		} catch (std::logic_error &e) {
			throw e;
		}
		if (!subTuple) {
			Py_DECREF(tuple);
			throw std::logic_error("Unable to allocate memory for Python tuple of tuples");
		}
		PyTuple_SET_ITEM(tuple, i, subTuple);
	}

	return tuple;
}



static PyObject* vectorVectorVectorToTuple(const std::vector< std::vector< std::vector<float> > > &data) {

    PyObject* tuple = PyTuple_New( data.size() );
	if (!tuple) throw std::logic_error("Unable to allocate memory for Python tuple");
	for (unsigned int i = 0; i < data.size(); i++) {
		PyObject* subTuple = NULL;
		try {
			subTuple = vectorVectorToTuple(data[i]);
		} catch (std::logic_error &e) {
			throw e;
		}
		if (!subTuple) {
			Py_DECREF(tuple);
			throw std::logic_error("Unable to allocate memory for Python tuple of tuples");
		}
		PyTuple_SET_ITEM(tuple, i, subTuple);
	}

	return tuple;
}

// ========================================================================== //
//                             USER FUNCTIONS                                 //
// ========================================================================== //


static PyObject* set_x(PyObject* self, PyObject* args) {
    _x.clear();
    _x = listTupleToVector(args);
    return Py_BuildValue("");
}

static PyObject* set_y(PyObject* self, PyObject* args) {
    _y.clear();
    _y = listTupleToVector(args);
    return Py_BuildValue("");
}

static PyObject* set_z(PyObject* self, PyObject* args) {
    _z.clear();
    _z = listTupleToVector(args);
    return Py_BuildValue("");
}

static PyObject* set_nx(PyObject* self, PyObject* args) {
    _nx.clear();
    _nx = listTupleToVector(args);
    return Py_BuildValue("");
}

static PyObject* set_ny(PyObject* self, PyObject* args) {
    _ny.clear();
    _ny = listTupleToVector(args);
    return Py_BuildValue("");
}

static PyObject* set_nz(PyObject* self, PyObject* args) {
    _nz.clear();
    _nz = listTupleToVector(args);
    return Py_BuildValue("");
}

static PyObject* set_c(PyObject* self, PyObject* args) {
    _c.clear();
    _c = listTupleToVector(args);
    return Py_BuildValue("");
}

static PyObject* initialize(PyObject* self, PyObject* args) {

    if (_x.size() != _y.size()) throw std::logic_error("Attribute size mismatch");
    if (_x.size() != _z.size()) throw std::logic_error("Attribute size mismatch");
    if (_x.size() != _nx.size()) throw std::logic_error("Attribute size mismatch");
    if (_x.size() != _ny.size()) throw std::logic_error("Attribute size mismatch");
    if (_x.size() != _nz.size()) throw std::logic_error("Attribute size mismatch");
    if (_x.size() != _c.size()) throw std::logic_error("Attribute size mismatch");

    std::vector<float> _coords;

    points.clear();
    for (unsigned int i=0; i<_x.size(); i++) {
        Point newPoint;

        _coords.clear();
        _coords.push_back(_x[i]);
        _coords.push_back(_y[i]);
        _coords.push_back(_z[i]);
        newPoint.coords = _coords;

        newPoint.nx = _nx[i];
        newPoint.ny = _ny[i];
        newPoint.nz = _nz[i];

        newPoint.c = _c[i];

        newPoint.used = false;

        points.push_back(newPoint);
    }
//     std::reverse(points.begin(), points.end());
    return Py_BuildValue("");
}

static PyObject* connect(PyObject* self, PyObject* args) {
    int index;
    unsigned int pointIndex = 0;
    PyObject *value;

    if (PyTuple_Check(args)) {

        value = PyTuple_GetItem(args, 0);
        int k = PyLong_AsLong(value);

		for(Py_ssize_t i = 1; i < PyTuple_Size(args); i++) {

            if (pointIndex >= points.size()) {
                throw std::logic_error("Tuple argument of wrong size");
                return Py_BuildValue("");
            }

            value = PyTuple_GetItem(args, i);
            index = PyLong_AsLong(value);

            Edge newEdge;
            newEdge.point = &points[index];
            points[pointIndex].edges.push_back(newEdge);

            if (i % k == 0) {
                pointIndex++;
            }

		}
	} else {
        throw std::logic_error("Argument not tuple");
    }

	return Py_BuildValue("");
}

float vector_angle(std::vector<float> vec1, std::vector<float> vec2) {

    float x1 = vec1[0];
    float y1 = vec1[1];
    float z1 = vec1[2];

    float x2 = vec2[0];
    float y2 = vec2[1];
    float z2 = vec2[2];

    float dot = x1*x2 + y1*y2 + z1*z2;
    float lenSq1 = sqrt(x1*x1 + y1*y1 + z1*z1);
    float lenSq2 = sqrt(x2*x2 + y2*y2 + z2*z2);
    float angle = acos(dot/(lenSq1 * lenSq2));

    return angle;
}

int get_min_curve_index() {
    float minCurve = 9999;
    int minCurveIndex = -1;
    for (unsigned int i=0; i<points.size(); i++) {
        if (!points[i].used) {
            if (points[i].c < minCurve) {
                minCurve = points[i].c;
                minCurveIndex = i;
            }
        }
    }
    return minCurveIndex;
}

static PyObject* NSCAN(PyObject* self, PyObject* args) {

    PyObject *value;

    value = PyTuple_GetItem(args, 0);
    unsigned int min_cluster_size = PyLong_AsLong(value);

    value = PyTuple_GetItem(args, 1);
    double curve_threshold = PyFloat_AsDouble(value);

    value = PyTuple_GetItem(args, 2);
    double angle_threshold = PyFloat_AsDouble(value);

    //------------------------------------------------------------//

    std::vector< std::vector<float> > region;
    std::vector< std::vector< std::vector<float> > > regionList;

    std::vector<Point*> seeds;
    Point* seed;
    Point* neighbor;

    int unusedPoints = points.size();
    int minCurveIndex;

    while (unusedPoints > 0) {
        region.clear();
        seeds.clear();

        minCurveIndex = get_min_curve_index();
        if (minCurveIndex == -1) {
            break;
        }
        if (points[minCurveIndex].c > curve_threshold) {
            break;
        }

        seeds.push_back(&points[minCurveIndex]);
        region.push_back(points[minCurveIndex].coords);
        points[minCurveIndex].used = true;
        unusedPoints--;

        while (seeds.size() > 0) {
            seed = seeds.front();
            seeds.erase(seeds.begin());

            for (unsigned int i=0; i<seed->edges.size(); i++) {
                neighbor = seed->edges[i].point;

                if (!neighbor->used) {
                    if (vector_angle(seed->coords,neighbor->coords) < angle_threshold) {
                        region.push_back(neighbor->coords);
                        neighbor->used = true;
                        unusedPoints--;

                        if (neighbor->c < curve_threshold) {
                            seeds.push_back(neighbor);
                        }
                    }
                }
            }
        }
        if (region.size() > min_cluster_size) {
            regionList.push_back(region);
        }
    }

    //was clear();
    seeds.clear();

    if (regionList.size() > 0){
        return vectorVectorVectorToTuple(regionList);
    }
    else {
        std::vector<float> emptyVec;
        return vectorToTuple(emptyVec);
    }
}

static PyObject* NSCAN_spread_exterior(PyObject* self, PyObject* args) {

    PyObject *value;

    value = PyTuple_GetItem(args, 0);
    unsigned int min_cluster_size = PyLong_AsLong(value);

    value = PyTuple_GetItem(args, 1);
    double curve_threshold = PyFloat_AsDouble(value);

    value = PyTuple_GetItem(args, 2);
    double angle_threshold = PyFloat_AsDouble(value);

    //------------------------------------------------------------//

    std::vector< std::vector<float> > region;
    std::vector< std::vector< std::vector<float> > > regionList;

    std::vector<Point*> seeds;
    Point* seed;
    Point* neighbor;

    int unusedPoints = points.size();
    int minCurveIndex;
    float angle;

    while (unusedPoints > 0) {
        region.clear();
        seeds.clear();

        minCurveIndex = get_min_curve_index();
        if (minCurveIndex == -1) {
            break;
        }
        if (points[minCurveIndex].c > curve_threshold) {
            break;
        }

        seeds.push_back(&points[minCurveIndex]);
        region.push_back(points[minCurveIndex].coords);
        points[minCurveIndex].used = true;
        unusedPoints--;

        while (seeds.size() > 0) {
            seed = seeds.front();
            seeds.erase(seeds.begin());

            for (unsigned int i=0; i<seed->edges.size(); i++) {
                neighbor = seed->edges[i].point;

                if (!neighbor->used) {
                    angle = vector_angle(seed->coords,neighbor->coords);
                    if (angle < angle_threshold) {
                        region.push_back(neighbor->coords);
                        neighbor->used = true;
                        unusedPoints--;

                        if (neighbor->c < curve_threshold) {
                            seeds.push_back(neighbor);
                        }
                    }
                }
            }
        }
        if (region.size() > min_cluster_size) {
            regionList.push_back(region);
        }
    }


    if (regionList.size() > 0){
        return vectorVectorVectorToTuple(regionList);
    }
    else {
        std::vector<float> emptyVec;
        return vectorToTuple(emptyVec);
    }
}

static PyObject* foo(PyObject* self, PyObject* args) {
    int usedObjs = 0;
    for (unsigned int i=0; i<points.size(); i++) {
        if (!points[i].used){
            usedObjs++;
        }
    }
    return Py_BuildValue("i",usedObjs);
}

static PyObject* clear(PyObject* self, PyObject* args) {
    _x.clear();
    _y.clear();
    _z.clear();

    _nx.clear();
    _ny.clear();
    _nz.clear();

    _c.clear();

    points.clear();

    return Py_BuildValue("");
}

static PyObject* get_size(PyObject* self, PyObject* args) {
	int size = points.size();

    return Py_BuildValue("i", size);
}

static PyObject* get_foo_objects(PyObject* self, PyObject* args) {

    std::vector<float> point;
    std::vector< std::vector<float> > object;
    std::vector< std::vector< std::vector<float> > > objects;

    int length = 10;

    for (int k=0; k<5; k++) {
        for (int i=0; i<length; i++) {
            for (int j=0; j<3; j++) {
                point.push_back(i+j);
            }
            object.push_back(point);
            point.clear();
        }
        objects.push_back(object);
        length--;
    }

    return vectorVectorVectorToTuple(objects);
}

// ========================================================================== //
//                                 WRAPPER                                    //
// ========================================================================== //

/*  define functions in module */
static PyMethodDef nscanMethods[] = {
    {"set_x", set_x, METH_VARARGS, "set x values"},
    {"set_y", set_y, METH_VARARGS, "set y values"},
    {"set_z", set_z, METH_VARARGS, "set z values"},
    {"set_nx", set_nx, METH_VARARGS, "set nx values"},
    {"set_ny", set_ny, METH_VARARGS, "set ny values"},
    {"set_nz", set_nz, METH_VARARGS, "set nz values"},
    {"set_c", set_c, METH_VARARGS, "set c values"},
    {"initialize", initialize, METH_VARARGS, "initialize points"},
    {"connect", connect, METH_VARARGS, "connect each point to neighbors"},
    {"NSCAN", NSCAN, METH_VARARGS, "normal spreading segmentation"},
    {"clear", clear, METH_VARARGS, "clear all points"},

    {"foo", foo, METH_VARARGS, "random tests"},
    {"get_size", get_size, METH_VARARGS, "get size of list"},
    {"get_foo_objects", get_foo_objects, METH_VARARGS, "test of list of lists"},
    {NULL, NULL, 0, NULL},
};

#if PY_MAJOR_VERSION >= 3
/* module initialization */
/* Python version 3*/
static struct PyModuleDef cModPyDem = {
    PyModuleDef_HEAD_INIT,
    "nscan", "Some documentation",
    -1,
    nscanMethods
};

PyMODINIT_FUNC
PyInit_nscan(void) {
    return PyModule_Create(&cModPyDem);
}

#else

/* module initialization */
/* Python version 2 */
PyMODINIT_FUNC
init_nscan(void) {
    (void) Py_InitModule("nscan",nscanMethods);
}

#endif
