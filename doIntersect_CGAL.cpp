/*
* doIntersect_CGAL.cpp
*
* use CGAL to test intersection between lines, triangles, tetrahedrons
*
* Usage : from MATLAB
*         >> outMatrix = doIntersect_CGAL(inMatrix1, inMatrix2)
*
* This is a C++ MEX-file for MATLAB.
* Copyright 2021 汤凯 HITSZ.
*
*/
#include "mex.hpp"
#include "mexAdapter.hpp"
#include <iostream>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/intersections.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Segment_3 Segment_3;
typedef Kernel::Triangle_3 Triangle_3;
typedef Kernel::Tetrahedron_3 Tetrahedron_3;

class MexFunction : public matlab::mex::Function {
public:
    void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        checkArguments(outputs, inputs);
        matlab::data::TypedArray<double> inShape1 = std::move(inputs[0]);
        matlab::data::TypedArray<double> inShape2 = std::move(inputs[1]);

        matlab::data::ArrayFactory factory;
        outputs[0] = factory.createScalar(doIntersect_CGAL(inShape1, inShape2));

    }

    bool doIntersect_CGAL(matlab::data::TypedArray<double>& inS1, matlab::data::TypedArray<double>& inS2) {
        std::vector<Point_3> shape1_Points, shape2_Points;
        size_t shape1_Pnum = inS1.getDimensions()[1];
		size_t shape2_Pnum = inS2.getDimensions()[1];

        for (size_t  i = 0; i != shape1_Pnum; ++i)
        {
        	Point_3 point(inS1[0][i], inS1[1][i], inS1[2][i]);
        	shape1_Points.push_back(point);
        }

        for (size_t  i = 0; i != shape2_Pnum; ++i)
        {
        	Point_3 point(inS2[0][i], inS2[1][i], inS2[2][i]);
        	shape2_Points.push_back(point);
        }

       if (shape1_Pnum == 2)
       {
       		Segment_3 shape1(shape1_Points[0], shape1_Points[1]);
          	if (shape2_Pnum == 2)
	       {
	       		Segment_3 shape2(shape2_Points[0], shape2_Points[1]);
	          	return CGAL::do_intersect(shape1, shape2);
	       }
	       else if (shape2_Pnum == 3)
	       {
	       		Triangle_3 shape2(shape2_Points[0], shape2_Points[1], shape2_Points[2]);
	       		return CGAL::do_intersect(shape1, shape2);
	       }
	       else
	       {
	       		Tetrahedron_3 shape2(shape2_Points[0], shape2_Points[1], shape2_Points[2], shape2_Points[3]);
	       		return CGAL::do_intersect(shape1, shape2);
	       }
       }
       else if (shape1_Pnum == 3)
       {
       		Triangle_3 shape1(shape1_Points[0], shape1_Points[1], shape1_Points[2]);
       		if (shape2_Pnum == 2)
	       {
	       		Segment_3 shape2(shape2_Points[0], shape2_Points[1]);
	          	return CGAL::do_intersect(shape1, shape2);
	       }
	       else if (shape2_Pnum == 3)
	       {
	       		Triangle_3 shape2(shape2_Points[0], shape2_Points[1], shape2_Points[2]);
	       		return CGAL::do_intersect(shape1, shape2);
	       }
	       else
	       {
	       		Tetrahedron_3 shape2(shape2_Points[0], shape2_Points[1], shape2_Points[2], shape2_Points[3]);
	       		return CGAL::do_intersect(shape1, shape2);
	       }
       }
       else
       {
       		Tetrahedron_3 shape1(shape1_Points[0], shape1_Points[1], shape1_Points[2], shape1_Points[3]);
       		if (shape2_Pnum == 2)
	       {
	       		Segment_3 shape2(shape2_Points[0], shape2_Points[1]);
	          	return CGAL::do_intersect(shape1, shape2);
	       }
	       else if (shape2_Pnum == 3)
	       {
	       		Triangle_3 shape2(shape2_Points[0], shape2_Points[1], shape2_Points[2]);
	       		return CGAL::do_intersect(shape1, shape2);
	       }
	       else
	       {
	       		Tetrahedron_3 shape2(shape2_Points[0], shape2_Points[1], shape2_Points[2], shape2_Points[3]);
	       		return CGAL::do_intersect(shape1, shape2);
	       }
       }
        
    }


    void checkArguments(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();
        matlab::data::ArrayFactory factory;

        if (inputs.size() != 2) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Two inputs required") }));
        }

        if (inputs[0].getDimensions()[0] != 3) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("shape1 Input must be 3XN dimension") }));
        }

        if (inputs[0].getDimensions()[1] < 2 || inputs[0].getDimensions()[1] > 4) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("shape1 Input must be line segment, triangle or tetrahedra") }));
        }
        
        if (inputs[0].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[0].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("shape1 Input multiplier must be type double") }));
        }

        if (inputs[1].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[1].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("shape2 Input matrix must be type double") }));
        }

        if (inputs[1].getDimensions()[0] != 3) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("shape2 Input must be 3XN dimension") }));
        }

        if (inputs[1].getDimensions()[1] < 2 || inputs[1].getDimensions()[1] > 4) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("shape2 Input must be line segment, triangle or tetrahedra") }));
        }
    }
};