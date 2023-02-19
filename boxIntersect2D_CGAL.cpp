/*
* boxIntersect2D_CGAL.cpp
*
* use CGAL to test 2D aligned box intersection with 2D line segment or triangle
*
* Usage : from MATLAB
*         >> outMatrix = boxIntersect2D_CGAL(inMatrix1, inMatrix2)
* inMatrix1:2XN1(N1=2,3), line, triangle
* inMatrix2:2X2, diagonal opposite vertices p and q of a triangular box
* This is a C++ MEX-file for MATLAB.
* Copyright 2021 汤凯 HITSZ.
*
*/
#include "mex.hpp"
#include "mexAdapter.hpp"
#include <iostream>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/intersections.h>
#include <CGAL/Iso_rectangle_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Segment_2 Segment_2;
typedef Kernel::Triangle_2 Triangle_2;
typedef Kernel::Iso_rectangle_2 Rectangle_2;

class MexFunction : public matlab::mex::Function {
public:
    void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        checkArguments(outputs, inputs);
        matlab::data::TypedArray<double> inShape = std::move(inputs[0]);
        matlab::data::TypedArray<double> inBox = std::move(inputs[1]);

        matlab::data::ArrayFactory factory;
        outputs[0] = factory.createScalar(boxIntersect2D_CGAL(inShape, inBox));

    }

    bool boxIntersect2D_CGAL(matlab::data::TypedArray<double>& inShape, matlab::data::TypedArray<double>& inBox) {
        std::vector<Point_2> inShape_Points;
        size_t inShape_Pnum = inShape.getDimensions()[1];
        
        Point_2 box_p(inBox[0][0], inBox[1][0]), box_q(inBox[0][1], inBox[1][1]);
        Rectangle_2 box(box_p, box_q);

        for (size_t  i = 0; i != inShape_Pnum; ++i)
        {
          Point_2 point(inShape[0][i], inShape[1][i]);
          inShape_Points.push_back(point);
        }

       if (inShape_Pnum == 2) //line segment and box
       {
          Segment_2 shape(inShape_Points[0], inShape_Points[1]);
          return CGAL::do_intersect(shape, box);
       }
       else //triangle and box
       {
          Triangle_2 shape(inShape_Points[0], inShape_Points[1], inShape_Points[2]);
          return CGAL::do_intersect(shape, box);
       }
        
    }


    void checkArguments(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();
        matlab::data::ArrayFactory factory;

        if (inputs.size() != 2) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Two inputs required") }));
        }

        if (inputs[0].getDimensions()[0] != 2) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("2D shape Input must be 2XN(N=2,3) dimension") }));
        }

        if (inputs[0].getDimensions()[1] < 2 || inputs[0].getDimensions()[1] > 3) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("2D shape Input must be 2D line segment or triangle") }));
        }
        
        if (inputs[0].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[0].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("2D shape Input multiplier must be type double") }));
        }

        if (inputs[1].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[1].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("2D Box Input matrix must be type double") }));
        }

        if (inputs[1].getDimensions()[0] != 2 || inputs[1].getDimensions()[1] != 2) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("2D Box Input must be 2X2 dimension, opposite vertices") }));
        }
    }
};