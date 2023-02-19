/*
* calCableSqrDis_CGAL.cpp
*
* use CGAL to calculate squared distance between 2 cables in 3D
*
* Usage : from MATLAB
*         >> outMatrix = calCableSqrDis_CGAL(inMatrix1, inMatrix2) 
* inMatrix:3X2, two end-points of the cable
* This is a C++ MEX-file for MATLAB.
* Copyright 2021 汤凯 HITSZ.
*
*/
#include "mex.hpp"
#include "mexAdapter.hpp"
#include <iostream>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/squared_distance_3.h>

typedef double FT; //define field number type: double
typedef CGAL::Simple_cartesian<FT> Kernel;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Segment_3 Segment_3;

class MexFunction : public matlab::mex::Function {
public:
    void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        checkArguments(outputs, inputs);
        matlab::data::TypedArray<double> inSeg1 = std::move(inputs[0]);
        matlab::data::TypedArray<double> inSeg2 = std::move(inputs[1]);

        matlab::data::ArrayFactory factory;
        outputs[0] = factory.createScalar(calCableSqrDis_CGAL(inSeg1, inSeg2));

    }

    double calCableSqrDis_CGAL(matlab::data::TypedArray<double>& inS1, matlab::data::TypedArray<double>& inS2) {

        Point_3 seg1P1(inS1[0][0], inS1[1][0], inS1[2][0]), seg1p2(inS1[0][1], inS1[1][1], inS1[2][1]);
        Point_3 seg2P1(inS2[0][0], inS2[1][0], inS2[2][0]), seg2p2(inS2[0][1], inS2[1][1], inS2[2][1]);
        Segment_3 seg1(seg1P1, seg1p2), seg2(seg2P1, seg2p2);
        FT distance;

        distance = CGAL::squared_distance(seg1, seg2);
        return distance;
    }

    void checkArguments(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();
        matlab::data::ArrayFactory factory;

        if (inputs.size() != 2) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("Two inputs required") }));
        }

        if (inputs[0].getDimensions()[0] != 3 || inputs[0].getDimensions()[1] != 2) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("segment1 points Input must be 3X2 dimension") }));
        }
        
        if (inputs[0].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[0].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("segment1 points Input multiplier must be a noncomplex scalar double") }));
        }

        if (inputs[1].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[1].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("segment2 points Input matrix must be type double") }));
        }

        if (inputs[1].getDimensions()[0] != 3 || inputs[1].getDimensions()[1] != 2) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("segment2 points Input must be 3X2 dimension") }));
        }
    }
};