// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <iostream>
#include <cmath>

#include "simulator.h"
/*
#include "vertex_se2.h"
#include "vertex_point_xy.h"
#include "edge_se2.h"
#include "edge_se2_pointxy.h"
#include "types_tutorial_slam2d.h"
 */
#include "vertex_se3.h"
#include "vertex_pointxyz.h"
#include "edge_se3.h"
#include "se3quat.h"
#include "edge_se3_pointxyz.h"
//#include "types_tutorial_slam3d.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

//#include "hello.h"

using namespace std;
using namespace g2o;
using namespace g2o::tutorial;


int main()
{

	typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
	typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

	// allocating the optimizer
	SparseOptimizer optimizer;
	SlamLinearSolver* linearSolver = new SlamLinearSolver();
	linearSolver->setBlockOrdering(false);
	SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
	OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(blockSolver);

	optimizer.setAlgorithm(solver);

	cout << "optimizer decleared" << endl;

// *********************************************************************************
	// * 最適化処理
	// ********************************************************************************

	optimizer.load("bfr.csv");	//ファイル読み込み

	// prepare and run the optimization
	// fix the first robot pose to account for gauge freedom
	VertexSE3* firstRobotPose = dynamic_cast<VertexSE3*>(optimizer.vertex(0));
	firstRobotPose->setFixed(true);
	optimizer.setVerbose(true);

	cerr << "Optimizing" << endl;
	optimizer.initializeOptimization();
	optimizer.optimize(10);		//最適化処理
	cerr << "done." << endl;

	optimizer.save("aft.csv");

	cout << "saved." << endl;

	// freeing the graph memory
	optimizer.clear();

	// destroy all the singletons
	Factory::destroy();
	OptimizationAlgorithmFactory::destroy();
//	HyperGraphActionLibrary::destroy();

	return 0;
}
