class PoissonParam
{
public:
	PoissonParam()
	{
		Depth=6;
		SolverDivide=8;
		IsoDivide=8;
		Refine=3;
		SamplesPerNode=1.0f;		
		Scale=1.25f;
		KernelDepth = -1;
		Offset = 1.0;
		BoundaryType = 1;
    MinDepth = 5;
    constraintWeight = 0.0;
    adaptiveExponent = 1;
    MinIters = 8;
    SolverAccuracy = 1e-3;
    MaxSolveDepth = 9;
    FixedIters = -1;
    VoxelDepth = -1;
    Threads = 8;

    NoResetSamples = false;
		NoClipTree = false;
		Confidence = true;
    ShowResidual = false;
    NonManifold = true;
    PolygonMesh = false;
		
	}
	bool Verbose,NoResetSamples,NoClipTree,Confidence, ShowResidual, NonManifold, PolygonMesh;

	float Offset; // an hacked offset value. if == 1 no offset. 0.5.. 2 are good values.

	
	int Depth;
	int SolverDivide;
	int IsoDivide;
	int Refine;
	int KernelDepth;
  int BoundaryType;
  int MinDepth;
  int adaptiveExponent;
  int MinIters;
  int MaxSolveDepth;
  int FixedIters;
  int VoxelDepth;
  int Threads;

  
  float constraintWeight;
	float SamplesPerNode;
	float Scale;
  float SolverAccuracy;
//	char* paramNames[]=
//	{
//		"in","depth","out","refine","noResetSamples","noClipTree",
//		"binary","solverDivide","isoDivide","scale","verbose",
//		"kernelDepth","samplesPerNode","confidence"
//	};
//	cmdLineReadable* params[]=
//	{
//		&In,&Depth,&Out,&Refine,&NoResetSamples,&NoClipTree,
//		&Binary,&SolverDivide,&IsoDivide,&Scale,&Verbose,
//		&KernelDepth,&SamplesPerNode,&Confidence
//	};
}
;



/*


[--depth <reconstruction depth>] 
This integer is the maximum depth of the tree that will be used for surface reconstruction. Running at depth d corresponds to solving on a voxel grid whose resolution is no larger than 2^d x 2^d x 2^d. Note that since the reconstructor adapts the octree to the sampling density, the specified reconstruction depth is only an upper bound.
The default value for this parameter is 8. 
[--minDepth <adaptive octree depth>] 
This integer specifies the depth beyond depth the octree will be adapted. At coarser depths, the octree will be complete, containing all 2^d x 2^d x 2^d nodes.
The default value for this parameter is 5. 
[--pointWeight <interpolation weight>] 
This floating point value specifies the importants that interpolation of the point samples is given in the formulation of the screened Poisson equation.
The results of the original (unscreened) Poisson Reconstruction can be obtained by setting this value to 0.
The default value for this parameter is 4. 
[--threads <number of processing threads>] 
This integer specifies the number of threads across which the reconstruction algorithm should be parallelized.
The default value for this parameter is equal to the numer of (virtual) processors on the executing machine. 
[--scale <scale factor>] 
This floating point value specifies the ratio between the diameter of the cube used for reconstruction and the diameter of the samples' bounding cube.
The default value is 1.1. 
[--solverDivide <solver subdivision depth>] 
This integer argument specifies the depth at which a block Gauss-Seidel solver is used to solve the Laplacian equation. Using this parameter helps reduce the memory overhead at the cost of a small increase in reconstruction time. (In practice, we have found that for reconstructions of depth 9 or higher a subdivide depth of 7 or 8 can greatly reduce the memory usage.)
The default value is 8. 
[--isoDivide <iso-surface extraction subdivision depth>] 
This integer argument specifies the depth at which a block iso-surface extractor should be used to extract the iso-surface. Using this parameter helps reduce the memory overhead at the cost of a small increase in extraction time. (In practice, we have found that for reconstructions of depth 9 or higher a subdivide depth of 7 or 8 can greatly reduce the memory usage.)
The default value is 8. 
[--samplesPerNode <minimum number of samples>] 
This floating point value specifies the minimum number of sample points that should fall within an octree node as the octree construction is adapted to sampling density. For noise-free samples, small values in the range [1.0 - 5.0] can be used. For more noisy samples, larger values in the range [15.0 - 20.0] may be needed to provide a smoother, noise-reduced, reconstruction.
The default value is 1.0. 
[--confidence] 
Enabling this flag tells the reconstructor to use the size of the normals as confidence information. When the flag is not enabled, all normals are normalized to have unit-length prior to reconstruction. 
[--polygonMesh] 
Enabling this flag tells the reconstructor to output a polygon mesh (rather than triangulating the results of Marching Cubes). 
[--density] 
Enabling this flag tells the reconstructor to output the estimated depth values of the iso-surface vertices. 
[--verbose] 
Enabling this flag provides a more verbose description of the running times and memory usages of individual components of the surface reconstructor. 




*/