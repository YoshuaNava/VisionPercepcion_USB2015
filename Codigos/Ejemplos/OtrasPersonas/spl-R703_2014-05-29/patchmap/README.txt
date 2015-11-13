Surface Patch Library (SPL) using the Point Cloud Library (PCL)

This is a C++ implementation of a subset of modules from the Surface Patch
Library (SPL) MATLAB toolbox using the Point Cloud Library (PCL).  All the
algorithms are described in the papers:

[1] Marsette Vona and Dimitrios Kanoulas, "Curved Surface Contact Patches with
Quantified Uncertainty", IROS 2012.
[2] Dimitrios Kanoulas and Marsette Vona, "Sparse Surface Modeling with Curved
Patches", ICRA 2013.
[3] Dimitrios Kanoulas and Marsette Vona, "Bio-Inspired Rough Terrain Contact 
Patch Perception", ICRA 2014.

-------------------------------------------------------------------------------
COMPILING FROM SOURCE

See the makefile and makefile.project for more info about compiling the code.  
Under spl/patchmap run:
> make

-------------------------------------------------------------------------------
DEPENDENCIES

imucam: http://www.ccs.neu.edu/research/gpc/imucam

rxkinfu: http://www.ccs.neu.edu/research/gpc/rxkinfu

PCL trunk: http://www.pointclouds.org (PCL modules common, io, visualization, features, filters, segmentation, surface)

-------------------------------------------------------------------------------
MODULES

FILTERS: filter_{approx_voxel_grid_organized, dec, lowres, nms}
PATCH FUNCTIONS: patch, patch_{fit, plot, select, sample, set}
SAMPLE HANDLING: sample_{filter, saliency}, integral_image_features
SEGMENTATION: autoseg, task
PLANNING: patch_selection
MAPPING: map_{kinfu_app, patch, cell}
MATH TOOLS: xform3
TEST PATCH FUNCTION: test_patch_{plot, sample, fit, residual}
TEST MAP FUNCTION: test_map_kinfu_app
TEST FUNCTION: test_{filter_nms, rexp}

-------------------------------------------------------------------------------
LICENSE: BSD.  See LICENSE.txt

DISCLAIMER:
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
POSSIBILITY OF SUCH DAMAGE.

Copyright (C) 2013 Dimitrios Kanoulas except where specified
