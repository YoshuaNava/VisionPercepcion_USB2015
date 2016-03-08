Surface Patch Library (SPL)

This is a Matlab toolbox for surface patch modeling and fitting as described
in the papers

Marsette Vona and Dimitrios Kanoulas, "Curved Surface Contact Patches with
Quantified Uncertainty", IROS 2011

Dimitrios Kanoulas and Marsette Vona, "Sparse Surface Modeling with Curved
Patches", ICRA 2013

For a high-level demonstration try running demoautoseg or demomanseg.

HIGH-LEVEL DRIVERS: demo{manseg,autoseg,fit,spl}
PATCH FUNCTIONS TEST/DEMO: testpatch{fit,pplot,sample,residual,coverage,area,es}
OTHER TEST/DEMO: test{approx,rexp,rangecovar}
PATCH FUNCTIONS: patch{fit,plot,print,sample,residual,coverage,chk}
SEGMENTATION: manseg, autoseg
RANGE DATA COVARIANCE FUNCTIONS: rangecovar, testrangecovar, covarplot
EXPMAP: r{exp,log,canon2,reparam}, dr{exp,log,canon2}, xform3, rtoq, qtor
SAMPLE HANDLING: sample{cvt,frustum,load,save,pick,plot,mesh,searcher}, datafn
SMALL ANGLE APPROXIMATIONS: alpha, beta, dalpha, dbeta, approxbase
MATH TOOLS: wlm, lm, lls, numj, quadsol, inrball, cpm
FIGURE FUNCTIONS: figaa, axescfg, subtitle, saveframe, hgbbox, fitbbox
DRAWING FUNCTIONS: frustumplot, draw{curves,triad}
CAMERA FUNCTIONS: aim, camxform, {set,get}cam

LICENSE: GPL.  See LICENSE.txt

DISCLAIMER:

THIS INFORMATION AND/OR SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS INFORMATION AND/OR
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Copyright (C) 2013 Marsette A. Vona except where specified

