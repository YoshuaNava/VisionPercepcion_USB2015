/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-, Dimitrios Kanoulas
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#define _CRT_SECURE_NO_DEPRECATE
#include <iostream>

//PCL headers
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/exceptions.h>

//SPL-CPP headers
#include "patch.h"
#include "patch_fit.h"
#include "patch_plot.h"

//RXKINFU headers
//#include "kinfu_app.h"
#include "map_kinfu_app.h"

using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace rxkinfu;

/** \brief Main loop. */
int
main(int argc, char **argv)
{    
  if (find_switch(argc, argv, "-h"))
  {
    MapKinfuApp::usageHelp();
    return 0;
  }
  else
  {
    print_highlight("run patchMap -h for command line help, hit h for online help\n");
  }
  
  try
  {
    MapKinfuApp(argc, argv).mainLoop();
  }
  catch (const PCLException &e)
  {
    cerr << "PCLException: " << e.what() << endl;
    return (-1);
  }
  catch (const std::bad_alloc &e)
  {
    cerr << "Bad alloc: " << e.what() << endl;
    return (-1);
  }
  catch (const std::exception &e)
  {
    cerr << "Exception: " << e.what() << endl;
    return (-1);
  }
  
  return 0;
}
