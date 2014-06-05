/*
 * Copyright © 2012, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 * 
 * The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
 * under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0.
 * 
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
*/
    
// Adapted from http://www.fundza.com/algorithmic/space_filling/hilbert/basics/index.html
/*
procedure hilbert(x, y, xi, xj, yi, yj, n)
// x and y are the coordinates of the bottom left corner
// xi & xj are the i & j components of the unit x vector of the frame
// similarly yi and yj
if (n <= 0) then
   LineTo(x + (xi + yi)/2, y + (xj + yj)/2);
else
   {
   hilbert(x,           y,           yi/2, yj/2,  xi/2,  xj/2, n-1);
   hilbert(x+xi/2,      y+xj/2 ,     xi/2, xj/2,  yi/2,  yj/2, n-1);
   hilbert(x+xi/2+yi/2, y+xj/2+yj/2, xi/2, xj/2,  yi/2,  yj/2, n-1);
   hilbert(x+xi/2+yi,   y+xj/2+yj,  -yi/2,-yj/2, -xi/2, -xj/2, n-1);
   }
end procedure;
*/

#include "core/tgModel.h" 

#include "core/tgSubject.h"

#include "core/tgPairs.h"
#include "tgcreator/tgUtil.h"
#include "core/tgRod.h"
#include "tgcreator/tgRodInfo.h"

#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"

// The C++ Standard Library
#include <iostream>

class SingleRibModel : public tgSubject<SingleRibModel>, public tgModel
{
public:

    SingleRibModel(int n = 1, double xsize = 10, double ysize = 0) : 
        m_n(n), 
        m_xsize(xsize), 
        m_ysize(ysize)
    {
        if(m_ysize == 0) {
            m_ysize = m_xsize;
        }
        
    }

    virtual void setup(tgWorld& world)
    {
        
        tgStructure tetra;
        
        ellipseNodes(tetra, 10, 5, 0, M_PI, m_n);

        makePairs(tetra);
        
        // Just get it out of the way of the other structure
        tetra.move(btVector3(25.0, 5.0, 0));
        
        const double density = 0.9;
        const double radius  = 0.5;
        const tgRod::Config rodConfig(radius, density);

        tgBuildSpec spec;
        spec.addBuilder("rod", new tgRodInfo(rodConfig));
        
        // Create your structureInfo
        tgStructureInfo structureInfo(&tetra, &spec);

        // Use the structureInfo to build ourselves
        structureInfo.buildInto(this, world);
    }


    void ellipseNodes(tgStructure& tetra, double a, double b, double startT, double endT, size_t n) 
    {
        double rodL = (endT - startT) / (double) n;
        
        for(size_t i = 0; i < n; i++)
        { 
            double x = a * cos( startT + i * rodL);
            double y = b * sin( startT + i * rodL);
            // Just build it in xy, can rotate later
            tgNode node( btVector3( x, y, 0.0));
            tetra.addNode( node );
        }
    }
    
    void makePairs(tgStructure& tetra) 
    {   
        size_t n = tetra.getNodes().size();
        std::cout << "Nodes size is " << n << std::endl;
        for(int i = 1; i < n; i++) {
            tetra.addPair(i-1, i, "rod");
        }
    }    

private:
    int m_n;
    double m_xsize;
    double m_ysize;
    
};
