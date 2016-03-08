/*
 *  Copyright (C) <2014>  <Michael Sapienza>
 *  
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//ms_overwrite_safe_buffer.h
#ifndef __MS_OVERWRITE_SAFE_BUFFER__
#define __MS_OVERWRITE_SAFE_BUFFER__

#include <queue>
using std::queue;

template <class Dtype>
class OwSafeBuffer{
    queue<Dtype> qbuf_;
    int size_;
    void Empty();
    
public:
    OwSafeBuffer(int size);
    Dtype Pop();
    void Reset(Dtype contents);
    void Fill(const Dtype &contents);
    bool IsEmpty();
};


#endif