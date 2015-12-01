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

#include <iostream>
using std::cout;
using std::endl;

#include <string>
using std::string;

#include <cassert>

#include <thread>
#include <mutex>

std::mutex mtx;

#include "ms_overwrite_safe_buffer.h"

template< class Dtype >
OwSafeBuffer<Dtype>::OwSafeBuffer(int size):size_(size){
}
template class OwSafeBuffer<int>;
template class OwSafeBuffer<string>;


template< class Dtype >
bool OwSafeBuffer<Dtype>::IsEmpty(){
    std::lock_guard<std::mutex> lck (mtx);
    return qbuf_.empty();
}
template bool OwSafeBuffer<int>::IsEmpty();
template bool OwSafeBuffer<string>::IsEmpty();


template< class Dtype >
Dtype OwSafeBuffer<Dtype>::Pop(){
    std::lock_guard<std::mutex> lck (mtx);
    assert(!qbuf_.empty());

    Dtype contents = qbuf_.front();
    qbuf_.pop();
    return contents;
    
}
template int OwSafeBuffer<int>::Pop();
template string OwSafeBuffer<string>::Pop();


template< class Dtype >
void OwSafeBuffer<Dtype>::Reset(Dtype contents){
    // using a local lock_guard to lock mtx guarantees unlocking on destruction / exception:
    std::lock_guard<std::mutex> lck (mtx);
    Empty();
    Fill(contents);
}
template void OwSafeBuffer<int>::Reset(int contents);
template void OwSafeBuffer<string>::Reset(string contents);


template< class Dtype >
void OwSafeBuffer<Dtype>::Fill(const Dtype &contents){
    assert(qbuf_.empty()==true);
    cout << "Filling.. ";
    for(int i=0;i<size_;++i){
        qbuf_.push(contents);
        cout << qbuf_.back();
    }
    cout << endl;
}
template void OwSafeBuffer<int>::Fill(const int &contents);
template void OwSafeBuffer<string>::Fill(const string &contents);

template< class Dtype >
void OwSafeBuffer<Dtype>::Empty(){
    cout << "Empty.. ";
    while(!qbuf_.empty()){
        cout << qbuf_.front();
        qbuf_.pop();
    }
    cout << endl;
}
template void OwSafeBuffer<int>::Empty();
template void OwSafeBuffer<string>::Empty();