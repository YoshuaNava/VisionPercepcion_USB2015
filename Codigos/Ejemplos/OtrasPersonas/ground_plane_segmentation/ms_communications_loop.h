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

#ifndef __MS_COMMUNICATIONS_LOOP__
#define __MS_COMMUNICATIONS_LOOP__

#include "ms_overwrite_safe_buffer.h"

#include <iostream>
using std::cout;
using std::endl;

#include <chrono>
#include <thread>

#include <string>
using std::string;

#define DUMMY 0
#define QUAD 1
#define ROVER 2
#define VISAR 3
#define EMPTY 4

#include "Net.h" //header for UDP transmission


class CommLoop{
    int comm_time_;
    bool stop_;
    int watch_dog_;
    int wait_limit_;

    int robot_id_;
    bool verb_;
    bool use_network_;
    int ip1_,ip2_,ip3_,ip4_,port_;
    bool debug_;
    
    net::Socket * socPtr_;
    
    public:
    CommLoop(int robot_id,bool verb,bool use_network,int ip1,int ip2,int ip3,int ip4,int port,bool debug, net::Socket *socPtr):
        comm_time_(50),
        stop_(false),
        watch_dog_(5),
        wait_limit_(5),
        robot_id_(robot_id),
        verb_(verb),
        use_network_(use_network),
        ip1_(ip1),ip2_(ip2),ip3_(ip3),ip4_(ip4),port_(port),
        debug_(debug),
        //socPtr_(new net::Socket)
        socPtr_(nullptr)
        {
            //socPtr = new(net::Socket);
            if(InitSocketUdp(socPtr)) exit(1);
        }

    void Send(OwSafeBuffer<string> *commbuf, net::Socket *socPtr){
        static int initial_sleep_until_setup = 1;
        if(initial_sleep_until_setup ){
            std::this_thread::sleep_for(std::chrono::milliseconds(2000)); 
            initial_sleep_until_setup =0;
        }
        bool empty = commbuf->IsEmpty();
        
        cout << "\n>>Sending message to vehicle.. " << "isempty=" << empty << "" ;
        if(empty) --watch_dog_;
        else{
            string command = commbuf->Pop();
            if(use_network_){
                cout << ", command=\"" << command << "\"" << endl;
                socPtr->Send( net::Address(ip1_,ip2_,ip3_,ip4_,port_), command.c_str(), sizeof(command.c_str()) );}
            watch_dog_ = watch_dog_ > wait_limit_ ? wait_limit_ : ++watch_dog_;
        }

        if(watch_dog_>0){
            //sleep(comm_time_);
            std::this_thread::sleep_for(std::chrono::milliseconds(comm_time_));
        }
        else
            stop_=true;
    }
    void Start(OwSafeBuffer<string> *commbuf, net::Socket *socPtr){
        while(!stop_){
            Send(commbuf,socPtr);
        }
        if(stop_)
            cout << "warning communication has stopped!!!!\n" << endl;
    }
    void Test(int dummy){
        cout << "dummy= " << dummy << endl;
    }
    int CommTime(){ 
        return 1;
    }
    int InitSocketUdp(net::Socket *socPtr)
    {
        //if(socPtr==nullptr) socPtr = new( net::Socket );
        printf( "creating socket on port %d\n", port_ );
        cout << ip1_ << "." << ip2_ << "." << ip3_ << "." << ip4_ << ":" << port_ << endl;
        if ( !socPtr->Open( port_ ) )
        {   printf( "failed to create socket!\n" );
        return 1;   
        }else printf( "successfully created socket!\n" );
        
        return 0;
    }
};


#endif