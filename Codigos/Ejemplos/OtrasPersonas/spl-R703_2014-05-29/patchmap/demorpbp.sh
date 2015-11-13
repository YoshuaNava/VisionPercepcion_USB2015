#!/bin/sh

./test_map_kinfu_app -imucam_grabber "#1 -norgb" -bf -bc -br 200,0,0,0,0,0  -bo 0,-1,0 -drawcam 0.35,1 -ndv -nrv -downgrav -mvfd -mvcnn -mvcv -drawdown -bubcamloc 5,-2,-2 -campos 1,1,1 -indcam -vs 2 -drawdown -saldon -saldong -salfpp 0.4,0.1,0.0 -salmdon 0.9645 -salmdong 0.8195 -salmdtfp 0.5 -mapsc 100 -mapmpmt 60 -mapmp 1000 -mapcdf 0.1 -mapmppc 5 -mapdrawsp -mapdrawpn -mapdrawpatches -mapr 0.05 -selp -selpt 0.1,0.05,0.32 -selptr 0.025 -selmaxcurv 1 -selmincurv -1 -selmaxslope 10

#./test_map_kinfu_app -imucam_grabber "#1 -norgb -um6 -um6port /dev/ttyUSB1 -um6nomag" -bf -bc -br 200,0,0,0,0,0  -bo 0,-1,0 -drawcam 0.35,1 -ndv -nrv -downgrav -mvfd -mvcnn -mvcv -drawdown -bubcamloc 5,-2,-2 -campos 1,1,1 -indcam -vs 2 -drawdown -saldon -saldong -salfpp 0.4,0.1,0.0 -salmdon 0.9645 -salmdong 0.8195 -salmdtfp 0.5 -mapsc 100 -mapmpmt 60 -mapmp 1000 -mapcdf 0.1 -mapmppc 5 -mapdrawsp -mapdrawpn -mapdrawsal -mapr 0.05 -selpt 0.1,0.05,0.32 -selptr 0.025 -selmaxcurv 1 -selmincurv -1 -selmaxslope 10
