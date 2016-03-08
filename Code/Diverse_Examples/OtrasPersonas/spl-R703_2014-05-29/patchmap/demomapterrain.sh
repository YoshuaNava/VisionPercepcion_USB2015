#!/bin/sh

./test_map_kinfu_app -imucam_reader "$@" -bf -bc -br 200,0,0,0,0,0 -bo 0,-1,0 -drawcam -camaa -1,1,0,0 -ndv -nrv -downgrav -drawdown -mvfd -mvcnn -mvcv -bubcam -bubcamloc 10,-4,-4 -campos 2,1.5,1.5 -vs 4 -b -indcam -headcam -drawhead -drawdown -usesal -saldon -salfpp 0.4,0.1,0.0 -salmdon 0.9645 -salmdong 0.8195 -salmdtfp 0.5 -mapsc 50 -mapmpmt 60 -mapmp 1000 -mapcdf 0.5 -mapmppc 5 -mapdrawsp -mapdrawpn -mapdrawpn -mapdrawcells -mapdrawgci -mapdrawgcit 3 -mapr 0.05 -selpt 0.0,0.0,0.8 -selptr 0.5 -selmaxcurv 1 -selmincurv -1 -selmaxslope 10
