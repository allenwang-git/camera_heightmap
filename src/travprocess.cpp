//
// Created by allen on 2020/11/2.
//
//lcm
#include "lcm/lcm-cpp.hpp"
#include "heightnew_t.hpp"
#include "traversability_float_t.hpp"

//#include "eigen3"
#include "travprocess.h"



int main (int argc, char** argv)
{
    Handler handlerobj
    traversability_float_t trav_lcm;
    lcm.subscribe("heightmapnew", &Handler::heighthandler, &handlerobj);
    _LCMThread = std::thread(&Handler::LCMThread, this);
    _height_mapnew = DMat<float>::Zero(xnew_size, ynew_size);
    float w_sd=1.,w_sl=2.,w_max=2.,w_min=2.;//score wieght

    for (size_t i(0); i < xnew_size; ++i) {//101
        for (size_t j(0); j < ynew_size; ++j) {//101
//            float travmap[100][100];
            if (i>=1&&j>=1&&i+1<xnew_size&&j+1<ynew_size)
            {
                float slope=0, sum=0;
                float hmax=_height_mapnew(i,j), hmin=_height_mapnew(i,j);
                float mean,sd,score;
                float tmpmap[9];
                int k=0;
                for (size_t m(i-1); m<=(i+1);m++){
                    for (size_t n(j-1); n<=(j+1);n++){
                        if (m!=i && n!=j)
//                            cout<<_height_mapnew(m,n)<<endl;
                            // slope
                            slope=abs((_height_mapnew(i, j)-_height_mapnew(m, n))/sqrt((int)((i-m)*(i-m)+(j-n)*(j-n))))+slope;
                        //max
                        if (_height_mapnew(m,n)>hmax)
                            hmax=_height_mapnew(m,n);
                        //min
                        if (_height_mapnew(m,n)<hmin)
                            hmin=_height_mapnew(m,n);
                        sum=sum+_height_mapnew(m,n);
                        tmpmap[k]=_height_mapnew(m,n);
                        k++;
                    }
                }
                //标准差
                mean=sum/9;
                sd=sqrt((tmpmap[0]-mean)*(tmpmap[0]-mean)+(tmpmap[1]-mean)*(tmpmap[1]-mean)+(tmpmap[2]-mean)*(tmpmap[2]-mean)+(tmpmap[3]-mean)*(tmpmap[3]-mean)+(tmpmap[4]-mean)*(tmpmap[4]-mean)
                        +(tmpmap[5]-mean)*(tmpmap[5]-mean)+(tmpmap[6]-mean)*(tmpmap[6]-mean)+(tmpmap[7]-mean)*(tmpmap[7]-mean)+(tmpmap[8]-mean)*(tmpmap[8]-mean));
                //斜率
                slope=slope/8;
                //traversability map
                score = w_sd*sd + w_sl*slope + w_max*(hmax-_height_mapnew(i,j)) + w_min*(_height_mapnew(i,j)-hmin);
                trav_lcm.map[i][j]=score;
//                cout << "i" << i << " j" << j << " score " << trav_lcm.map[i][j] << " slope: " << slope <<" sd: "<<sd<<endl;
            }
            else if((i==0 || j==0)&&i+1<xnew_size&&j+1<ynew_size) {
                trav_lcm.map[i][j] = 0;
//                cout << "i" << i << " j" << j << " score " << trav_lcm.map[i][j] << endl;
            }
        } // y loop
    } // x loop

    lcm.publish("traversability_float", &trav_lcm);
}
