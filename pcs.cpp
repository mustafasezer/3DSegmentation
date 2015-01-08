#include "pcs.h"

PCS::PCS()
{
    cnt = 0;
}

bool PCS::loadFromFile(string file)
{
    cout<<"loading... "<<endl;
    string line;
    ifstream myfile (file.c_str());
    if (myfile.is_open()){
        pc.reset(new Cloud);
        while ( getline (myfile,line) ){
            vector<string> flds = split(line, ' ');
            PointT point;
            double t = QString(flds.at(0).c_str()).toDouble();
            point.x = QString(flds.at(1).c_str()).toDouble();
            point.y = QString(flds.at(2).c_str()).toDouble();
            point.z = QString(flds.at(3).c_str()).toDouble();
            point.r = QString(flds.at(4).c_str()).toDouble();
            point.g = QString(flds.at(5).c_str()).toDouble();
            point.b = QString(flds.at(6).c_str()).toDouble();

            /*double t = stod(flds.at(0));
            point.x = stod(flds.at(1));
            point.y = stod(flds.at(2));
            point.z = stod(flds.at(3));
            point.r = stod(flds.at(4));
            point.g = stod(flds.at(5));
            point.b = stod(flds.at(6));*/

            if(t == cnt){
                pc->points.push_back(point);
            }
            else if(t == cnt+1){
                // at new time
                pcs.push_back(pc);
                pc.reset(new Cloud);
                pc->points.push_back(point);
                cnt = cnt+1;
            }
        }
        pcs.push_back(pc);
        nTime = pcs.size();
        myfile.close();
    }
    else return 0;

    return 1;
}

vector<string> PCS::split(string source, char delim, int rep) {
    vector<string> flds;
    string work = source;
    string buf = "";
    int i = 0;
    while (i < work.length()) {
        if (work[i] != delim)
            buf += work[i];
        else if (rep == 1) {
            flds.push_back(buf);
            buf = "";
        } else if (buf.length() > 0) {
            flds.push_back(buf);
            buf = "";
        }
        i++;
    }
    if (!buf.empty())
        flds.push_back(buf);
    return flds;
}

void PCS::push(double t, PointT point)
{

}
