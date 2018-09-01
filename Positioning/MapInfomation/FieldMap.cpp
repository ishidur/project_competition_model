#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "FieldMap.h"

using namespace Positioning::MapInfomation;

FieldMap::FieldMap(){

}

int FieldMap::ReadMap(const char* filename){
    char *p;
    char buf[1000];

    int d;
    int x=0,y=0;

    FILE* map_file;
    
    // read csv of map
    if((map_file = fopen(filename,"r"))==NULL){
        printf("fail to open: %s\n", filename);
        return 0;
    }

    // read width & height
    fscanf(map_file,"%d,%d\n",&mapWidth,&mapHeight);

    // split 1 line to each token
    while(fgets(buf, 1000, map_file)!=NULL){
        p = strtok(buf,",");
        while(p!=NULL){
            p = strtok(NULL, ",");
            if(p!=NULL){
              sscanf(p,"%d",&d);
              mapValue[x][mapHeight-y-1] = d;
            }
            x++;
        }
        y++; x=0;
    }
    fclose(map_file);

    printf("finish reading map: %s\n", filename);
    return 1;
}